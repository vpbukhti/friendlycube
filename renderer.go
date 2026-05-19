package main

import (
	"fmt"
	"strings"
	"unsafe"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/mathgl/mgl32"
)

// vertexStride matches the Vertex struct below: pos(3) + nrm(3) + col(3) +
// emiss(1) = 10 float32s.
const vertexStride = 10 * 4

type Vertex struct {
	X, Y, Z    float32
	Nx, Ny, Nz float32
	R, G, B    float32
	Emit       float32 // 0 or 1 — multiplies the self-glow term
}

// Renderer holds the GL resources for one frame's geometry. Geometry is
// re-uploaded whenever the scene is rebuilt.
type Renderer struct {
	program        uint32
	uniMVP         int32
	uniModel       int32
	uniNormalMat   int32
	uniCamPos      int32
	uniLightDirs   int32
	uniLightCols   int32
	uniAmbient     int32

	vao        uint32
	vbo        uint32
	numVerts   int32
	bgColor    [3]float32
}

const vertSrc = `#version 410 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec3 aColor;
layout(location=3) in float aEmit;

uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat3 uNormalMat;

out vec3 vNormal;
out vec3 vWorldPos;
out vec3 vColor;
out float vEmit;

void main() {
  vNormal = normalize(uNormalMat * aNormal);
  vec4 wp = uModel * vec4(aPos, 1.0);
  vWorldPos = wp.xyz;
  vColor = aColor;
  vEmit = aEmit;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
` + "\x00"

const fragSrc = `#version 410 core
in vec3 vNormal;
in vec3 vWorldPos;
in vec3 vColor;
in float vEmit;

uniform vec3 uCamPos;
uniform vec3 uLightDirs[3];
uniform vec3 uLightCols[3];
uniform vec3 uAmbient;

out vec4 fragColor;

void main() {
  vec3 N = normalize(vNormal);
  vec3 V = normalize(uCamPos - vWorldPos);
  vec3 lit = uAmbient * vColor;
  for (int i = 0; i < 3; ++i) {
    vec3 L = normalize(-uLightDirs[i]);
    float ndl = max(dot(N, L), 0.0);
    // gentle wrap so the back side isn't pitch black on lit-from-one-side
    float wrap = max(dot(N, L) * 0.5 + 0.5, 0.0);
    vec3 diffuse = uLightCols[i] * (0.7 * ndl + 0.3 * wrap) * vColor;
    // subtle spec for a hint of marble polish
    vec3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0);
    float spec = pow(ndh, 32.0) * 0.15;
    lit += diffuse + uLightCols[i] * spec;
  }
  vec3 emissive = vColor * 0.15 * vEmit;
  vec3 outc = lit + emissive;
  // gentle tone curve / gamma-ish
  outc = pow(outc, vec3(1.0/1.6));
  fragColor = vec4(outc, 1.0);
}
` + "\x00"

func compileShader(src string, kind uint32) (uint32, error) {
	sh := gl.CreateShader(kind)
	cs, free := gl.Strs(src)
	gl.ShaderSource(sh, 1, cs, nil)
	free()
	gl.CompileShader(sh)
	var status int32
	gl.GetShaderiv(sh, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var ln int32
		gl.GetShaderiv(sh, gl.INFO_LOG_LENGTH, &ln)
		log := strings.Repeat("\x00", int(ln+1))
		gl.GetShaderInfoLog(sh, ln, nil, gl.Str(log))
		return 0, fmt.Errorf("shader compile failed: %s", log)
	}
	return sh, nil
}

func NewRenderer(bg [3]float32) (*Renderer, error) {
	vs, err := compileShader(vertSrc, gl.VERTEX_SHADER)
	if err != nil {
		return nil, err
	}
	fs, err := compileShader(fragSrc, gl.FRAGMENT_SHADER)
	if err != nil {
		return nil, err
	}
	prog := gl.CreateProgram()
	gl.AttachShader(prog, vs)
	gl.AttachShader(prog, fs)
	gl.LinkProgram(prog)
	var status int32
	gl.GetProgramiv(prog, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var ln int32
		gl.GetProgramiv(prog, gl.INFO_LOG_LENGTH, &ln)
		log := strings.Repeat("\x00", int(ln+1))
		gl.GetProgramInfoLog(prog, ln, nil, gl.Str(log))
		return nil, fmt.Errorf("program link failed: %s", log)
	}
	gl.DeleteShader(vs)
	gl.DeleteShader(fs)

	r := &Renderer{program: prog, bgColor: bg}
	r.uniMVP = gl.GetUniformLocation(prog, gl.Str("uMVP\x00"))
	r.uniModel = gl.GetUniformLocation(prog, gl.Str("uModel\x00"))
	r.uniNormalMat = gl.GetUniformLocation(prog, gl.Str("uNormalMat\x00"))
	r.uniCamPos = gl.GetUniformLocation(prog, gl.Str("uCamPos\x00"))
	r.uniLightDirs = gl.GetUniformLocation(prog, gl.Str("uLightDirs\x00"))
	r.uniLightCols = gl.GetUniformLocation(prog, gl.Str("uLightCols\x00"))
	r.uniAmbient = gl.GetUniformLocation(prog, gl.Str("uAmbient\x00"))

	gl.GenVertexArrays(1, &r.vao)
	gl.GenBuffers(1, &r.vbo)
	gl.BindVertexArray(r.vao)
	gl.BindBuffer(gl.ARRAY_BUFFER, r.vbo)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointerWithOffset(0, 3, gl.FLOAT, false, vertexStride, 0)
	gl.EnableVertexAttribArray(1)
	gl.VertexAttribPointerWithOffset(1, 3, gl.FLOAT, false, vertexStride, 3*4)
	gl.EnableVertexAttribArray(2)
	gl.VertexAttribPointerWithOffset(2, 3, gl.FLOAT, false, vertexStride, 6*4)
	gl.EnableVertexAttribArray(3)
	gl.VertexAttribPointerWithOffset(3, 1, gl.FLOAT, false, vertexStride, 9*4)
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)

	gl.Enable(gl.DEPTH_TEST)
	gl.Enable(gl.CULL_FACE)
	gl.CullFace(gl.BACK)
	gl.ClearColor(bg[0], bg[1], bg[2], 1.0)
	return r, nil
}

// Upload flattens a scene Group into the renderer's VBO. Each batch is the
// same material so we expand colors per vertex and bind once.
func (r *Renderer) Upload(root *Group) {
	batches := root.Flatten()
	totalTris := 0
	for _, b := range batches {
		totalTris += len(b.Tris)
	}
	verts := make([]Vertex, 0, totalTris*3)
	for _, b := range batches {
		emit := float32(0)
		if b.Mat.Emissive {
			emit = 1
		}
		c := b.Mat.Color
		for _, t := range b.Tris {
			verts = append(verts,
				Vertex{float32(t.A[0]), float32(t.A[1]), float32(t.A[2]),
					float32(t.NA[0]), float32(t.NA[1]), float32(t.NA[2]),
					c[0], c[1], c[2], emit},
				Vertex{float32(t.B[0]), float32(t.B[1]), float32(t.B[2]),
					float32(t.NB[0]), float32(t.NB[1]), float32(t.NB[2]),
					c[0], c[1], c[2], emit},
				Vertex{float32(t.C[0]), float32(t.C[1]), float32(t.C[2]),
					float32(t.NC[0]), float32(t.NC[1]), float32(t.NC[2]),
					c[0], c[1], c[2], emit},
			)
		}
	}
	r.numVerts = int32(len(verts))
	gl.BindBuffer(gl.ARRAY_BUFFER, r.vbo)
	if r.numVerts > 0 {
		gl.BufferData(gl.ARRAY_BUFFER, len(verts)*int(unsafe.Sizeof(Vertex{})), gl.Ptr(verts), gl.DYNAMIC_DRAW)
	}
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
}

// Draw renders the uploaded geometry with the given camera + sculpture rotation.
func (r *Renderer) Draw(width, height int, rotX, rotY float32, camPos mgl32.Vec3) {
	gl.Viewport(0, 0, int32(width), int32(height))
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(r.program)

	// Build matrices.
	aspect := float32(width) / float32(height)
	if aspect == 0 {
		aspect = 1
	}
	proj := mgl32.Perspective(mgl32.DegToRad(38), aspect, 0.1, 100)
	view := mgl32.LookAtV(camPos, mgl32.Vec3{0, 0, 0}, mgl32.Vec3{0, 1, 0})
	model := mgl32.HomogRotate3DX(rotX).Mul4(mgl32.HomogRotate3DY(rotY))
	mvp := proj.Mul4(view).Mul4(model)

	// Normal matrix = transpose(inverse(model3x3)). For pure rotations
	// (which is all we apply at the model level), it's identical to model3x3.
	nm := mgl32.Mat3{
		model[0], model[1], model[2],
		model[4], model[5], model[6],
		model[8], model[9], model[10],
	}

	gl.UniformMatrix4fv(r.uniMVP, 1, false, &mvp[0])
	gl.UniformMatrix4fv(r.uniModel, 1, false, &model[0])
	gl.UniformMatrix3fv(r.uniNormalMat, 1, false, &nm[0])
	gl.Uniform3f(r.uniCamPos, camPos[0], camPos[1], camPos[2])

	// Three-light setup mirroring the JS: warm key, cool fill, warm rim
	// (directions point FROM the source toward the scene; the fragment
	// shader negates).
	dirs := [9]float32{
		-5, -8, -4,
		4, -2, 3,
		0, 4, 5,
	}
	cols := [9]float32{
		1.0, 0.96, 0.88,
		0.79, 0.85, 0.91,
		1.0, 0.91, 0.77,
	}
	// Scale each light by its intensity (0.9 / 0.4 / 0.3).
	intens := [3]float32{0.9, 0.4, 0.3}
	for i := 0; i < 3; i++ {
		cols[i*3+0] *= intens[i]
		cols[i*3+1] *= intens[i]
		cols[i*3+2] *= intens[i]
	}
	gl.Uniform3fv(r.uniLightDirs, 3, &dirs[0])
	gl.Uniform3fv(r.uniLightCols, 3, &cols[0])
	gl.Uniform3f(r.uniAmbient, 0.5, 0.5, 0.5)

	gl.BindVertexArray(r.vao)
	gl.DrawArrays(gl.TRIANGLES, 0, r.numVerts)
	gl.BindVertexArray(0)
}
