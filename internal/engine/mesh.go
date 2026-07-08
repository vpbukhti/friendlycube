package engine

import "math"

// Triangle stored in world space, with a per-vertex shared normal (flat
// normals are computed at upload time if we want per-face shading).
type Triangle struct {
	A, B, C    Vec3
	NA, NB, NC Vec3
}

// MeshBuffers flattens a scene Group into flat, non-indexed float32 attribute
// arrays for a WebGL client: per triangle vertex, a position (3), a normal (3),
// and a color (3, the batch material). Lengths are all 9 × triangleCount.
func MeshBuffers(g *Group) (pos, nrm, col []float32) {
	batches := g.Flatten()
	verts := 0
	for _, b := range batches {
		verts += len(b.Tris) * 3
	}
	pos = make([]float32, 0, verts*3)
	nrm = make([]float32, 0, verts*3)
	col = make([]float32, 0, verts*3)
	for _, b := range batches {
		c := b.Mat.Color
		for _, t := range b.Tris {
			for _, v := range [3]struct{ p, n Vec3 }{{t.A, t.NA}, {t.B, t.NB}, {t.C, t.NC}} {
				pos = append(pos, float32(v.p[0]), float32(v.p[1]), float32(v.p[2]))
				nrm = append(nrm, float32(v.n[0]), float32(v.n[1]), float32(v.n[2]))
				col = append(col, c[0], c[1], c[2])
			}
		}
	}
	return pos, nrm, col
}

// Mesh is a list of triangles in some local frame, plus its material.
type Mesh struct {
	Tris []Triangle
	Mat  Material
}

// Material is a simple Phong-ish description.
type Material struct {
	Color    [3]float32 // base color (linear-ish, treated sRGB-ish — good enough for preview)
	Emissive bool       // if true, add color * 0.15 as a self-glow term in the shader
}

// hex2rgb expands a 0xRRGGBB color int into 3 normalized float32 channels.
func hex2rgb(hex int) [3]float32 {
	r := float32((hex>>16)&0xff) / 255.0
	g := float32((hex>>8)&0xff) / 255.0
	b := float32(hex&0xff) / 255.0
	return [3]float32{r, g, b}
}

func newMat(hex int, emissive bool) Material {
	return Material{Color: hex2rgb(hex), Emissive: emissive}
}

// transformMesh: bake a mesh's triangles through a transform (positions and
// normals).
func transformMesh(m *Mesh, t Transform, out *[]Triangle) {
	// For uniform scale, rotating normals is fine without inverse-transpose.
	// Our scales are uniform everywhere EXCEPT the palm (1.05, 0.55, 0.85) and
	// the knuckle (1.1, 0.7, 1.0). For those, normals will be slightly off
	// but visually unimportant for a preview.
	for _, tr := range m.Tris {
		nt := Triangle{
			A:  t.Apply(tr.A),
			B:  t.Apply(tr.B),
			C:  t.Apply(tr.C),
			NA: t.ApplyDir(tr.NA).Normalize(),
			NB: t.ApplyDir(tr.NB).Normalize(),
			NC: t.ApplyDir(tr.NC).Normalize(),
		}
		*out = append(*out, nt)
	}
}

// makeCylinder mirrors three.js CylinderGeometry(radiusTop=top, radiusBot=bot,
// height, radialSegments, heightSegments=1). Axis is Y. Centered at origin.
// We always include both end caps.
func makeCylinder(radiusTop, radiusBot, height float64, radialSegments int) []Triangle {
	if radialSegments < 3 {
		radialSegments = 3
	}
	tris := make([]Triangle, 0, radialSegments*4)
	halfH := height / 2
	// Pre-compute the per-segment angle table.
	cs := make([]float64, radialSegments+1)
	sn := make([]float64, radialSegments+1)
	for i := 0; i <= radialSegments; i++ {
		theta := 2 * math.Pi * float64(i) / float64(radialSegments)
		cs[i] = math.Cos(theta)
		sn[i] = math.Sin(theta)
	}
	// Side normal in three.js cylinder: slope between top/bot radii. For our
	// purposes (thin pads, slight tapers), a flat radial normal is fine.
	slope := (radiusBot - radiusTop) / height
	for i := 0; i < radialSegments; i++ {
		c0, s0 := cs[i], sn[i]
		c1, s1 := cs[i+1], sn[i+1]
		// 4 corners of the side quad
		ptl := V(radiusTop*c0, halfH, radiusTop*s0)
		ptr := V(radiusTop*c1, halfH, radiusTop*s1)
		pbl := V(radiusBot*c0, -halfH, radiusBot*s0)
		pbr := V(radiusBot*c1, -halfH, radiusBot*s1)
		// Normals: radial + slope component (three.js does this).
		// With our parametrization x = r·cosθ, z = r·sinθ, going from i to
		// i+1 traces CCW in XZ when viewed from +Y. That means the side
		// quad's correct outside-CCW order is ptl → ptr → pbl (not the
		// ptl → pbl → ptr I had originally — that's CW from outside,
		// which got culled and made cylinders read with inverted lighting).
		n0 := V(c0, slope, s0).Normalize()
		n1 := V(c1, slope, s1).Normalize()
		tris = append(tris,
			Triangle{A: ptl, B: ptr, C: pbl, NA: n0, NB: n1, NC: n0},
			Triangle{A: ptr, B: pbr, C: pbl, NA: n1, NB: n1, NC: n0},
		)
	}
	// End caps. Same winding gotcha: from above (+Y) the top cap should be
	// CCW with our θ direction, which means center → next → this.
	if radiusTop > 1e-9 {
		nTop := V(0, 1, 0)
		cTop := V(0, halfH, 0)
		for i := 0; i < radialSegments; i++ {
			p0 := V(radiusTop*cs[i], halfH, radiusTop*sn[i])
			p1 := V(radiusTop*cs[i+1], halfH, radiusTop*sn[i+1])
			tris = append(tris, Triangle{A: cTop, B: p1, C: p0, NA: nTop, NB: nTop, NC: nTop})
		}
	}
	if radiusBot > 1e-9 {
		nBot := V(0, -1, 0)
		cBot := V(0, -halfH, 0)
		for i := 0; i < radialSegments; i++ {
			p0 := V(radiusBot*cs[i], -halfH, radiusBot*sn[i])
			p1 := V(radiusBot*cs[i+1], -halfH, radiusBot*sn[i+1])
			tris = append(tris, Triangle{A: cBot, B: p0, C: p1, NA: nBot, NB: nBot, NC: nBot})
		}
	}
	return tris
}

// makeSphere: UV sphere, axis Y, three.js SphereGeometry(radius, wSeg, hSeg).
func makeSphere(radius float64, widthSegs, heightSegs int) []Triangle {
	if widthSegs < 3 {
		widthSegs = 3
	}
	if heightSegs < 2 {
		heightSegs = 2
	}
	// Build vertex grid then triangulate. Pole rings collapse.
	grid := make([][]Vec3, heightSegs+1)
	for j := 0; j <= heightSegs; j++ {
		row := make([]Vec3, widthSegs+1)
		phi := math.Pi * float64(j) / float64(heightSegs)
		sinPhi, cosPhi := math.Sin(phi), math.Cos(phi)
		for i := 0; i <= widthSegs; i++ {
			theta := 2 * math.Pi * float64(i) / float64(widthSegs)
			x := -math.Cos(theta) * sinPhi * radius
			y := cosPhi * radius
			z := math.Sin(theta) * sinPhi * radius
			row[i] = V(x, y, z)
		}
		grid[j] = row
	}
	tris := make([]Triangle, 0, widthSegs*heightSegs*2)
	for j := 0; j < heightSegs; j++ {
		for i := 0; i < widthSegs; i++ {
			v00 := grid[j][i]
			v01 := grid[j][i+1]
			v10 := grid[j+1][i]
			v11 := grid[j+1][i+1]
			n00 := v00.Normalize()
			n01 := v01.Normalize()
			n10 := v10.Normalize()
			n11 := v11.Normalize()
			if j != 0 {
				tris = append(tris, Triangle{A: v00, B: v10, C: v01, NA: n00, NB: n10, NC: n01})
			}
			if j != heightSegs-1 {
				tris = append(tris, Triangle{A: v01, B: v10, C: v11, NA: n01, NB: n10, NC: n11})
			}
		}
	}
	return tris
}

// makeOctahedron: three.js OctahedronGeometry(radius) — 8-faced regular
// octahedron with vertices at ±radius along each axis.
func makeOctahedron(radius float64) []Triangle {
	r := radius
	p := V(r, 0, 0)
	n := V(-r, 0, 0)
	u := V(0, r, 0)
	d := V(0, -r, 0)
	f := V(0, 0, r)
	b := V(0, 0, -r)
	faces := [][3]Vec3{
		{p, u, f}, {f, u, n}, {n, u, b}, {b, u, p},
		{p, f, d}, {f, n, d}, {n, b, d}, {b, p, d},
	}
	tris := make([]Triangle, 0, 8)
	for _, fc := range faces {
		nrm := fc[1].Sub(fc[0]).Cross(fc[2].Sub(fc[0])).Normalize()
		tris = append(tris, Triangle{A: fc[0], B: fc[1], C: fc[2], NA: nrm, NB: nrm, NC: nrm})
	}
	return tris
}
