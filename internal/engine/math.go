package engine

import "math"

type Vec3 [3]float64

func V(x, y, z float64) Vec3 { return Vec3{x, y, z} }

func (a Vec3) Add(b Vec3) Vec3    { return Vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]} }
func (a Vec3) Sub(b Vec3) Vec3    { return Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]} }
func (a Vec3) Mul(s float64) Vec3 { return Vec3{a[0] * s, a[1] * s, a[2] * s} }
func (a Vec3) Neg() Vec3          { return Vec3{-a[0], -a[1], -a[2]} }
func (a Vec3) Dot(b Vec3) float64 { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}
func (a Vec3) Len() float64  { return math.Sqrt(a.Dot(a)) }
func (a Vec3) Len2() float64 { return a.Dot(a) }
func (a Vec3) Normalize() Vec3 {
	l := a.Len()
	if l < 1e-12 {
		return Vec3{0, 1, 0}
	}
	return a.Mul(1.0 / l)
}
func (a Vec3) DistTo(b Vec3) float64 { return a.Sub(b).Len() }
func (a Vec3) Equals(b Vec3) bool {
	return math.Abs(a[0]-b[0]) < 1e-9 && math.Abs(a[1]-b[1]) < 1e-9 && math.Abs(a[2]-b[2]) < 1e-9
}
func Lerp(a, b Vec3, t float64) Vec3 { return a.Add(b.Sub(a).Mul(t)) }

// Quaternion as [x, y, z, w]
type Quat [4]float64

func QuatIdentity() Quat { return Quat{0, 0, 0, 1} }

// QuatFromUnitVectors mirrors three.js Quaternion.setFromUnitVectors:
// shortest-arc rotation taking `from` to `to` (both must be unit-length).
func QuatFromUnitVectors(from, to Vec3) Quat {
	r := from.Dot(to) + 1
	if r < 1e-9 {
		// 180 degree rotation; pick any orthogonal axis.
		if math.Abs(from[0]) > math.Abs(from[2]) {
			q := Quat{-from[1], from[0], 0, 0}
			return quatNormalize(q)
		}
		q := Quat{0, -from[2], from[1], 0}
		return quatNormalize(q)
	}
	c := from.Cross(to)
	q := Quat{c[0], c[1], c[2], r}
	return quatNormalize(q)
}

func quatNormalize(q Quat) Quat {
	l := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	if l < 1e-12 {
		return Quat{0, 0, 0, 1}
	}
	return Quat{q[0] / l, q[1] / l, q[2] / l, q[3] / l}
}

// QuatFromAxisAngle returns a rotation by `angle` (radians) about a unit axis.
func QuatFromAxisAngle(axis Vec3, angle float64) Quat {
	half := angle * 0.5
	s := math.Sin(half)
	return Quat{axis[0] * s, axis[1] * s, axis[2] * s, math.Cos(half)}
}

// QuatMul applies a*b (a first, then b is applied to the result — i.e. b∘a).
// We use the three.js convention: q.multiply(p) means q = q * p (q then p).
func QuatMul(a, b Quat) Quat {
	return Quat{
		a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
		a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
		a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
		a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
	}
}

// RotateVec applies a quaternion to a vector.
func (q Quat) RotateVec(v Vec3) Vec3 {
	// v' = q * v * q^-1, expanded.
	x, y, z, w := q[0], q[1], q[2], q[3]
	tx := 2 * (y*v[2] - z*v[1])
	ty := 2 * (z*v[0] - x*v[2])
	tz := 2 * (x*v[1] - y*v[0])
	return Vec3{
		v[0] + w*tx + (y*tz - z*ty),
		v[1] + w*ty + (z*tx - x*tz),
		v[2] + w*tz + (x*ty - y*tx),
	}
}

// Transform: scale (per-axis), then rotate (quaternion), then translate.
type Transform struct {
	Position Vec3
	Rotation Quat
	Scale    Vec3
}

func IdentityTransform() Transform {
	return Transform{Position: V(0, 0, 0), Rotation: QuatIdentity(), Scale: V(1, 1, 1)}
}

// Apply maps a local-space point through the transform.
func (t Transform) Apply(v Vec3) Vec3 {
	scaled := Vec3{v[0] * t.Scale[0], v[1] * t.Scale[1], v[2] * t.Scale[2]}
	rotated := t.Rotation.RotateVec(scaled)
	return rotated.Add(t.Position)
}

// ApplyDir maps a direction (no translation, but does scale-rotate; for
// uniform scale this is fine for normals — we renormalize after).
func (t Transform) ApplyDir(v Vec3) Vec3 {
	scaled := Vec3{v[0] * t.Scale[0], v[1] * t.Scale[1], v[2] * t.Scale[2]}
	return t.Rotation.RotateVec(scaled)
}

// Compose applies `parent` after `child` — parent ∘ child.
func Compose(parent, child Transform) Transform {
	pos := parent.Apply(child.Position)
	rot := QuatMul(parent.Rotation, child.Rotation)
	scl := Vec3{parent.Scale[0] * child.Scale[0], parent.Scale[1] * child.Scale[1], parent.Scale[2] * child.Scale[2]}
	return Transform{Position: pos, Rotation: rot, Scale: scl}
}

func clampF(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
