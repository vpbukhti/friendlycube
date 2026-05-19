package main

import "math"

// Primitive is anything with a signed-distance evaluation. Negative inside,
// zero on the surface, positive outside.
type Primitive interface {
	Dist(p Vec3) float64
	Bounds() AABB
}

type AABB struct {
	Min, Max Vec3
}

func aabbFromPoint(c Vec3, r float64) AABB {
	return AABB{Min: V(c[0]-r, c[1]-r, c[2]-r), Max: V(c[0]+r, c[1]+r, c[2]+r)}
}

func aabbUnion(a, b AABB) AABB {
	return AABB{
		Min: V(math.Min(a.Min[0], b.Min[0]), math.Min(a.Min[1], b.Min[1]), math.Min(a.Min[2], b.Min[2])),
		Max: V(math.Max(a.Max[0], b.Max[0]), math.Max(a.Max[1], b.Max[1]), math.Max(a.Max[2], b.Max[2])),
	}
}

func aabbExpand(a AABB, m float64) AABB {
	return AABB{Min: V(a.Min[0]-m, a.Min[1]-m, a.Min[2]-m), Max: V(a.Max[0]+m, a.Max[1]+m, a.Max[2]+m)}
}

// Sphere: |p - C| - r.
type Sphere struct {
	Center Vec3
	Radius float64
}

func (s Sphere) Dist(p Vec3) float64 { return p.Sub(s.Center).Len() - s.Radius }
func (s Sphere) Bounds() AABB        { return aabbFromPoint(s.Center, s.Radius) }

// RoundedBox: Minkowski-rounded cube — an inner box of half-extents
// HalfSize, expanded by R. Total outer half-extent on each axis is
// HalfSize + R. Used as a clip primitive (smooth-max'd into the field) so
// the outer cube reads as a slightly-rounded d6: flat faces, quarter-
// cylinder edges of radius R, 1/8-sphere corners of radius R.
type RoundedBox struct {
	HalfSize Vec3
	R        float64
}

func (b RoundedBox) Dist(p Vec3) float64 {
	qx := math.Abs(p[0]) - b.HalfSize[0]
	qy := math.Abs(p[1]) - b.HalfSize[1]
	qz := math.Abs(p[2]) - b.HalfSize[2]
	mx := math.Max(qx, 0)
	my := math.Max(qy, 0)
	mz := math.Max(qz, 0)
	outer := math.Sqrt(mx*mx + my*my + mz*mz)
	inner := math.Min(math.Max(math.Max(qx, qy), qz), 0)
	return outer + inner - b.R
}

func (b RoundedBox) Bounds() AABB {
	hx, hy, hz := b.HalfSize[0]+b.R, b.HalfSize[1]+b.R, b.HalfSize[2]+b.R
	return AABB{Min: V(-hx, -hy, -hz), Max: V(hx, hy, hz)}
}

// Capsule: distance to segment A→B, minus radius.
type Capsule struct {
	A, B   Vec3
	Radius float64
}

func (c Capsule) Dist(p Vec3) float64 {
	ab := c.B.Sub(c.A)
	ap := p.Sub(c.A)
	abLen2 := ab.Dot(ab)
	t := 0.0
	if abLen2 > 1e-18 {
		t = clampF(ap.Dot(ab)/abLen2, 0, 1)
	}
	q := c.A.Add(ab.Mul(t))
	return p.Sub(q).Len() - c.Radius
}

func (c Capsule) Bounds() AABB {
	rA := aabbFromPoint(c.A, c.Radius)
	rB := aabbFromPoint(c.B, c.Radius)
	return aabbUnion(rA, rB)
}

// smin: smooth-minimum from the user's spec. Implementation note: directly
// computing exp(-k·a) underflows when a is large, which causes -log to blow
// up. Subtract the smaller distance from both before the exp so the larger
// arg is clamped at zero (numerically stable; mathematically identical).
func smin(a, b, k float64) float64 {
	m := math.Min(a, b)
	ea := math.Exp(-k * (a - m))
	eb := math.Exp(-k * (b - m))
	return m - math.Log(ea+eb)/k
}

// smax: smooth-maximum via the standard identity. Used to intersect the
// smin'd field with a clip primitive (negative inside the clip shape).
func smax(a, b, k float64) float64 {
	return -smin(-a, -b, k)
}

// Field is the signed-distance function that smooth-mins every primitive in
// `Prims`. The K parameter controls joint sharpness: large K → sharp/boolean
// look; small K → soft / organic.
//
// Clip is an optional primitive that the resulting field is smooth-MAX'd
// against. The composite shape is then `(union of Prims) ∩ (interior of
// Clip)` — i.e., the union of all primitives clipped to fit inside the clip
// shape. Used here to wrap the strut wireframe in a rounded-cube envelope.
type Field struct {
	Prims []Primitive
	Clip  Primitive
	K     float64
	bvh   *bvhNode // built lazily on first eval
}

// Build eagerly constructs the BVH so concurrent Eval callers don't race.
// Idempotent.
func (f *Field) Build() {
	if f.bvh != nil {
		return
	}
	f.bvh = buildBVH(f.Prims)
}

func (f *Field) Eval(p Vec3) float64 {
	const initial = 1e9
	v := initial
	// We seed with a hard min from the BVH to cull primitives whose bounding
	// boxes are far enough that they can't influence the smooth-min within
	// the kernel's effective range (~3/K). Once we have a tight hard min,
	// only nearby primitives meaningfully contribute.
	hardMin := bvhHardMin(f.bvh, p, initial)
	if hardMin > 5.0/f.K {
		// Far-from-anything fast path: smooth-min ≈ hard-min, skip the
		// second BVH descent entirely.
		v = hardMin
	} else {
		cutoff := hardMin + 3.0/f.K // e^{-k·(d-hardMin)} < e^-3 ≈ 0.05 past this
		v = bvhSmoothMin(f.bvh, p, v, cutoff, f.K)
		if v >= initial-1 {
			v = hardMin
		}
	}
	if f.Clip != nil {
		// Hard max (not smooth) for the clip. The outer envelope is meant to
		// be a sharp boundary; smoothing it would erode features whose scale
		// is comparable to or smaller than the kernel width 3/K, which is
		// exactly the case for the thin d6 edge fillets. The strut surface
		// and the rounded-cube fillet are tangent at the meeting ring, so
		// the seam is C¹ smooth in practice anyway.
		cd := f.Clip.Dist(p)
		if cd > v {
			v = cd
		}
	}
	return v
}

// Gradient via central differences. Used for per-vertex normals on the
// extracted mesh.
func (f *Field) Gradient(p Vec3, eps float64) Vec3 {
	dx := f.Eval(V(p[0]+eps, p[1], p[2])) - f.Eval(V(p[0]-eps, p[1], p[2]))
	dy := f.Eval(V(p[0], p[1]+eps, p[2])) - f.Eval(V(p[0], p[1]-eps, p[2]))
	dz := f.Eval(V(p[0], p[1], p[2]+eps)) - f.Eval(V(p[0], p[1], p[2]-eps))
	return V(dx, dy, dz).Normalize()
}

// ---- BVH ----
// Bog-standard top-down AABB BVH. Built once per Field, queried many times.

type bvhNode struct {
	box         AABB
	left, right *bvhNode
	prims       []Primitive // only set on leaves
}

func buildBVH(prims []Primitive) *bvhNode {
	if len(prims) == 0 {
		return &bvhNode{box: AABB{Min: V(0, 0, 0), Max: V(0, 0, 0)}}
	}
	box := prims[0].Bounds()
	for _, p := range prims[1:] {
		box = aabbUnion(box, p.Bounds())
	}
	if len(prims) <= 4 {
		return &bvhNode{box: box, prims: prims}
	}
	// Split on the longest axis at the median centroid coordinate.
	ext := V(box.Max[0]-box.Min[0], box.Max[1]-box.Min[1], box.Max[2]-box.Min[2])
	axis := 0
	if ext[1] > ext[0] && ext[1] >= ext[2] {
		axis = 1
	} else if ext[2] > ext[0] {
		axis = 2
	}
	// Median split — quick and adequate for our ~50–200 primitive counts.
	prims = append([]Primitive(nil), prims...)
	quickSortByCentroid(prims, axis)
	mid := len(prims) / 2
	return &bvhNode{
		box:   box,
		left:  buildBVH(prims[:mid]),
		right: buildBVH(prims[mid:]),
	}
}

func centroidAxis(p Primitive, axis int) float64 {
	b := p.Bounds()
	return 0.5 * (b.Min[axis] + b.Max[axis])
}

func quickSortByCentroid(prims []Primitive, axis int) {
	if len(prims) < 2 {
		return
	}
	pivot := centroidAxis(prims[len(prims)/2], axis)
	i, j := 0, len(prims)-1
	for i <= j {
		for centroidAxis(prims[i], axis) < pivot {
			i++
		}
		for centroidAxis(prims[j], axis) > pivot {
			j--
		}
		if i <= j {
			prims[i], prims[j] = prims[j], prims[i]
			i++
			j--
		}
	}
	quickSortByCentroid(prims[:j+1], axis)
	quickSortByCentroid(prims[i:], axis)
}

// distToAABB: returns 0 if p is inside, else the Euclidean distance to the
// box surface (a conservative under-estimate for the actual primitive's SDF).
func distToAABB(p Vec3, box AABB) float64 {
	var d float64
	for i := 0; i < 3; i++ {
		if p[i] < box.Min[i] {
			d += (box.Min[i] - p[i]) * (box.Min[i] - p[i])
		} else if p[i] > box.Max[i] {
			d += (p[i] - box.Max[i]) * (p[i] - box.Max[i])
		}
	}
	return math.Sqrt(d)
}

func bvhHardMin(n *bvhNode, p Vec3, current float64) float64 {
	if n == nil {
		return current
	}
	// Prune if this whole subtree is farther than current best.
	if distToAABB(p, n.box) >= current {
		return current
	}
	if n.prims != nil {
		for _, pr := range n.prims {
			d := pr.Dist(p)
			if d < current {
				current = d
			}
		}
		return current
	}
	// Visit the closer child first for better pruning.
	dL := distToAABB(p, n.left.box)
	dR := distToAABB(p, n.right.box)
	if dL < dR {
		current = bvhHardMin(n.left, p, current)
		current = bvhHardMin(n.right, p, current)
	} else {
		current = bvhHardMin(n.right, p, current)
		current = bvhHardMin(n.left, p, current)
	}
	return current
}

func bvhSmoothMin(n *bvhNode, p Vec3, current, cutoff, k float64) float64 {
	if n == nil {
		return current
	}
	// Conservative cull: if the box is farther than cutoff, no primitive in
	// it can be within the smooth-min kernel's effective support.
	if distToAABB(p, n.box) > cutoff {
		return current
	}
	if n.prims != nil {
		for _, pr := range n.prims {
			d := pr.Dist(p)
			if d > cutoff {
				continue
			}
			current = smin(current, d, k)
		}
		return current
	}
	current = bvhSmoothMin(n.left, p, current, cutoff, k)
	current = bvhSmoothMin(n.right, p, current, cutoff, k)
	return current
}
