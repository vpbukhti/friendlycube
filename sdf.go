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

// RoundedDie: a rounded box with a SEPARATELY-sized 1/8-sphere bulge at each
// of the 8 vertices. Edge fillets stay at EdgeR (quarter-cylinder of radius
// EdgeR along each cube edge); vertex bulges have radius VertexR centered
// exactly on each cube vertex. When VertexR > EdgeR the corners visibly
// bulge *outward* beyond the natural rounded-box corners — the die look
// where each corner reads as a soft ball, not a flat chamfer.
//
// SDF = min(rounded_box_with_EdgeR, sphere_at_nearest_vertex_with_VertexR).
// At the seam (where the corner sphere meets the edge cylinder) there's a
// small geometric ridge — visually a faint crease, no worse than the small
// crease between cap and cylinder on a real die.
type RoundedDie struct {
	HalfSize Vec3
	EdgeR    float64
	VertexR  float64
}

func (b RoundedDie) Dist(p Vec3) float64 {
	qx := math.Abs(p[0]) - b.HalfSize[0]
	qy := math.Abs(p[1]) - b.HalfSize[1]
	qz := math.Abs(p[2]) - b.HalfSize[2]
	mx := math.Max(qx, 0)
	my := math.Max(qy, 0)
	mz := math.Max(qz, 0)
	outer := math.Sqrt(mx*mx + my*my + mz*mz)
	inner := math.Min(math.Max(math.Max(qx, qy), qz), 0)
	boxSDF := outer + inner - b.EdgeR
	if b.VertexR <= b.EdgeR {
		return boxSDF
	}
	// Distance to the nearest of the 8 cube vertices.
	dx := p[0] - signF(p[0])*b.HalfSize[0]
	dy := p[1] - signF(p[1])*b.HalfSize[1]
	dz := p[2] - signF(p[2])*b.HalfSize[2]
	sphereSDF := math.Sqrt(dx*dx+dy*dy+dz*dz) - b.VertexR
	if sphereSDF < boxSDF {
		return sphereSDF
	}
	return boxSDF
}

func (b RoundedDie) Bounds() AABB {
	maxR := math.Max(b.EdgeR, b.VertexR)
	hx, hy, hz := b.HalfSize[0]+maxR, b.HalfSize[1]+maxR, b.HalfSize[2]+maxR
	return AABB{Min: V(-hx, -hy, -hz), Max: V(hx, hy, hz)}
}

func signF(x float64) float64 {
	if x < 0 {
		return -1
	}
	return 1
}

// Plane: half-space SDF. Negative on the "inside" side (where p·Normal <
// Offset), positive on the outside. Used as a clip primitive to cut a
// corner with a flat plane perpendicular to a chosen direction.
type Plane struct {
	Normal Vec3    // unit-length
	Offset float64 // signed distance from origin along Normal
}

func (pl Plane) Dist(p Vec3) float64 {
	return p.Dot(pl.Normal) - pl.Offset
}

func (pl Plane) Bounds() AABB {
	// Planes are infinite; we hand back a huge AABB so any BVH that
	// happens to see one keeps it always-relevant. In practice Planes are
	// added to Field.Clips, which sit outside the BVH and are evaluated
	// every voxel anyway.
	big := 1e9
	return AABB{Min: V(-big, -big, -big), Max: V(big, big, big)}
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

// sminPoly: polynomial smooth-minimum (Inigo Quilez's quadratic form) with an
// explicit blend width k (a length, not a stiffness). Unlike the exponential
// smin above, k here is the fillet radius directly, which is exactly what the
// corner-cut wants: "round the rim over a fillet width m". Reduces to a hard
// min at k <= 0.
func sminPoly(a, b, k float64) float64 {
	if k <= 0 {
		return math.Min(a, b)
	}
	h := clampF(0.5+0.5*(b-a)/k, 0, 1)
	return b*(1-h) + a*h - k*h*(1-h)
}

// smaxPoly: polynomial smooth-maximum. Crucially, this only ever INCREASES the
// field relative to a hard max — so a corner cut built on it can only move the
// surface inward, never bulge it outward past the tube envelope.
func smaxPoly(a, b, k float64) float64 {
	return -sminPoly(-a, -b, k)
}

// CornerMorph reshapes the field at the 8 cube vertices, morphing each corner
// across sharp → round → flat. It assumes an axis-aligned cube centered at the
// origin with half-extent Half, so the nearest vertex to a query point is just
// (sign(x)·Half, sign(y)·Half, sign(z)·Half) and only that one corner is ever
// relevant (the effect is local to a radius-R neighborhood of each vertex).
//
// Sharp/round side (Power > 0): union a power-norm solid. Writing a_i for the
// perpendicular distance from p to the line along edge i through the vertex,
// the solid is (a_x^p + a_y^p + a_z^p)^(1/p) ≤ R. At p→∞ this is the sharp
// miter — the intersection of the three infinite edge cylinders (a Steinmetz
// solid), whose walls extend the tubes seamlessly. Lowering p rounds the
// creases and tip; by p≈5.4 the solid has shrunk inside the baseline sphere
// cap and the union is a no-op, i.e. the natural round corner. Because the
// solid is always contained in max(a_x,a_y,a_z) ≤ R, it can never protrude past
// the tube walls.
//
// Flat side (Cut): intersect with a plane h(p) = (p−v)·n̂ − δ perpendicular to
// the corner diagonal, via smooth-max so the rim is filleted over width Fillet.
// δ = R is tangent to the corner tip (cuts nothing); smaller δ truncates deeper.
type CornerMorph struct {
	Half   float64 // cube half-extent (vertices at ±Half on each axis)
	R      float64 // tube radius; corner solid radius and cut tangent depth
	Union  bool    // enable the sharp-side power-norm union
	Power  float64 // power-norm exponent p (large → sharp miter, ~5.4 → round)
	Cut    bool    // enable the flat-side plane cut
	Delta  float64 // plane offset along the diagonal
	Fillet float64 // smooth-max fillet width m for the cut rim
}

// cornerUnionBlend is the tiny smooth-min width used to union the corner solid
// into the wireframe — just enough to take the numerical edge off the miter
// creases without visibly rounding them (matches the reference EPSB).
const cornerUnionBlend = 0.006

func (cm *CornerMorph) apply(p Vec3, f float64) float64 {
	// Nearest vertex = the corner of p's octant.
	sx, sy, sz := signF(p[0]), signF(p[1]), signF(p[2])
	dx := p[0] - sx*cm.Half
	dy := p[1] - sy*cm.Half
	dz := p[2] - sz*cm.Half
	d2 := dx*dx + dy*dy + dz*dz
	if cm.Union {
		// Only near the vertex — the solid lives inside a ~1.55R ball.
		lim := 1.55 * cm.R
		if d2 <= lim*lim {
			// u_i = (a_i/R)^2, where a_i is the perpendicular distance to edge
			// line i. u_i^(p/2) = (a_i/R)^p, so blob = R·((Σ u_i^(p/2))^(1/p) − 1)
			// = (Σ a_i^p)^(1/p) − R. Working in squared distances avoids a sqrt.
			invR2 := 1.0 / (cm.R * cm.R)
			u1 := (d2 - dx*dx) * invR2
			u2 := (d2 - dy*dy) * invR2
			u3 := (d2 - dz*dz) * invR2
			h := cm.Power * 0.5
			sS := math.Pow(u1, h) + math.Pow(u2, h) + math.Pow(u3, h)
			blob := cm.R * (math.Pow(sS, 1.0/cm.Power) - 1)
			f = sminPoly(blob, f, cornerUnionBlend)
		}
	}
	if cm.Cut {
		// Plane ⟂ the corner diagonal: h = (p−v)·n̂ − δ, n̂ = octant sign / √3.
		// smaxPoly only ever raises the field, so the surface moves inward only.
		const inv = 0.5773502691896258 // 1/√3
		hplane := (dx*sx+dy*sy+dz*sz)*inv - cm.Delta
		if hplane > f-cm.Fillet {
			f = smaxPoly(f, hplane, cm.Fillet)
		}
	}
	return f
}

// Field is the signed-distance function that combines every primitive in
// `Prims` with the "organic sleeve" crowding field (see
// organic_sleeve_method.md), the generalization of the exponential smooth-min:
//
//	e_min = minᵢ eᵢ                              (hard union of the tubes)
//	S     = Σᵢ exp( −(eᵢ − e_min) / k )          (effective contributor count)
//	dip   = k · (ln S)^γ                          (crowding push-out)
//	D     = e_min − B · tanh( dip / B )           (cap the push-out at B)
//
// The classic log-sum-exp smooth-min is exactly the γ=1, B=∞ case. The two
// extra knobs are the point of the rework:
//   - K  (a length, "blend reach") — how far apart / far along tubes still merge.
//   - Gamma — how blend depth grows with the number of contributing tubes.
//     <1 flattens busy high-valence joints while keeping pairwise fillets full.
//   - Cap (B) — a hard tanh ceiling on how far any joint may bulge past the tube.
//
// Clips is an optional list of primitives the field is intersected with via
// hard-max (in order). Each entry trims the composite shape to fit inside
// it. Used here to wrap the strut wireframe in a rounded-cube envelope.
//
// Subs is an optional list of primitives subtracted via hard max(field,
// -sub.Dist). Material is removed inside each sub primitive. Used to carve
// curved bites out of the 8 cube vertices for the rounded-die look.
type Field struct {
	Prims []Primitive
	// Wire holds the 12 wireframe edge capsules. They are combined with a HARD
	// min (not the crowding field), so the three hemispherical caps meeting at
	// each cube vertex coincide into an exact sphere of radius R — no puff. The
	// corner morph (below) is then applied to this hard-min wireframe field.
	// The morphed wireframe distance enters the crowding field as a SINGLE
	// contributor term alongside the struts, so strut↔frame joints still fillet
	// by the same crowding rule, while bare cube corners (where only the wire
	// term is active, S≈1, dip≈0) keep their exact morphed shape.
	Wire []Primitive
	// Corners, if set, morphs the wireframe field at the 8 cube vertices
	// (sharp ↔ round ↔ flat).
	Corners *CornerMorph
	Clips   []Primitive
	Subs    []Primitive
	K       float64  // blend reach k (a length); guidance 0.5–2 × tube radius
	Gamma   float64  // crowding exponent γ; 1 = classic log growth, <1 flattens joints
	Cap     float64  // absolute push-out cap B; <=0 means "effectively off"
	KSharp  float64  // sharp anchor stiffness k' ≪ k; the smooth stand-in for the
	//           hard min. <=0 falls back to the hard-min anchor (kinked at γ≠1).
	bvh *bvhNode // built lazily on first eval (over Prims only)
}

// crowdCutoff: distance (in units of k) beyond e_min past which a term's
// exp(−Δ/k) is negligible. exp(−18) ≈ 1.5e-8, below single-float resolution.
const crowdCutoff = 18.0

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
	k := f.K

	// --- Gather the contributors into the smooth crowding field. ---
	// Each strut/support (Prims) contributes one distance term; the whole
	// wireframe (hard-min'd + corner-morphed) contributes one further term.
	// Over those distances we form two log-sum-exp softmins — a soft one at the
	// blend reach k, and a sharp one at k' ≪ k that stands in *smoothly* for
	// the hard min. The crowding measure is x = (sm_k' − sm_k)/k, the surface
	// anchor is sm_k', and dip = k·x^γ (capped). Because every piece is a
	// composition of smooth functions, the field is C^∞ even for γ ≠ 1 —
	// unlike anchoring the dip on the raw hard min, which creases along every
	// blend seam (the fillet valleys and vertex joints). As k' → 0, sm_k' → the
	// hard min and x → the classic ln S, recovering the earlier formulation.
	//
	// e_min (the true hard min) is computed only as the numerically-stable
	// exponent offset — it cancels analytically inside both softmins.
	eMinStrut := initial
	if len(f.Prims) > 0 {
		eMinStrut = bvhHardMin(f.bvh, p, initial)
	}

	haveWire := len(f.Wire) > 0
	w := initial
	if haveWire {
		for _, wc := range f.Wire {
			if d := wc.Dist(p); d < w {
				w = d
			}
		}
		if f.Corners != nil {
			w = f.Corners.apply(p, w)
		}
	}

	eMin := math.Min(eMinStrut, w)

	var v float64
	switch {
	case eMin >= initial-1:
		// Nothing anywhere near this point.
		v = eMin
	case f.KSharp <= 0:
		// Fallback: anchor on the hard min (kinked at γ≠1). Kept for A/B tests.
		s := 0.0
		cutoff := eMin + crowdCutoff*k
		if len(f.Prims) > 0 && eMinStrut <= cutoff {
			s += bvhSumExp(f.bvh, p, eMin, k, cutoff)
		}
		if haveWire && w <= cutoff {
			s += math.Exp(-(w - eMin) / k)
		}
		x := 0.0
		if s > 1 {
			x = math.Log(s)
		}
		dip := k * math.Pow(x, f.Gamma)
		if f.Cap > 0 {
			dip = f.Cap * math.Tanh(dip/f.Cap)
		}
		v = eMin - dip
	default:
		kp := f.KSharp
		cutoff := eMin + crowdCutoff*k
		// Sk  = Σ exp(−(eᵢ−eMin)/k)   (soft, reach k)
		// Skp = Σ exp(−(eᵢ−eMin)/k')  (sharp, reach k' — the smooth hard-min)
		// Both ≥ 1: the eMin term itself contributes exp(0)=1 to each.
		sK, sKp := 0.0, 0.0
		if len(f.Prims) > 0 && eMinStrut <= cutoff {
			bK, bKp := bvhSumExp2(f.bvh, p, eMin, k, kp, cutoff)
			sK += bK
			sKp += bKp
		}
		if haveWire && w <= cutoff {
			dw := w - eMin
			sK += math.Exp(-dw / k)
			sKp += math.Exp(-dw / kp)
		}
		lnSK, lnSKp := 0.0, 0.0
		if sK > 1 {
			lnSK = math.Log(sK)
		}
		if sKp > 1 {
			lnSKp = math.Log(sKp)
		}
		// Smooth sharp anchor sm_k' = eMin − k'·ln Skp (hard eMin cancels).
		smKp := eMin - kp*lnSKp
		// Crowding x = (sm_k' − sm_k)/k = ln Sk − (k'/k)·ln Skp ≥ 0.
		x := lnSK - (kp/k)*lnSKp
		if x < 0 {
			x = 0
		}
		dip := k * math.Pow(x, f.Gamma)
		if f.Cap > 0 {
			dip = f.Cap * math.Tanh(dip/f.Cap)
		}
		v = smKp - dip
	}
	// Hard max (not smooth) for clips. Outer envelopes should be sharp
	// boundaries; smoothing would erode thin features (e.g. d6 edge fillets
	// whose width is comparable to or smaller than the kernel 3/K). At
	// tangent rings between the inner field and a clip surface the seam is
	// C¹ smooth in practice anyway.
	for _, c := range f.Clips {
		cd := c.Dist(p)
		if cd > v {
			v = cd
		}
	}
	// Subtractions: hard max(field, -sub.Dist). Material remains only OUTSIDE
	// each sub primitive — i.e., the sub carves a curved bite out of the
	// composite shape.
	for _, sb := range f.Subs {
		sd := -sb.Dist(p)
		if sd > v {
			v = sd
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

// bvhSumExp accumulates Σ exp(−(dᵢ−eMin)/k) over every primitive whose SDF is
// within `cutoff` of eMin, using the box distance as a conservative lower
// bound to prune whole subtrees. eMin must be the true minimum SDF over the
// same primitive set (so every exponent is ≤ 0 and nothing overflows).
func bvhSumExp(n *bvhNode, p Vec3, eMin, k, cutoff float64) float64 {
	if n == nil {
		return 0
	}
	// Conservative cull: distToAABB is a lower bound on any contained
	// primitive's SDF, so if the box is beyond cutoff none can contribute.
	if distToAABB(p, n.box) > cutoff {
		return 0
	}
	if n.prims != nil {
		sum := 0.0
		for _, pr := range n.prims {
			d := pr.Dist(p)
			if d > cutoff {
				continue
			}
			sum += math.Exp(-(d - eMin) / k)
		}
		return sum
	}
	return bvhSumExp(n.left, p, eMin, k, cutoff) + bvhSumExp(n.right, p, eMin, k, cutoff)
}

// bvhSumExp2 is bvhSumExp accumulating two log-sum-exp sums in one traversal:
// one at stiffness k (soft) and one at k' (sharp). Each primitive's SDF is
// evaluated once. Returns (Σ exp(−(dᵢ−eMin)/k), Σ exp(−(dᵢ−eMin)/k')). The
// cutoff is in k units (k > k'), so it conservatively covers both sums — a
// term dropped for k contributes even less at the sharper k'.
func bvhSumExp2(n *bvhNode, p Vec3, eMin, k, kp, cutoff float64) (float64, float64) {
	if n == nil {
		return 0, 0
	}
	if distToAABB(p, n.box) > cutoff {
		return 0, 0
	}
	if n.prims != nil {
		var sumK, sumKp float64
		for _, pr := range n.prims {
			d := pr.Dist(p)
			if d > cutoff {
				continue
			}
			dd := d - eMin
			sumK += math.Exp(-dd / k)
			sumKp += math.Exp(-dd / kp)
		}
		return sumK, sumKp
	}
	lK, lKp := bvhSumExp2(n.left, p, eMin, k, kp, cutoff)
	rK, rKp := bvhSumExp2(n.right, p, eMin, k, kp, cutoff)
	return lK + rK, lKp + rKp
}
