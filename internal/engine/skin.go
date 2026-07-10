package engine

import "math"

// SkinParams controls the implicit-skin mesher behavior.
type SkinParams struct {
	Resolution int     // marching-cubes grid resolution (per axis)
	BlendK     float64 // crowding blend reach k (a length); ~0.5–2 × StrutR
	Gamma      float64 // crowding exponent γ; 1 = classic, <1 flattens busy joints
	Cap        float64 // absolute joint push-out cap B; <=0 = off
	// SharpRatio sets the smooth-anchor stiffness k' = SharpRatio × BlendK. It
	// keeps the field C^∞ for γ≠1: smaller = a sharper (more faithful to hard-
	// min, but less smoothed) seam; larger = seams rounded over a bigger fillet.
	// ~0.15–0.3 is a good range. <=0 reverts to the kinked hard-min anchor.
	SharpRatio float64
	// VertexGuard is the radius of the corner-protection ball around each cube
	// vertex where the outer edge clip is suppressed (so the CornerMorph rules
	// the corners). <=0 = auto (1.6×r + Cap). Only used in skin mode.
	VertexGuard float64
	// ClipBlend is the fillet width of the outer edge clip. >0 = smooth join
	// (concave fillet where the sleeve meets the skin, no crease); <=0 = hard
	// crease. <0 sentinel is treated as auto (0.4×r). Only used in skin mode.
	ClipBlend float64
	// ClipVBlend is the width over which the corner-protection ball necks into
	// the clipped edge (removes the crease ring around each vertex, smoothing
	// the corner→edge transition). <0 = auto (≈r). 0 = hard. Only skin mode.
	ClipVBlend float64
	// Relax runs N surface-tension (constrained mean-curvature) passes on the
	// extracted mesh; 0 disables it. Lambda is the per-pass Laplacian step.
	Relax   int
	Lambda  float64
	Padding float64 // extra margin around the cube AABB
	// Fillet is the rounded-cube edge/corner radius. <=0 means "use
	// Settings.StrutR" — so the d6 fillet visually matches the strut bodies
	// by default.
	Fillet float64
	// Corner ∈ [0, 1] morphs each of the 8 cube corners across a single axis:
	//   0.0 → sharpest miter (power-norm Steinmetz solid, walls flush with the
	//         tubes, straight creases converging to a point);
	//   0.5 → round (the natural sphere formed by the three hemispherical
	//         capsule caps — no modification);
	//   1.0 → flattest cut (plane perpendicular to the body diagonal, sliced
	//         down to the vertex with a filleted rim).
	// Below 0.5 the sharp corner solid is unioned in with a power that drops
	// from ~40 (sharp) to ~5.4 (round). Above 0.5 material is removed by a
	// smooth-max plane cut whose depth and fillet grow toward 1.0.
	Corner float64
}

func DefaultSkinParams() SkinParams {
	// BlendK/Gamma/Cap are the organic-sleeve knobs. Defaults tuned around the
	// default StrutR (0.085): k ≈ 0.7×r for local fillets, γ<1 so the busy
	// cube joints stay modest, B ≈ 1.4×r to cap the largest bulges.
	return SkinParams{
		Resolution: 160, BlendK: 0.06, Gamma: 0.6, Cap: 0.12, SharpRatio: 0.2,
		ClipBlend: -1, ClipVBlend: -1, // auto: 0.4×r fillet, ≈r corner neck
		Relax: 0, Lambda: 0.5, Padding: 0.3, Fillet: 0, Corner: 0.5,
	}
}

// cornerMorphFromSlider maps the Corner slider ∈ [0, 1] onto a CornerMorph,
// using the same formulas as the reference implementation.
//
//	t < 0.5  (sharp → round): union a power-norm solid whose exponent falls
//	         quadratically from 128 (near-perfect miter) to 5.4 (solid has
//	         shrunk inside the baseline sphere; union is a no-op → round).
//	t == 0.5 (round): no modification — the exact radius-r vertex sphere.
//	t > 0.5  (round → flat): a plane cut sliding from tangent-to-the-tip
//	         (δ = r, no cut) to deep, with a rim fillet that widens with depth.
func cornerMorphFromSlider(t float64, half, r float64) *CornerMorph {
	cm := &CornerMorph{Half: half, R: r}
	switch {
	case t <= 0.495:
		w := t * 2 // 0 = sharpest, 1 = round
		cm.Union = true
		cm.Power = 5.4 + 122.6*(1-w)*(1-w)
	case t > 0.501:
		v := (t - 0.5) * 2 // 0 = round, 1 = flattest
		cm.Cut = true
		cm.Delta = r - v*(r+0.2*half)
		cm.Fillet = 1e-4 + v*(0.8*r+0.035)
	}
	return cm
}

// BuildSkin bakes the skeleton for one seed and meshes its implicit skin.
func BuildSkin(s Settings, seed uint32, sp SkinParams) *Group {
	return BuildSkinFromSkeleton(s, BakeSkeleton(s, seed), sp)
}

// BuildSkinFromSkeleton wraps an explicit skeleton in one signed-distance field
// and extracts its watertight skin. Every strut is a FULL-length capsule (thick
// or thin) — the skin is complete everywhere; handshakes are a separate overlay
// (BuildHandshakeOverlay), never a gap in the body. Supports come straight from
// the skeleton. The field/clip/corner-morph/marching-cubes/relax stages are
// unchanged from the original pipeline.
func BuildSkinFromSkeleton(s Settings, sk *Skeleton, sp SkinParams) *Group {
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)

	var prims []Primitive
	var wire []Primitive

	// Wireframe: 12 fixed edge capsules at strut radius. Hard-min'd + corner-
	// morphed so the three caps at each vertex form one exact sphere.
	for _, e := range topo.Edges {
		wire = append(wire, Capsule{A: e.A, B: e.B, Radius: s.StrutR})
	}

	// Struts: one full watertight capsule each, at the strut's effective radius.
	for _, st := range sk.Struts {
		prims = append(prims, Capsule{A: sk.EndA(st), B: sk.EndB(st), Radius: st.EffRadius(s)})
	}
	// Support web: thin capsules.
	for _, ss := range sk.Supports {
		prims = append(prims, Capsule{A: sk.EndP1(ss), B: sk.EndP2(ss), Radius: s.ButtressR})
	}

	// Tube radius for the corner morph (matches strut girth by default).
	r := sp.Fillet
	if r <= 0 {
		r = s.StrutR
	}
	cm := cornerMorphFromSlider(sp.Corner, half, r)
	field := &Field{
		Prims:   prims,
		Wire:    wire,
		Corners: cm,
		K:       sp.BlendK,
		Gamma:   sp.Gamma,
		Cap:     sp.Cap,
		KSharp:  sp.SharpRatio * sp.BlendK,
	}
	// Outer edge clip (see original notes in git history): a rounded cube whose
	// edges coincide with the 12 wireframe tubes, guarded at the corners where
	// the sharp miter pokes past the outer skin.
	vguard := sp.VertexGuard
	if vguard <= 0 {
		vguard = 0
		if cm.Union {
			tip := r * math.Sqrt(1.5) / math.Pow(3, 1.0/cm.Power)
			vguard = 1.05 * tip
		}
	}
	clipBlend := sp.ClipBlend
	if clipBlend < 0 {
		clipBlend = 0.4 * r
	}
	clipVBlend := sp.ClipVBlend
	if clipVBlend < 0 {
		clipVBlend = r
	}
	field.OuterClip = RoundedBox{HalfSize: V(half, half, half), R: r}
	field.ClipHalf = half
	field.ClipVGuard = vguard
	field.ClipBlend = clipBlend
	field.ClipVBlend = clipVBlend

	maxSleeve := r + 0.05
	if sp.Cap > 0 {
		maxSleeve += sp.Cap
	}
	pad := sp.Padding
	if pad < maxSleeve {
		pad = maxSleeve
	}
	box := AABB{
		Min: V(-half-pad, -half-pad, -half-pad),
		Max: V(half+pad, half+pad, half+pad),
	}
	tris := MarchingCubes(field, box, sp.Resolution)

	if sp.Relax > 0 {
		tubes := collectTubes(prims, wire)
		tris = relaxSurfaceTension(tris, tubes, sp.Relax, sp.Lambda)
	}

	skinMat := newMat(0xeae3d4, false)
	skin := NewGroup()
	skin.Mesh = &Mesh{Tris: tris, Mat: skinMat}

	root := NewGroup()
	root.Add(skin)
	return root
}

// BuildHandshakeOverlay rebuilds the handshake arm meshes as a separate overlay
// (not fused into the watertight body). Each handshake follows its strut's
// current endpoints, and is tinted red when flagged undesirable. Deterministic:
// no RNG — everything is replayed from the stored SkelHandshake fields.
func BuildHandshakeOverlay(s Settings, sk *Skeleton) *Group {
	root := NewGroup()
	scratch := NewRand(0) // buildArm takes a *Rand but consumes no draws
	for _, h := range sk.Handshakes {
		st := sk.strutByID(h.StrutID)
		if st == nil {
			continue
		}
		meet, armPortion, ok := sk.handshakeGeom(h, *st)
		if !ok || armPortion <= 0 {
			continue
		}
		a, b := sk.EndA(*st), sk.EndB(*st)
		dirAB := b.Sub(a).Normalize()
		towardA := dirAB.Neg()
		towardB := dirAB
		shA := meet.Add(towardA.Mul(armPortion))
		shB := meet.Add(towardB.Mul(armPortion))

		armA := buildArm(s, armPortion, h.Grip, scratch)
		armA.Transform.Position = shA
		armA.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardB)
		armA.Transform.Rotation = QuatMul(armA.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), h.JitA))
		armB := buildArm(s, armPortion, h.Grip, scratch)
		armB.Transform.Position = shB
		armB.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardA)
		armB.Transform.Rotation = QuatMul(armB.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), h.JitB))
		if h.Undesirable {
			setGroupColor(armA, Colors.CollisionSnap)
			setGroupColor(armB, Colors.CollisionSnap)
		}
		root.Add(armA)
		root.Add(armB)
	}
	return root
}

// setGroupColor recolors every mesh in a group subtree (used to flag an
// undesirable handshake red without changing buildArm's signature).
func setGroupColor(g *Group, hex int) {
	c := hex2rgb(hex)
	var walk func(n *Group)
	walk = func(n *Group) {
		if n.Mesh != nil {
			n.Mesh.Mat.Color = c
		}
		for _, ch := range n.Children {
			walk(ch)
		}
	}
	walk(g)
}
