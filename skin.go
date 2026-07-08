package main

import "math"

// SkinParams controls the implicit-skin mesher behavior.
type SkinParams struct {
	Resolution int     // marching-cubes grid resolution (per axis)
	BlendK     float64 // crowding blend reach k (a length); ~0.5–2 × StrutR
	Gamma      float64 // crowding exponent γ; 1 = classic, <1 flattens busy joints
	Cap        float64 // absolute joint push-out cap B; <=0 = off
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
		Resolution: 160, BlendK: 0.06, Gamma: 0.6, Cap: 0.12,
		Relax: 0, Lambda: 0.5, Padding: 0.3, Fillet: 0, Corner: 0.5,
	}
}

// cornerMorphFromSlider maps the Corner slider ∈ [0, 1] onto a CornerMorph,
// using the same formulas as the reference implementation.
//   t < 0.5  (sharp → round): union a power-norm solid whose exponent falls
//            quadratically from 128 (near-perfect miter) to 5.4 (solid has
//            shrunk inside the baseline sphere; union is a no-op → round).
//   t == 0.5 (round): no modification — the exact radius-r vertex sphere.
//   t > 0.5  (round → flat): a plane cut sliding from tangent-to-the-tip
//            (δ = r, no cut) to deep, with a rim fillet that widens with depth.
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

// BuildSkin runs the same generation pipeline as BuildScene, but emits SDF
// primitives for the structural skeleton (wireframe, strut bodies, joints,
// supports) instead of direct meshes. The handshake hands/arms/palms still
// render as their existing triangle meshes (per user direction — they're a
// placeholder until being replaced in Blender). Output: one Group containing
// the marching-cubes skin and the concatenated handshake mesh.
func BuildSkin(s Settings, seed uint32, sp SkinParams) *Group {
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)

	var prims []Primitive
	var wire []Primitive
	handshakeRoot := NewGroup()

	// Wireframe: 12 edge capsules at strut radius. These go in their own list
	// (hard-min'd, then corner-morphed) so the three caps meeting at each
	// vertex form an exact radius-R sphere with no smooth-min bulge.
	for _, e := range topo.Edges {
		wire = append(wire, Capsule{A: e.A, B: e.B, Radius: s.StrutR})
	}

	struts := GenerateStruts(&topo, s, seed)
	dec := NewRand(seed ^ 0x9e3779b9)
	var anchors []Anchor

	for _, st := range struts {
		hs := dec.F() < s.HandshakeProb
		skinStrut(s, &topo, st, hs, dec, &prims, handshakeRoot, &anchors)
	}
	for i := range anchors {
		anchors[i].AnchorIdx = i
	}

	anchorsByFace := map[int][]Anchor{}
	for _, a := range anchors {
		if a.Kind != "skin" {
			continue
		}
		anchorsByFace[a.Face.Idx] = append(anchorsByFace[a.Face.Idx], a)
	}
	supports := planWebbingSupports(s, &topo, anchorsByFace, dec, struts)
	for _, a := range anchors {
		if a.Kind != "floating" {
			continue
		}
		ps := planPyramidSupports(s, &topo, a.Anchor3D, a.PadR, a.AnchorIdx, dec, struts, []int{a.OwnerStrutID})
		supports = append(supports, ps...)
	}
	supports = globalSupportVsSupportPass(s, supports)
	for _, ss := range supports {
		prims = append(prims, Capsule{A: ss.P1, B: ss.P2, Radius: s.ButtressR})
	}

	// Tube radius. The corner morph reuses this as both the corner-solid
	// radius and the plane-cut tangent depth so corners meet the tubes
	// seamlessly. sp.Fillet <= 0 means "match the strut girth".
	r := sp.Fillet
	if r <= 0 {
		r = s.StrutR
	}
	// No outer envelope: the wireframe is just the 12 capsules, so away from
	// the vertices only one capsule term is ever active and the tubes stay
	// exact cylinders of radius r. All corner shaping is confined to the 8
	// vertices via CornerMorph, driven by the Corner slider ∈ [0, 1]:
	//   0.0 → sharpest miter   0.5 → round (baseline)   1.0 → flattest cut.
	field := &Field{
		Prims:   prims,
		Wire:    wire,
		Corners: cornerMorphFromSlider(sp.Corner, half, r),
		K:       sp.BlendK,
		Gamma:   sp.Gamma,
		Cap:     sp.Cap,
	}
	// The sleeve can bulge up to r + Cap past a vertex; pad the box to clear
	// that plus a couple of cells so the isosurface never touches the boundary.
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

	// Optional surface-tension pass: relax the mesh toward a taut, concave
	// fillet while keeping every vertex outside the tube skeleton (the tubes
	// act as the film's wire frame). See organic_sleeve_method.md Step 6.
	if sp.Relax > 0 {
		tubes := collectTubes(prims, wire)
		tris = relaxSurfaceTension(tris, tubes, sp.Relax, sp.Lambda)
	}

	// Render skin as a single material — a warm off-white in the spirit of
	// polished marble. (User intends to retouch in Blender.)
	skinMat := newMat(0xeae3d4, false)
	skin := NewGroup()
	skin.Mesh = &Mesh{Tris: tris, Mat: skinMat}

	root := NewGroup()
	root.Add(skin)
	root.Add(handshakeRoot)
	return root
}

// skinStrut: SDF-mode counterpart to buildStrut. Same RNG consumption order
// so a given seed produces matching geometry between debug and skin modes.
// No ball joints — strut capsules run anchor-to-anchor and clip into the
// wireframe / each other via the smooth-min blend.
func skinStrut(s Settings, topo *CubeTopology, strut Strut, withHandshake bool, rand *Rand, prims *[]Primitive, handshakeRoot *Group, anchorOut *[]Anchor) {
	half := s.CubeSize / 2
	a, b := strut.A, strut.B
	dirAB := b.Sub(a).Normalize()
	dirBA := dirAB.Neg()
	aOnSkin := topo.ClassifyBoundary(a, half) >= 1
	bOnSkin := topo.ClassifyBoundary(b, half) >= 1

	if aOnSkin {
		faceA := topo.FaceForBoundaryPoint(a, dirBA, half)
		cls := topo.ClassifyBoundary(a, half)
		if faceA != nil && cls == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: a, PadR: anchorPadR(s, cls, false), Face: faceA,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: a, PadR: anchorPadR(s, 0, true), Kind: "floating", OwnerStrutID: strut.ID,
		})
	}
	if bOnSkin {
		faceB := topo.FaceForBoundaryPoint(b, dirAB, half)
		cls := topo.ClassifyBoundary(b, half)
		if faceB != nil && cls == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: b, PadR: anchorPadR(s, cls, false), Face: faceB,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: b, PadR: anchorPadR(s, 0, true), Kind: "floating", OwnerStrutID: strut.ID,
		})
	}

	cleanA, cleanB := a, b
	cleanLen := cleanA.DistTo(cleanB)

	if !withHandshake {
		*prims = append(*prims, Capsule{A: cleanA, B: cleanB, Radius: s.StrutR})
		return
	}

	// Handshake — match buildStrut's RNG order so seeds stay equivalent.
	offset := (rand.F() - 0.5) * cleanLen * 0.15
	cleanMid := cleanA.Add(cleanB).Mul(0.5)
	meet := cleanMid.Add(dirAB.Mul(offset))
	distA := cleanA.DistTo(meet)
	distB := cleanB.DistTo(meet)
	armPortion := math.Min(0.5, math.Min(distA*0.55, distB*0.55))
	towardA := dirAB.Neg()
	towardB := dirAB
	shA := meet.Add(towardA.Mul(armPortion))
	shB := meet.Add(towardB.Mul(armPortion))

	// Stumps as SDF capsules so they blend smoothly into joints.
	if cleanA.DistTo(shA) > 0.05 {
		*prims = append(*prims, Capsule{A: cleanA, B: shA, Radius: s.StrutR})
	}
	if cleanB.DistTo(shB) > 0.05 {
		*prims = append(*prims, Capsule{A: cleanB, B: shB, Radius: s.StrutR})
	}
	grip := gripStyles[int(rand.F()*float64(len(gripStyles)))%len(gripStyles)]
	_ = rand.F()
	_ = rand.F()
	armA := buildArm(s, armPortion, grip, rand)
	armA.Transform.Position = shA
	armA.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardB)
	armA.Transform.Rotation = QuatMul(armA.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), (rand.F()-0.5)*0.4))
	handshakeRoot.Add(armA)
	armB := buildArm(s, armPortion, grip, rand)
	armB.Transform.Position = shB
	armB.Transform.Rotation = QuatFromUnitVectors(V(0, 0, 1), towardA)
	armB.Transform.Rotation = QuatMul(armB.Transform.Rotation, QuatFromAxisAngle(V(0, 0, 1), (rand.F()-0.5)*0.4))
	handshakeRoot.Add(armB)
}
