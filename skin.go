package main

import "math"

// SkinParams controls the implicit-skin mesher behavior.
type SkinParams struct {
	Resolution int     // marching-cubes grid resolution (per axis)
	BlendK     float64 // smooth-min sharpness (large = sharp, small = soft)
	Padding    float64 // extra margin around the cube AABB
	// Fillet is the rounded-cube edge/corner radius. <=0 means "use
	// Settings.StrutR" — so the d6 fillet visually matches the strut bodies
	// by default.
	Fillet float64
}

func DefaultSkinParams() SkinParams {
	return SkinParams{Resolution: 160, BlendK: 35, Padding: 0.3, Fillet: 0}
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
	handshakeRoot := NewGroup()

	// Wireframe: 12 edge capsules at strut radius. No corner spheres — the
	// three capsules meeting at each vertex just clip into one another, and
	// smooth-min handles the visual blend.
	for _, e := range topo.Edges {
		prims = append(prims, Capsule{A: e.A, B: e.B, Radius: s.StrutR})
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

	// Rounded-box envelope. Inner half-size = `half` (i.e. the cube edge
	// axes), so the edge fillet's axis sits exactly on the cube edge axis
	// and the corner 1/8-sphere is centered on the cube vertex. The outer
	// flat faces are at ±(half + r) — i.e. one fillet radius beyond the
	// cube edge axes, in the outward direction. This keeps the cube edges
	// at their full strut girth instead of clipping them down to a thin
	// sliver.
	r := sp.Fillet
	if r <= 0 {
		r = s.StrutR
	}
	field := &Field{
		Prims: prims,
		Clip:  RoundedBox{HalfSize: V(half, half, half), R: r},
		K:     sp.BlendK,
	}
	pad := sp.Padding
	if pad < r+0.05 {
		pad = r + 0.05
	}
	box := AABB{
		Min: V(-half-pad, -half-pad, -half-pad),
		Max: V(half+pad, half+pad, half+pad),
	}
	tris := MarchingCubes(field, box, sp.Resolution)

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
