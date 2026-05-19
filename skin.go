package main

import "math"

// SkinParams controls the implicit-skin mesher behavior.
type SkinParams struct {
	Resolution int     // marching-cubes grid resolution (per axis)
	BlendK     float64 // smooth-min sharpness (large = sharp, small = soft)
	Padding    float64 // extra margin around the cube AABB
}

func DefaultSkinParams() SkinParams {
	return SkinParams{Resolution: 160, BlendK: 35, Padding: 0.3}
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

	// Wireframe: 12 edge capsules + 8 corner spheres.
	for _, e := range topo.Edges {
		prims = append(prims, Capsule{A: e.A, B: e.B, Radius: s.WireR})
	}
	for _, v := range topo.Verts {
		prims = append(prims, Sphere{Center: v, Radius: s.CornerR})
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
		beadR := s.ButtressR * 1.3
		if ss.Kind == "pyramid" {
			beadR = s.ButtressR * 1.4
		}
		prims = append(prims, Sphere{Center: ss.P2, Radius: beadR})
	}

	field := &Field{Prims: prims, K: sp.BlendK}
	box := AABB{
		Min: V(-half-sp.Padding, -half-sp.Padding, -half-sp.Padding),
		Max: V(half+sp.Padding, half+sp.Padding, half+sp.Padding),
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
// Emits SDF primitives for the strut body and joints, and the handshake
// arms (if any) as direct triangles into handshakeRoot.
func skinStrut(s Settings, topo *CubeTopology, strut Strut, withHandshake bool, rand *Rand, prims *[]Primitive, handshakeRoot *Group, anchorOut *[]Anchor) {
	half := s.CubeSize / 2
	a, b := strut.A, strut.B
	dirAB := b.Sub(a).Normalize()
	dirBA := dirAB.Neg()
	aOnSkin := topo.ClassifyBoundary(a, half) >= 1
	bOnSkin := topo.ClassifyBoundary(b, half) >= 1

	// Joint A
	var endA Vec3
	var padRA float64
	if aOnSkin {
		faceA := topo.FaceForBoundaryPoint(a, dirBA, half)
		cls := topo.ClassifyBoundary(a, half)
		mult := 2.4
		switch cls {
		case 1:
			mult = 1.8
		case 2:
			mult = 2.1
		}
		padRA = s.StrutR * mult
		// Joint sphere pulled slightly inward so the body capsule blends
		// into it instead of the boundary plane.
		jointCenter := a.Add(dirAB.Mul(s.StrutR * 0.6))
		*prims = append(*prims, Sphere{Center: jointCenter, Radius: padRA * 0.9})
		// "Clean" interior end where the strut body starts.
		endA = a.Add(dirAB.Mul(s.StrutR * 1.6))
		if faceA != nil && cls == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: a, PadR: padRA, Face: faceA, Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		padRA = s.StrutR * 1.25
		*prims = append(*prims, Sphere{Center: a, Radius: padRA})
		endA = a
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: a, PadR: padRA, Kind: "floating", OwnerStrutID: strut.ID,
		})
	}

	// Joint B
	var endB Vec3
	var padRB float64
	if bOnSkin {
		faceB := topo.FaceForBoundaryPoint(b, dirAB, half)
		cls := topo.ClassifyBoundary(b, half)
		mult := 2.4
		switch cls {
		case 1:
			mult = 1.8
		case 2:
			mult = 2.1
		}
		padRB = s.StrutR * mult
		jointCenter := b.Add(dirBA.Mul(s.StrutR * 0.6))
		*prims = append(*prims, Sphere{Center: jointCenter, Radius: padRB * 0.9})
		endB = b.Add(dirBA.Mul(s.StrutR * 1.6))
		if faceB != nil && cls == 1 {
			*anchorOut = append(*anchorOut, Anchor{
				Anchor3D: b, PadR: padRB, Face: faceB, Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		padRB = s.StrutR * 1.25
		*prims = append(*prims, Sphere{Center: b, Radius: padRB})
		endB = b
		*anchorOut = append(*anchorOut, Anchor{
			Anchor3D: b, PadR: padRB, Kind: "floating", OwnerStrutID: strut.ID,
		})
	}

	cleanA, cleanB := endA, endB
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
