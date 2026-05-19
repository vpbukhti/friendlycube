package main

// BuildScene produces a Group representing the full sculpture for one seed,
// with the given settings. The output is in world space (origin at cube
// center). Determinism: same seed + same settings → bit-identical geometry.
func BuildScene(s Settings, seed uint32) *Group {
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)
	root := NewGroup()
	root.Add(buildWireframe(s, &topo))
	struts := GenerateStruts(&topo, s, seed)
	// Decoration RNG: a second LCG seeded from (seed ^ 0x9e3779b9) so toggling
	// the handshake probability doesn't change the strut skeleton.
	dec := NewRand(seed ^ 0x9e3779b9)
	var anchors []Anchor
	for _, st := range struts {
		hs := dec.F() < s.HandshakeProb
		root.Add(buildStrut(s, &topo, st, hs, dec, &anchors))
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
		root.Add(renderSupport(s, ss))
	}
	return root
}

func buildWireframe(s Settings, topo *CubeTopology) *Group {
	g := NewGroup()
	mWire := newMat(Colors.Wireframe, false)
	for _, v := range topo.Verts {
		addSphere(g, v, s.CornerR, 24, 18, mWire)
	}
	for _, e := range topo.Edges {
		a, b := e.A, e.B
		dir := b.Sub(a)
		l := dir.Len()
		shrink := s.CornerR * 0.7
		mid := a.Add(b).Mul(0.5)
		addCylinder(g, mid, dir.Normalize(), l-shrink, s.WireR, s.WireR, 16, mWire)
	}
	return g
}
