package engine

// BuildScene produces the debug Group (colored per-feature diagram) for one
// seed. It bakes the skeleton and renders it. Determinism: same seed + settings
// → bit-identical geometry.
func BuildScene(s Settings, seed uint32) *Group {
	return BuildSceneFromSkeleton(s, BakeSkeleton(s, seed))
}

// BuildSceneFromSkeleton renders an explicit skeleton as the debug diagram:
// the fixed wireframe, each strut as a colored cylinder (no handshakes — those
// are a separate overlay now), snap markers, and the support web. Rendering
// order (wireframe → struts → supports) matches the old BuildScene so a freshly
// baked, unedited skeleton reproduces the previous output exactly.
func BuildSceneFromSkeleton(s Settings, sk *Skeleton) *Group {
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)
	root := NewGroup()
	root.Add(buildWireframe(s, &topo))
	for _, st := range sk.Struts {
		root.Add(renderSkelStrut(s, &topo, sk, st))
	}
	for _, ss := range sk.Supports {
		root.Add(renderSupport(s, supportFromSkel(sk, ss)))
	}
	return root
}

// renderSkelStrut is the debug counterpart of buildStrut's plain (non-handshake)
// path: snap markers (when the endpoint sits on the skin and was snapped) then
// one full cylinder colored by length-factor / generation kind.
func renderSkelStrut(s Settings, topo *CubeTopology, sk *Skeleton, st SkelStrut) *Group {
	g := NewGroup()
	a, b := sk.EndA(st), sk.EndB(st)
	half := s.CubeSize / 2
	var strutColor int
	switch {
	case st.Lf < 0.999:
		strutColor = Colors.StrutShortened
	case st.KindGen == "edge":
		strutColor = Colors.StrutEdgeToEdge
	default:
		strutColor = Colors.StrutFull
	}
	matStrut := newMat(strutColor, false)
	if topo.ClassifyBoundary(a, half) >= 1 && st.SnappedA {
		g.Add(buildSnapMarker(s, a))
	}
	if topo.ClassifyBoundary(b, half) >= 1 && st.SnappedB {
		g.Add(buildSnapMarker(s, b))
	}
	r := st.EffRadius(s)
	dir := b.Sub(a)
	addCylinder(g, a.Add(b).Mul(0.5), dir.Normalize(), dir.Len(), r, r, 24, matStrut)
	return g
}

// supportFromSkel adapts a stored SkelSupport back to a Support for
// renderSupport, resolving its endpoints through the shared vertex table.
func supportFromSkel(sk *Skeleton, ss SkelSupport) Support {
	return Support{
		P1: sk.EndP1(ss), P2: sk.EndP2(ss), Kind: ss.Kind, AnchorIdx: ss.AnchorIdx,
		ApexSkip: ss.ApexSkip, PadR: ss.PadR, BeadKind: ss.BeadKind,
	}
}

func buildWireframe(s Settings, topo *CubeTopology) *Group {
	g := NewGroup()
	mWire := newMat(Colors.Wireframe, false)
	for _, e := range topo.Edges {
		a, b := e.A, e.B
		dir := b.Sub(a)
		l := dir.Len()
		mid := a.Add(b).Mul(0.5)
		addCylinder(g, mid, dir.Normalize(), l, s.StrutR, s.StrutR, 16, mWire)
	}
	return g
}
