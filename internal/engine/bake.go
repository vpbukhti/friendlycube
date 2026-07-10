package engine

// BakeSkeleton runs the procedural generator ONCE and freezes the result into an
// explicit, editable Skeleton. It reproduces the exact RNG-consumption order of
// the old BuildScene / BuildSkin pipelines so the baked struts and supports are
// bit-identical to what those produced:
//
//   - the primary LCG (inside GenerateStruts) is untouched;
//   - the decoration LCG `dec` is drawn in the same order and count — one
//     draw per strut for the handshake coin-flip, then, for a handshake, the
//     same six draws skinStrut/buildStrut make (offset, grip index, two warm
//     draws, two arm-jitter draws), then the support planners' draws.
//
// buildArm consumes zero draws, so no extra draws are hidden there.
func BakeSkeleton(s Settings, seed uint32) *Skeleton {
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)
	struts := GenerateStruts(&topo, s, seed)
	dec := NewRand(seed ^ 0x9e3779b9)

	sk := &Skeleton{}
	var anchors []Anchor
	for _, st := range struts {
		hs := dec.F() < s.HandshakeProb
		sk.Struts = append(sk.Struts, SkelStrut{
			ID: st.ID, A: st.A, B: st.B, AVert: -1, BVert: -1,
			Kind: StrutThick, Width: 0, Lf: st.Lf, KindGen: st.KindGen,
			SnappedA: st.SnappedA, SnappedB: st.SnappedB,
		})
		collectAnchors(s, &topo, st, &anchors)
		if hs {
			// Mirror skinStrut's six dec draws exactly (skin.go:287–316).
			offFrac := (dec.F() - 0.5) * 0.15
			grip := gripStyles[int(dec.F()*float64(len(gripStyles)))%len(gripStyles)]
			_ = dec.F() // warmA
			_ = dec.F() // warmB
			jitA := (dec.F() - 0.5) * 0.4
			jitB := (dec.F() - 0.5) * 0.4
			sk.Handshakes = append(sk.Handshakes, SkelHandshake{
				StrutID: st.ID, Mid: st.A.Add(st.B).Mul(0.5),
				OffsetFrac: offFrac, Grip: grip, JitA: jitA, JitB: jitB,
			})
		}
	}

	// Support planning — identical to rebuild.go:24–38 / skin.go:115–134.
	for i := range anchors {
		anchors[i].AnchorIdx = i
	}
	anchorsByFace := map[int][]Anchor{}
	for _, a := range anchors {
		if a.Kind == "skin" {
			anchorsByFace[a.Face.Idx] = append(anchorsByFace[a.Face.Idx], a)
		}
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
	for i, ss := range supports {
		sk.Supports = append(sk.Supports, SkelSupport{
			ID: i, P1: ss.P1, P2: ss.P2, Kind: ss.Kind, BeadKind: ss.BeadKind,
			AnchorIdx: ss.AnchorIdx, ApexSkip: ss.ApexSkip, PadR: ss.PadR,
		})
	}

	sk.NextStrutID = len(struts)
	sk.buildSharedVertices()
	sk.RecomputeHandshakeFlags(s)
	return sk
}

// collectAnchors mirrors the anchor bookkeeping in skinStrut / buildStrut
// (skin.go:249–276, joints.go:145–178) — same order (A then B), same skin-vs-
// floating classification — so support planning sees identical anchors. It
// consumes no RNG.
func collectAnchors(s Settings, topo *CubeTopology, strut Strut, out *[]Anchor) {
	half := s.CubeSize / 2
	a, b := strut.A, strut.B
	dirAB := b.Sub(a).Normalize()
	dirBA := dirAB.Neg()

	if topo.ClassifyBoundary(a, half) >= 1 {
		cls := topo.ClassifyBoundary(a, half)
		faceA := topo.FaceForBoundaryPoint(a, dirBA, half)
		if faceA != nil && cls == 1 {
			*out = append(*out, Anchor{
				Anchor3D: a, PadR: anchorPadR(s, cls, false), Face: faceA,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		*out = append(*out, Anchor{
			Anchor3D: a, PadR: anchorPadR(s, 0, true), Kind: "floating", OwnerStrutID: strut.ID,
		})
	}
	if topo.ClassifyBoundary(b, half) >= 1 {
		cls := topo.ClassifyBoundary(b, half)
		faceB := topo.FaceForBoundaryPoint(b, dirAB, half)
		if faceB != nil && cls == 1 {
			*out = append(*out, Anchor{
				Anchor3D: b, PadR: anchorPadR(s, cls, false), Face: faceB,
				Kind: "skin", OwnerStrutID: strut.ID,
			})
		}
	} else {
		*out = append(*out, Anchor{
			Anchor3D: b, PadR: anchorPadR(s, 0, true), Kind: "floating", OwnerStrutID: strut.ID,
		})
	}
}
