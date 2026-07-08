package engine

import "math"

type Support struct {
	P1, P2    Vec3
	Kind      string // "pyramid" or "webbing"
	AnchorIdx int
	ApexSkip  float64
	PadR      float64
	BeadKind  string // "inner", "support", "wire", "landing"
}

// snapToInteriorStrut: snap a (p1,p2) segment that gets too close to any
// interior strut (not in excludeIds) to that strut's nearest endpoint.
// Returns (snapPt, true) on a hit; (_, false) otherwise.
func snapToInteriorStrut(s Settings, p1, p2 Vec3, struts []Strut, excludeIds []int, apexSkip float64) (Vec3, bool) {
	fullLen := p1.DistTo(p2)
	if fullLen < apexSkip*1.1 {
		return Vec3{}, false
	}
	dir := p2.Sub(p1).Normalize()
	a1 := p1.Add(dir.Mul(apexSkip))
	a2 := p2
	threshold := s.ButtressR + s.StrutR + 0.005
	var bestSnap Vec3
	bestT := math.Inf(1)
	found := false
	for _, st := range struts {
		excluded := false
		for _, id := range excludeIds {
			if st.ID == id {
				excluded = true
				break
			}
		}
		if excluded {
			continue
		}
		hit := ClosestApproach(a1, a2, st.A, st.B)
		if hit.Dist >= threshold {
			continue
		}
		dA := hit.QOnEx.DistTo(st.A)
		dB := hit.QOnEx.DistTo(st.B)
		target := st.B
		if dA <= dB {
			target = st.A
		}
		tFromP1 := p1.DistTo(hit.POnAB)
		if tFromP1 < bestT {
			bestT = tFromP1
			bestSnap = target
			found = true
		}
	}
	return bestSnap, found
}

func planPyramidSupports(s Settings, topo *CubeTopology, apex Vec3, padR float64, anchorIdx int, rand *Rand, struts []Strut, excludeIds []int) []Support {
	var out []Support
	half := s.CubeSize / 2
	face := topo.PyramidFaceFor(apex)
	if face == nil {
		return out
	}
	origin2D := [2]float64{apex.Dot(face.U), apex.Dot(face.V)}
	planeOffset3D := face.Normal.Mul(half)
	distToEdge2D := half - math.Max(math.Abs(origin2D[0]), math.Abs(origin2D[1]))
	if distToEdge2D < 0.05 {
		return out
	}
	NUM := s.SupportNum
	if NUM < 1 {
		NUM = 1
	}
	baseAngle := rand.F() * 2 * math.Pi
	angles := make([]float64, NUM)
	slice := (2 * math.Pi) / float64(NUM)
	for i := 0; i < NUM; i++ {
		angles[i] = baseAngle + slice*float64(i) + (rand.F()-0.5)*slice*s.SupportJitter
	}
	placedLocal := []Support{}
	MIN_LEN := padR * s.SupportMinLen
	APEX_SKIP := padR * s.ApexSkipMult
	for _, ang := range angles {
		dir2D := [2]float64{math.Cos(ang), math.Sin(ang)}
		bestT := math.Inf(1)
		var bestEnd2D [2]float64
		haveBest := false
		// Find face-boundary intersection via parametric AABB on the 2D square.
		for _, sign := range []float64{-1, 1} {
			if math.Abs(dir2D[0]) > 1e-7 {
				t := (sign*half - origin2D[0]) / dir2D[0]
				if t > 0 && t < bestT {
					py := origin2D[1] + t*dir2D[1]
					if math.Abs(py) <= half+1e-6 {
						bestT = t
						bestEnd2D = [2]float64{sign * half, py}
						haveBest = true
					}
				}
			}
			if math.Abs(dir2D[1]) > 1e-7 {
				t := (sign*half - origin2D[1]) / dir2D[1]
				if t > 0 && t < bestT {
					px := origin2D[0] + t*dir2D[0]
					if math.Abs(px) <= half+1e-6 {
						bestT = t
						bestEnd2D = [2]float64{px, sign * half}
						haveBest = true
					}
				}
			}
		}
		if !haveBest {
			continue
		}
		landing3D := planeOffset3D.Add(face.U.Mul(bestEnd2D[0])).Add(face.V.Mul(bestEnd2D[1]))
		p1 := apex
		p2 := landing3D
		collidedSnap := false
		snappedToWire := false
		snappedToInner := false
		// up to 3 refinement passes against earlier supports in this fan.
		for pass := 0; pass < 3; pass++ {
			fullLen := p1.DistTo(p2)
			if fullLen < APEX_SKIP*1.1 {
				break
			}
			dir := p2.Sub(p1).Normalize()
			a1 := p1.Add(dir.Mul(APEX_SKIP))
			a2 := p2
			hitFound := false
			for _, ex := range placedLocal {
				exLen := ex.P1.DistTo(ex.P2)
				if exLen < APEX_SKIP*1.1 {
					continue
				}
				exDir := ex.P2.Sub(ex.P1).Normalize()
				b1 := ex.P1.Add(exDir.Mul(APEX_SKIP))
				b2 := ex.P2
				threshold := s.ButtressR*2.0 + 0.005
				hit := ClosestApproach(a1, a2, b1, b2)
				if hit.Dist < threshold {
					fullHit := ClosestApproach(p1, p2, ex.P1, ex.P2)
					p2 = fullHit.QOnEx
					collidedSnap = true
					hitFound = true
					break
				}
			}
			if !hitFound {
				break
			}
		}
		if snapPt, ok := snapToInteriorStrut(s, p1, p2, struts, excludeIds, APEX_SKIP); ok {
			p2 = snapPt
			snappedToInner = true
			collidedSnap = false
			snappedToWire = false
		}
		if !collidedSnap && !snappedToInner {
			sn := topo.SnapToWireFeature(p2, s.StrutR*s.WireSnapMult)
			if sn.Snapped {
				p2 = sn.Point
				snappedToWire = true
			}
		}
		if p1.DistTo(p2) < MIN_LEN {
			continue
		}
		beadKind := "landing"
		switch {
		case snappedToInner:
			beadKind = "inner"
		case collidedSnap:
			beadKind = "support"
		case snappedToWire:
			beadKind = "wire"
		}
		rec := Support{P1: p1, P2: p2, Kind: "pyramid", AnchorIdx: anchorIdx, ApexSkip: APEX_SKIP, PadR: padR, BeadKind: beadKind}
		placedLocal = append(placedLocal, rec)
		out = append(out, rec)
	}
	return out
}

func planWebbingSupports(s Settings, topo *CubeTopology, anchorsByFace map[int][]Anchor, rand *Rand, struts []Strut) []Support {
	var out []Support
	half := s.CubeSize / 2
	for faceIdx := range topo.Faces {
		face := &topo.Faces[faceIdx]
		anchors := anchorsByFace[faceIdx]
		if len(anchors) == 0 {
			continue
		}
		type item struct {
			anchor3D     Vec3
			padR         float64
			ownerStrutId int
			anchorIdx    int
			p2           [2]float64
		}
		items := make([]item, len(anchors))
		for i, a := range anchors {
			items[i] = item{
				anchor3D: a.Anchor3D, padR: a.PadR,
				ownerStrutId: a.OwnerStrutID, anchorIdx: a.AnchorIdx,
				p2: [2]float64{a.Anchor3D.Dot(face.U), a.Anchor3D.Dot(face.V)},
			}
		}
		placed2D := [][2][2]float64{}
		order := make([]int, len(items))
		for i := range order {
			order[i] = i
		}
		// Fisher-Yates with our seeded RNG to mirror JS shuffle.
		for i := len(order) - 1; i > 0; i-- {
			j := int(rand.F() * float64(i+1))
			if j > i {
				j = i
			}
			order[i], order[j] = order[j], order[i]
		}
		for _, ai := range order {
			it := items[ai]
			center2D := it.p2
			padR := it.padR
			distToEdge := half - math.Max(math.Abs(center2D[0]), math.Abs(center2D[1]))
			if distToEdge < padR*1.8 {
				continue
			}
			NUM := s.SupportNum
			if NUM < 1 {
				NUM = 1
			}
			baseAngle := rand.F() * 2 * math.Pi
			angles := make([]float64, NUM)
			slice := (2 * math.Pi) / float64(NUM)
			for i2 := 0; i2 < NUM; i2++ {
				angles[i2] = baseAngle + slice*float64(i2) + (rand.F()-0.5)*slice*s.SupportJitter
			}
			for _, ang := range angles {
				dir2D := [2]float64{math.Cos(ang), math.Sin(ang)}
				start2D := [2]float64{
					center2D[0] + dir2D[0]*padR*0.92,
					center2D[1] + dir2D[1]*padR*0.92,
				}
				bestT := math.Inf(1)
				var bestEnd2D [2]float64
				haveBest := false
				for _, sign := range []float64{-1, 1} {
					if math.Abs(dir2D[0]) > 1e-7 {
						t := (sign*half - start2D[0]) / dir2D[0]
						if t > 0 && t < bestT {
							py := start2D[1] + t*dir2D[1]
							if math.Abs(py) <= half+1e-6 {
								bestT = t
								bestEnd2D = [2]float64{sign * half, py}
								haveBest = true
							}
						}
					}
					if math.Abs(dir2D[1]) > 1e-7 {
						t := (sign*half - start2D[1]) / dir2D[1]
						if t > 0 && t < bestT {
							px := start2D[0] + t*dir2D[0]
							if math.Abs(px) <= half+1e-6 {
								bestT = t
								bestEnd2D = [2]float64{px, sign * half}
								haveBest = true
							}
						}
					}
				}
				if !haveBest {
					continue
				}
				// clip against earlier in-plane supports on this face
				for _, ps := range placed2D {
					inter, _, ok := SegIntersect2D(start2D, bestEnd2D, ps[0], ps[1])
					if !ok {
						continue
					}
					dx := inter[0] - start2D[0]
					dy := inter[1] - start2D[1]
					tDist := math.Sqrt(dx*dx + dy*dy)
					if tDist < bestT {
						bestT = tDist
						bestEnd2D = inter
					}
				}
				// clip against other anchors' pads on this face
				for _, oa := range items {
					if oa.anchorIdx == it.anchorIdx {
						continue
					}
					sx := bestEnd2D[0] - start2D[0]
					sy := bestEnd2D[1] - start2D[1]
					segLen := math.Sqrt(sx*sx + sy*sy)
					if segLen < 1e-6 {
						continue
					}
					sdx := sx / segLen
					sdy := sy / segLen
					tcx := oa.p2[0] - start2D[0]
					tcy := oa.p2[1] - start2D[1]
					proj := tcx*sdx + tcy*sdy
					if proj > 0 && proj < segLen {
						perpDist := math.Abs(tcx*sdy - tcy*sdx)
						if perpDist < oa.padR {
							clipT := proj - math.Sqrt(math.Max(0, oa.padR*oa.padR-perpDist*perpDist))
							if clipT > 0 && clipT < bestT {
								bestT = clipT
								bestEnd2D = [2]float64{start2D[0] + sdx*clipT, start2D[1] + sdy*clipT}
							}
						}
					}
				}
				if bestT < padR*0.6 {
					continue
				}
				planeOffset := face.Normal.Mul(half)
				start3D := planeOffset.Add(face.U.Mul(start2D[0])).Add(face.V.Mul(start2D[1]))
				end3D := planeOffset.Add(face.U.Mul(bestEnd2D[0])).Add(face.V.Mul(bestEnd2D[1]))
				snappedToInner := false
				apexSkipWeb := padR * 1.1
				if snapPt, ok := snapToInteriorStrut(s, start3D, end3D, struts, []int{it.ownerStrutId}, apexSkipWeb); ok {
					end3D = snapPt
					snappedToInner = true
				}
				if start3D.DistTo(end3D) < 0.05 {
					continue
				}
				beadKind := "landing"
				if snappedToInner {
					beadKind = "inner"
				}
				out = append(out, Support{
					P1: start3D, P2: end3D, Kind: "webbing",
					AnchorIdx: it.anchorIdx, ApexSkip: apexSkipWeb, PadR: padR, BeadKind: beadKind,
				})
				placed2D = append(placed2D, [2][2]float64{start2D, bestEnd2D})
			}
		}
	}
	return out
}

// globalSupportVsSupportPass: phase 2 of the support pipeline. After
// intra-fan collisions were resolved in plan*Supports(), this pass handles
// inter-anchor collisions: pairs of supports from different anchors. The
// LATER support's tip snaps to the NEAREST endpoint of the EARLIER support,
// with a 'support' bead. Filters out supports that became too short.
func globalSupportVsSupportPass(s Settings, supports []Support) []Support {
	for i := 0; i < len(supports); i++ {
		me := &supports[i]
		for j := 0; j < i; j++ {
			ex := &supports[j]
			if me.AnchorIdx == ex.AnchorIdx {
				continue
			}
			myLen := me.P1.DistTo(me.P2)
			exLen := ex.P1.DistTo(ex.P2)
			skip := math.Max(me.ApexSkip, ex.ApexSkip)
			if myLen < skip*1.1 || exLen < skip*1.1 {
				continue
			}
			myDir := me.P2.Sub(me.P1).Normalize()
			exDir := ex.P2.Sub(ex.P1).Normalize()
			a1 := me.P1.Add(myDir.Mul(me.ApexSkip))
			a2 := me.P2
			b1 := ex.P1.Add(exDir.Mul(ex.ApexSkip))
			b2 := ex.P2
			threshold := s.ButtressR*2.0 + 0.005
			hit := ClosestApproach(a1, a2, b1, b2)
			if hit.Dist >= threshold {
				continue
			}
			fullHit := ClosestApproach(me.P1, me.P2, ex.P1, ex.P2)
			dToExP1 := fullHit.QOnEx.DistTo(ex.P1)
			dToExP2 := fullHit.QOnEx.DistTo(ex.P2)
			target := ex.P2
			if dToExP1 <= dToExP2 {
				target = ex.P1
			}
			me.P2 = target
			me.BeadKind = "support"
			break
		}
	}
	filtered := supports[:0]
	for _, ss := range supports {
		if ss.P1.DistTo(ss.P2) >= ss.PadR*0.4 {
			filtered = append(filtered, ss)
		}
	}
	return filtered
}

func renderSupport(s Settings, ss Support) *Group {
	g := NewGroup()
	baseColor := Colors.SkinWebbing
	if ss.Kind == "pyramid" {
		baseColor = Colors.PyramidSupport
	}
	baseMat := newMat(baseColor, false)
	l := ss.P1.DistTo(ss.P2)
	mid := ss.P1.Add(ss.P2).Mul(0.5)
	dirN := ss.P2.Sub(ss.P1).Normalize()
	addCylinder(g, mid, dirN, l, s.ButtressR*0.85, s.ButtressR*1.05, 16, baseMat)
	return g
}
