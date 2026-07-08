package engine

import "math"

// Strut is one accepted strut, possibly shortened via length factor.
type Strut struct {
	ID       int
	A, B     Vec3
	Lf       float64
	SnappedA bool
	SnappedB bool
	KindGen  string // "random" or "edge"
}

func pickLengthFactor(s Settings, rand *Rand) float64 {
	ws := [5]float64{s.Lf100, s.Lf95, s.Lf90, s.Lf85, s.Lf70}
	factors := [5]float64{1.00, 0.95, 0.90, 0.85, 0.70}
	total := 0.0
	for _, w := range ws {
		if w > 0 {
			total += w
		}
	}
	if total < 1e-9 {
		return 1.0
	}
	r := rand.F() * total
	for i := 0; i < 5; i++ {
		w := ws[i]
		if w < 0 {
			w = 0
		}
		r -= w
		if r <= 0 {
			return factors[i]
		}
	}
	return factors[4]
}

func pickPointOnEdge(t *CubeTopology, edgeIdx int, edgeBias float64, rand *Rand) Vec3 {
	e := t.Edges[edgeIdx]
	tt := math.Pow(rand.F(), edgeBias)
	if rand.F() < 0.5 {
		tt = 1 - tt
	}
	tt = clampF(tt, 0.01, 0.99)
	return Lerp(e.A, e.B, tt)
}

func pickOppositeEdge(t *CubeTopology, srcEdgeIdx int, neighborWeight float64, rand *Rand, banSet map[int]bool) int {
	weights := make([]float64, len(t.Edges))
	total := 0.0
	for i := range t.Edges {
		if i == srcEdgeIdx {
			continue
		}
		if banSet[i] {
			continue
		}
		sc := t.EdgesShareCount(srcEdgeIdx, i)
		if sc >= 2 {
			continue
		}
		w := 1.0
		if sc == 1 {
			w = neighborWeight
		}
		weights[i] = w
		total += w
	}
	if total < 1e-9 {
		return -1
	}
	r := rand.F() * total
	for i, w := range weights {
		r -= w
		if r <= 0 && w > 0 {
			return i
		}
	}
	return len(weights) - 1
}

type rawStrut struct {
	a, b               Vec3
	aSnapped, bSnapped bool
}

func generateRandomLineRaw(half, cubeSize float64, rand *Rand) (Vec3, Vec3, bool) {
	randR := func() float64 { return rand.F()*2 - 1 }
	for i := 0; i < 60; i++ {
		dir := V(randR(), randR(), randR())
		if dir.Len() < 0.2 {
			continue
		}
		dir = dir.Normalize()
		p := V(randR()*half*0.7, randR()*half*0.7, randR()*half*0.7)
		e1, e2, ok := ClipLineToCube(p, dir, half)
		if !ok {
			continue
		}
		if e1.DistTo(e2) < cubeSize*0.6 {
			continue
		}
		return e1, e2, true
	}
	return Vec3{}, Vec3{}, false
}

func recastFromAnchor(_ *CubeTopology, anchor Vec3, cubeSize, half float64, rand *Rand) (Vec3, bool) {
	randR := func() float64 { return rand.F()*2 - 1 }
	for tries := 0; tries < 40; tries++ {
		dir := V(randR(), randR(), randR())
		if dir.Len() < 0.2 {
			continue
		}
		dir = dir.Normalize()
		e1, e2, ok := ClipLineToCube(anchor, dir, half)
		if !ok {
			continue
		}
		dA := anchor.DistTo(e1)
		dB := anchor.DistTo(e2)
		other := e2
		if dA > dB {
			other = e1
		}
		if anchor.DistTo(other) < cubeSize*0.5 {
			continue
		}
		return other, true
	}
	return Vec3{}, false
}

func generateRandomLineStrut(t *CubeTopology, s Settings, presnapR, wireSnapR, half float64, rand *Rand) (rawStrut, bool) {
	a, b, ok := generateRandomLineRaw(half, s.CubeSize, rand)
	if !ok {
		return rawStrut{}, false
	}
	snA := t.SnapRandomLineEndpoint(a, presnapR, wireSnapR)
	if snA.EdgeMidSnapped {
		return rawStrut{}, false
	}
	A := snA.Point
	aSnapped := snA.VertexSnapped
	snB := t.SnapRandomLineEndpoint(b, presnapR, wireSnapR)
	B := snB.Point
	bSnapped := snB.VertexSnapped
	if snB.EdgeMidSnapped {
		ok = false
		for attempt := 0; attempt < 8; attempt++ {
			newB, rcOK := recastFromAnchor(t, A, s.CubeSize, half, rand)
			if !rcOK {
				continue
			}
			snNew := t.SnapRandomLineEndpoint(newB, presnapR, wireSnapR)
			if snNew.EdgeMidSnapped {
				continue
			}
			B = snNew.Point
			bSnapped = snNew.VertexSnapped
			ok = true
			break
		}
		if !ok {
			return rawStrut{}, false
		}
	}
	return rawStrut{a: A, b: B, aSnapped: aSnapped, bSnapped: bSnapped}, true
}

func generateEdgeToEdgeStrut(t *CubeTopology, s Settings, edgeCylinderR float64, rand *Rand) (rawStrut, bool) {
	srcIdx := int(rand.F() * float64(len(t.Edges)))
	if srcIdx >= len(t.Edges) {
		srcIdx = len(t.Edges) - 1
	}
	ptA := pickPointOnEdge(t, srcIdx, s.EdgeBias, rand)
	ecA := t.SnapToEdgeCylinder(ptA, edgeCylinderR)
	endA := ecA.Point
	snappedToVertexA := false
	for _, v := range t.Verts {
		if v.DistTo(endA) < OnEdgeEps {
			snappedToVertexA = true
			break
		}
	}
	edgesA := t.EdgesContaining(endA)
	if len(edgesA) == 0 {
		return rawStrut{}, false
	}
	banned := map[int]bool{}
	for attempt := 0; attempt < 10; attempt++ {
		dstIdx := pickOppositeEdge(t, srcIdx, s.NeighborWeight, rand, banned)
		if dstIdx < 0 {
			return rawStrut{}, false
		}
		ptB := pickPointOnEdge(t, dstIdx, s.EdgeBias, rand)
		ecB := t.SnapToEdgeCylinder(ptB, edgeCylinderR)
		endB := ecB.Point
		snappedToVertexB := false
		for _, v := range t.Verts {
			if v.DistTo(endB) < OnEdgeEps {
				snappedToVertexB = true
				break
			}
		}
		edgesB := t.EdgesContaining(endB)
		shared := []int{}
		for _, ai := range edgesA {
			for _, bi := range edgesB {
				if ai == bi {
					shared = append(shared, ai)
				}
			}
		}
		if len(shared) > 0 {
			banned[dstIdx] = true
			for _, x := range shared {
				banned[x] = true
			}
			continue
		}
		return rawStrut{a: endA, b: endB, aSnapped: snappedToVertexA, bSnapped: snappedToVertexB}, true
	}
	return rawStrut{}, false
}

func GenerateStruts(topo *CubeTopology, s Settings, seed uint32) []Strut {
	rand := NewRand(seed)
	half := s.CubeSize / 2
	presnapR := s.CubeSize * s.VertexPresnapPct
	wireSnapR := s.StrutR * s.WireSnapMult
	edgeCylR := s.CubeSize * s.EdgeCylinderPct
	minClearance := s.StrutR * s.MinClearance
	target := s.Target
	if target < 1 {
		target = 1
	}
	const maxTries = 1500
	var struts []Strut
	nextId := 0
	for tries := 0; tries < maxTries && len(struts) < target; tries++ {
		isEdgeToEdge := rand.F() < s.Mix
		var built rawStrut
		var ok bool
		if isEdgeToEdge {
			built, ok = generateEdgeToEdgeStrut(topo, s, edgeCylR, rand)
		} else {
			built, ok = generateRandomLineStrut(topo, s, presnapR, wireSnapR, half, rand)
		}
		if !ok {
			continue
		}
		endA, endB := built.a, built.b
		if endA.DistTo(endB) < s.CubeSize*0.5 {
			continue
		}
		lf := 1.0
		shrunkA, shrunkB := endA, endB
		if !isEdgeToEdge {
			lf = pickLengthFactor(s, rand)
			if lf < 0.999 {
				mid := endA.Add(endB).Mul(0.5)
				shrunkA = mid.Add(endA.Sub(mid).Mul(lf))
				shrunkB = mid.Add(endB.Sub(mid).Mul(lf))
			}
		}
		clear := true
		for _, st := range struts {
			if SegSegDist(shrunkA, shrunkB, st.A, st.B) < minClearance {
				clear = false
				break
			}
		}
		if !clear {
			continue
		}
		kind := "random"
		if isEdgeToEdge {
			kind = "edge"
		}
		struts = append(struts, Strut{
			ID: nextId, A: shrunkA, B: shrunkB, Lf: lf,
			SnappedA: built.aSnapped && lf == 1.0,
			SnappedB: built.bSnapped && lf == 1.0,
			KindGen:  kind,
		})
		nextId++
	}
	return struts
}
