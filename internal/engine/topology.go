package engine

import "math"

type CubeEdge struct {
	A, B Vec3
}

type CubeFace struct {
	Idx    int
	Normal Vec3
	U, V   Vec3
}

type CubeTopology struct {
	Verts         []Vec3
	Edges         []CubeEdge
	EdgeMidpoints []Vec3
	Faces         []CubeFace
}

func BuildCubeTopology(half float64) CubeTopology {
	t := CubeTopology{}
	for _, x := range []float64{-half, half} {
		for _, y := range []float64{-half, half} {
			for _, z := range []float64{-half, half} {
				t.Verts = append(t.Verts, V(x, y, z))
			}
		}
	}
	for i := 0; i < len(t.Verts); i++ {
		for j := i + 1; j < len(t.Verts); j++ {
			diff := 0
			if t.Verts[i][0] != t.Verts[j][0] {
				diff++
			}
			if t.Verts[i][1] != t.Verts[j][1] {
				diff++
			}
			if t.Verts[i][2] != t.Verts[j][2] {
				diff++
			}
			if diff == 1 {
				t.Edges = append(t.Edges, CubeEdge{A: t.Verts[i], B: t.Verts[j]})
			}
		}
	}
	for _, e := range t.Edges {
		t.EdgeMidpoints = append(t.EdgeMidpoints, e.A.Add(e.B).Mul(0.5))
	}
	t.Faces = []CubeFace{
		{Idx: 0, Normal: V(1, 0, 0), U: V(0, 1, 0), V: V(0, 0, 1)},
		{Idx: 1, Normal: V(-1, 0, 0), U: V(0, 1, 0), V: V(0, 0, 1)},
		{Idx: 2, Normal: V(0, 1, 0), U: V(1, 0, 0), V: V(0, 0, 1)},
		{Idx: 3, Normal: V(0, -1, 0), U: V(1, 0, 0), V: V(0, 0, 1)},
		{Idx: 4, Normal: V(0, 0, 1), U: V(1, 0, 0), V: V(0, 1, 0)},
		{Idx: 5, Normal: V(0, 0, -1), U: V(1, 0, 0), V: V(0, 1, 0)},
	}
	return t
}

const OnEdgeEps = 0.005

func (t *CubeTopology) EdgesShareCount(i, j int) int {
	if i == j {
		return 2
	}
	ea, eb := t.Edges[i], t.Edges[j]
	shared := 0
	if ea.A.Equals(eb.A) || ea.A.Equals(eb.B) {
		shared++
	}
	if ea.B.Equals(eb.A) || ea.B.Equals(eb.B) {
		shared++
	}
	return shared
}

// EdgesContaining returns the indices of cube edges whose segments contain p
// (perpendicular distance < OnEdgeEps after clamping the projection).
func (t *CubeTopology) EdgesContaining(p Vec3) []int {
	var out []int
	for i, e := range t.Edges {
		ab := e.B.Sub(e.A)
		abLen2 := ab.Dot(ab)
		tParam := clampF(p.Sub(e.A).Dot(ab)/abLen2, 0, 1)
		proj := e.A.Add(ab.Mul(tParam))
		if p.DistTo(proj) < OnEdgeEps {
			out = append(out, i)
		}
	}
	return out
}

// PyramidFaceFor: which cube face is most facing point p?
func (t *CubeTopology) PyramidFaceFor(p Vec3) *CubeFace {
	best := -1
	bestVal := math.Inf(-1)
	for i, fc := range t.Faces {
		v := p.Dot(fc.Normal)
		if v > bestVal {
			bestVal = v
			best = i
		}
	}
	if best < 0 {
		return nil
	}
	return &t.Faces[best]
}

// ClassifyBoundary: 0=interior, 1=face, 2=edge, 3=vertex.
func (t *CubeTopology) ClassifyBoundary(p Vec3, half float64) int {
	const eps = 0.015
	c := 0
	if math.Abs(math.Abs(p[0])-half) < eps {
		c++
	}
	if math.Abs(math.Abs(p[1])-half) < eps {
		c++
	}
	if math.Abs(math.Abs(p[2])-half) < eps {
		c++
	}
	return c
}

// FaceForBoundaryPoint: choose the face whose normal best aligns with outward
// among the faces this boundary point lies on.
func (t *CubeTopology) FaceForBoundaryPoint(p Vec3, outward Vec3, half float64) *CubeFace {
	const eps = 0.015
	var cands []*CubeFace
	for i := range t.Faces {
		if math.Abs(p.Dot(t.Faces[i].Normal)-half) < eps {
			cands = append(cands, &t.Faces[i])
		}
	}
	if len(cands) == 0 {
		return nil
	}
	if len(cands) == 1 {
		return cands[0]
	}
	best := cands[0]
	bestDot := math.Inf(-1)
	for _, fc := range cands {
		d := fc.Normal.Dot(outward)
		if d > bestDot {
			bestDot = d
			best = fc
		}
	}
	return best
}

// SegIntersect2D returns intersection of segments p1->p2 and q1->q2 in 2D.
// Inputs are 2-vectors stored as Vec3 with z=0.
func SegIntersect2D(p1, p2, q1, q2 [2]float64) (point [2]float64, t1 float64, ok bool) {
	rvx, rvy := p2[0]-p1[0], p2[1]-p1[1]
	sx, sy := q2[0]-q1[0], q2[1]-q1[1]
	rxs := rvx*sy - rvy*sx
	if math.Abs(rxs) < 1e-9 {
		return [2]float64{}, 0, false
	}
	qpx, qpy := q1[0]-p1[0], q1[1]-p1[1]
	t := (qpx*sy - qpy*sx) / rxs
	u := (qpx*rvy - qpy*rvx) / rxs
	if t < 0 || t > 1 || u < 0 || u > 1 {
		return [2]float64{}, 0, false
	}
	return [2]float64{p1[0] + rvx*t, p1[1] + rvy*t}, t, true
}

// ClosestApproach: segment-to-segment closest-point pair.
type CloseHit struct {
	Dist         float64
	POnAB, QOnEx Vec3
}

func ClosestApproach(p1a, p1b, p2a, p2b Vec3) CloseHit {
	u := p1b.Sub(p1a)
	v := p2b.Sub(p2a)
	w := p1a.Sub(p2a)
	a := u.Dot(u)
	b := u.Dot(v)
	c := v.Dot(v)
	d := u.Dot(w)
	e := v.Dot(w)
	D := a*c - b*b
	var sc, tc float64
	if D < 1e-7 {
		sc = 0
		if b > c {
			tc = d / b
		} else {
			tc = e / c
		}
	} else {
		sc = (b*e - c*d) / D
		tc = (a*e - b*d) / D
	}
	sc = clampF(sc, 0, 1)
	tc = clampF(tc, 0, 1)
	pOnAB := p1a.Add(u.Mul(sc))
	qOnEx := p2a.Add(v.Mul(tc))
	return CloseHit{Dist: pOnAB.DistTo(qOnEx), POnAB: pOnAB, QOnEx: qOnEx}
}

func SegSegDist(p1a, p1b, p2a, p2b Vec3) float64 {
	return ClosestApproach(p1a, p1b, p2a, p2b).Dist
}

// ClipLineToCube: parametric AABB clip with the cube [-half, +half]^3.
// Returns (entry, exit, ok).
func ClipLineToCube(p, d Vec3, half float64) (Vec3, Vec3, bool) {
	tEnter := math.Inf(-1)
	tExit := math.Inf(1)
	for i := 0; i < 3; i++ {
		pi := p[i]
		di := d[i]
		if math.Abs(di) < 1e-8 {
			if pi < -half || pi > half {
				return Vec3{}, Vec3{}, false
			}
			continue
		}
		t1 := (-half - pi) / di
		t2 := (half - pi) / di
		tLo, tHi := t1, t2
		if tLo > tHi {
			tLo, tHi = tHi, tLo
		}
		if tLo > tEnter {
			tEnter = tLo
		}
		if tHi < tExit {
			tExit = tHi
		}
		if tEnter > tExit {
			return Vec3{}, Vec3{}, false
		}
	}
	return p.Add(d.Mul(tEnter)), p.Add(d.Mul(tExit)), true
}

// ---- snapping ----

type SnapResult struct {
	Point   Vec3
	Snapped bool
	Kind    string // "vertex" or "edgeMid" or ""
}

func (t *CubeTopology) SnapToWireFeature(p Vec3, radius float64) SnapResult {
	bestPt := p
	bestD := radius
	bestSnap := false
	bestKind := ""
	for _, v := range t.Verts {
		d := p.DistTo(v)
		if d < bestD {
			bestD = d
			bestPt = v
			bestSnap = true
			bestKind = "vertex"
		}
	}
	for _, m := range t.EdgeMidpoints {
		d := p.DistTo(m)
		if d < bestD {
			bestD = d
			bestPt = m
			bestSnap = true
			bestKind = "edgeMid"
		}
	}
	return SnapResult{Point: bestPt, Snapped: bestSnap, Kind: bestKind}
}

func (t *CubeTopology) SnapToVertex(p Vec3, radius float64) SnapResult {
	bestPt := p
	bestD := radius
	bestSnap := false
	for _, v := range t.Verts {
		d := p.DistTo(v)
		if d < bestD {
			bestD = d
			bestPt = v
			bestSnap = true
		}
	}
	return SnapResult{Point: bestPt, Snapped: bestSnap}
}

func (t *CubeTopology) SnapToEdgeCylinder(p Vec3, radius float64) SnapResult {
	bestPt := p
	bestD := radius
	bestSnap := false
	for _, e := range t.Edges {
		ab := e.B.Sub(e.A)
		abLen2 := ab.Dot(ab)
		tParam := clampF(p.Sub(e.A).Dot(ab)/abLen2, 0, 1)
		proj := e.A.Add(ab.Mul(tParam))
		d := p.DistTo(proj)
		if d < bestD {
			bestD = d
			bestPt = proj
			bestSnap = true
		}
	}
	return SnapResult{Point: bestPt, Snapped: bestSnap}
}

type RandomEndpointResult struct {
	Point          Vec3
	VertexSnapped  bool
	EdgeMidSnapped bool
}

func (t *CubeTopology) SnapRandomLineEndpoint(p Vec3, presnapR, wireSnapR float64) RandomEndpointResult {
	pre := t.SnapToVertex(p, presnapR)
	if pre.Snapped {
		return RandomEndpointResult{Point: pre.Point, VertexSnapped: true}
	}
	w := t.SnapToWireFeature(p, wireSnapR)
	if w.Snapped && w.Kind == "vertex" {
		return RandomEndpointResult{Point: w.Point, VertexSnapped: true}
	}
	if w.Snapped && w.Kind == "edgeMid" {
		return RandomEndpointResult{Point: w.Point, EdgeMidSnapped: true}
	}
	return RandomEndpointResult{Point: p}
}
