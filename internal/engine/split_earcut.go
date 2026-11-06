package engine

import (
	"math"
	"sort"
)

// Polygon triangulation for cut caps: ear clipping with holes bridged into the
// outer ring, after mapbox/earcut (ISC) — the same algorithm, ported without
// its z-order hashing (cap rings are a few hundred points).

type vec2 struct{ X, Y float64 }

func (a vec2) sub(b vec2) vec2        { return vec2{a.X - b.X, a.Y - b.Y} }
func (a vec2) dist(b vec2) float64    { return math.Hypot(a.X-b.X, a.Y-b.Y) }
func cross2(a, b vec2) float64        { return a.X*b.Y - a.Y*b.X }
func ringArea(r []vec2) float64       { return 0.5 * ringArea2(r) }
func lerp2(a, b vec2, t float64) vec2 { return vec2{a.X + (b.X-a.X)*t, a.Y + (b.Y-a.Y)*t} }

// ringArea2 is twice the signed area (positive counter-clockwise).
func ringArea2(r []vec2) float64 {
	s := 0.0
	for i := range r {
		j := (i + 1) % len(r)
		s += r[i].X*r[j].Y - r[j].X*r[i].Y
	}
	return s
}

// inRing is the even-odd point-in-polygon test.
func inRing(p vec2, r []vec2) bool {
	in := false
	for i, j := 0, len(r)-1; i < len(r); j, i = i, i+1 {
		a, b := r[i], r[j]
		if (a.Y > p.Y) != (b.Y > p.Y) && p.X < (b.X-a.X)*(p.Y-a.Y)/(b.Y-a.Y)+a.X {
			in = !in
		}
	}
	return in
}

// segDist2 is the distance from p to segment ab.
func segDist2(p, a, b vec2) float64 {
	ab := b.sub(a)
	t := 0.0
	if l2 := ab.X*ab.X + ab.Y*ab.Y; l2 > 0 {
		t = clampF((p.sub(a).X*ab.X+p.sub(a).Y*ab.Y)/l2, 0, 1)
	}
	return p.dist(lerp2(a, b, t))
}

type ecNode struct {
	i          int
	x, y       float64
	prev, next *ecNode
	steiner    bool
}

// triangulate fills the polygon rings[0] minus the holes rings[1:]. Indices
// refer to the rings' points in order (ring 0 first). Triangles come out
// counter-clockwise whatever the rings' own orientation.
func triangulate(rings [][]vec2) [][3]int {
	var tris [][3]int
	if len(rings) == 0 || len(rings[0]) < 3 {
		return nil
	}
	base := make([]int, len(rings))
	n := 0
	for k, r := range rings {
		base[k] = n
		n += len(r)
	}
	outer := ecRing(rings[0], base[0], true)
	if outer == nil || outer.next == outer.prev {
		return nil
	}
	if len(rings) > 1 {
		var queue []*ecNode
		for k := 1; k < len(rings); k++ {
			l := ecRing(rings[k], base[k], false)
			if l == nil {
				continue
			}
			if l == l.next {
				l.steiner = true
			}
			queue = append(queue, ecLeftmost(l))
		}
		sort.SliceStable(queue, func(a, b int) bool { return queue[a].x < queue[b].x })
		for _, h := range queue {
			outer = ecEliminateHole(h, outer)
		}
	}
	ecLinked(outer, &tris, 0)
	return restoreSkipped(tris, base, rings)
}

// restoreSkipped puts back ring points the clipper dropped as duplicate or
// collinear: each run of them lies on the edge of one triangle between its
// kept neighbours, which is fanned out through them. A mesh cap must use every
// point of its ring, or it no longer meets the faces beside it.
func restoreSkipped(tris [][3]int, base []int, rings [][]vec2) [][3]int {
	used := map[int]bool{}
	for _, t := range tris {
		used[t[0]], used[t[1]], used[t[2]] = true, true, true
	}
	for k, r := range rings {
		n := len(r)
		start := -1
		for i := 0; i < n; i++ {
			if used[base[k]+i] {
				start = i
				break
			}
		}
		if start < 0 {
			continue
		}
		for i := 0; i < n; {
			p := (start + i) % n
			var run []int
			j := i + 1
			for ; j <= n && !used[base[k]+(start+j)%n]; j++ {
				run = append(run, base[k]+(start+j)%n)
			}
			q := (start + j) % n
			if len(run) > 0 {
				tris = fanEdge(tris, base[k]+p, base[k]+q, run)
			}
			i = j
		}
	}
	return tris
}

// fanEdge splits the triangle holding edge a–b through the points of run
// (in order from a to b).
func fanEdge(tris [][3]int, a, b int, run []int) [][3]int {
	for ti, t := range tris {
		for e := 0; e < 3; e++ {
			x, y, c := t[e], t[(e+1)%3], t[(e+2)%3]
			var chain []int
			switch {
			case x == a && y == b:
				chain = append(append([]int{a}, run...), b)
			case x == b && y == a:
				chain = []int{b}
				for i := len(run) - 1; i >= 0; i-- {
					chain = append(chain, run[i])
				}
				chain = append(chain, a)
			default:
				continue
			}
			tris[ti] = [3]int{chain[0], chain[1], c}
			for i := 1; i+1 < len(chain); i++ {
				tris = append(tris, [3]int{chain[i], chain[i+1], c})
			}
			return tris
		}
	}
	return tris
}

// ecRing links a ring counter-clockwise (outer) or clockwise (hole).
func ecRing(r []vec2, base int, ccw bool) *ecNode {
	var last *ecNode
	if (ringArea2(r) > 0) == ccw {
		for i := range r {
			last = ecInsert(base+i, r[i].X, r[i].Y, last)
		}
	} else {
		for i := len(r) - 1; i >= 0; i-- {
			last = ecInsert(base+i, r[i].X, r[i].Y, last)
		}
	}
	if last != nil && ecEquals(last, last.next) {
		ecRemove(last)
		last = last.next
	}
	return last
}

func ecLinked(ear *ecNode, tris *[][3]int, pass int) {
	if ear == nil {
		return
	}
	stop := ear
	for ear.prev != ear.next {
		prev, next := ear.prev, ear.next
		if ecIsEar(ear) {
			*tris = append(*tris, [3]int{prev.i, ear.i, next.i})
			ecRemove(ear)
			ear = next.next
			stop = next.next
			continue
		}
		ear = next
		if ear == stop {
			switch pass {
			case 0:
				ecLinked(ecFilter(ear, nil), tris, 1)
			case 1:
				ear = ecCureLocal(ecFilter(ear, nil), tris)
				ecLinked(ear, tris, 2)
			case 2:
				ecSplit(ear, tris)
			}
			break
		}
	}
}

// ecArea is earcut's orientation test: negative for a left (convex) turn.
func ecArea(p, q, r *ecNode) float64 {
	return (q.y-p.y)*(r.x-q.x) - (q.x-p.x)*(r.y-q.y)
}

func ecEquals(a, b *ecNode) bool { return a.x == b.x && a.y == b.y }

func ecIsEar(ear *ecNode) bool {
	a, b, c := ear.prev, ear, ear.next
	if ecArea(a, b, c) >= 0 {
		return false // reflex
	}
	x0, x1 := math.Min(a.x, math.Min(b.x, c.x)), math.Max(a.x, math.Max(b.x, c.x))
	y0, y1 := math.Min(a.y, math.Min(b.y, c.y)), math.Max(a.y, math.Max(b.y, c.y))
	for p := c.next; p != a; p = p.next {
		if p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1 &&
			!(p.x == a.x && p.y == a.y) &&
			ecInTri(a.x, a.y, b.x, b.y, c.x, c.y, p.x, p.y) &&
			ecArea(p.prev, p, p.next) >= 0 {
			return false
		}
	}
	return true
}

func ecInTri(ax, ay, bx, by, cx, cy, px, py float64) bool {
	return (cx-px)*(ay-py) >= (ax-px)*(cy-py) &&
		(ax-px)*(by-py) >= (bx-px)*(ay-py) &&
		(bx-px)*(cy-py) >= (cx-px)*(by-py)
}

// ecFilter drops duplicate and collinear points.
func ecFilter(start, end *ecNode) *ecNode {
	if start == nil {
		return nil
	}
	if end == nil {
		end = start
	}
	p := start
	for {
		again := false
		if !p.steiner && (ecEquals(p, p.next) || ecArea(p.prev, p, p.next) == 0) {
			ecRemove(p)
			p = p.prev
			end = p
			if p == p.next {
				break
			}
			again = true
		} else {
			p = p.next
		}
		if !again && p == end {
			break
		}
	}
	return end
}

func ecCureLocal(start *ecNode, tris *[][3]int) *ecNode {
	p := start
	for {
		a, b := p.prev, p.next.next
		if !ecEquals(a, b) && ecIntersects(a, p, p.next, b) && ecLocallyInside(a, b) && ecLocallyInside(b, a) {
			*tris = append(*tris, [3]int{a.i, p.i, b.i})
			ecRemove(p)
			ecRemove(p.next)
			p, start = b, b
		}
		p = p.next
		if p == start {
			break
		}
	}
	return ecFilter(p, nil)
}

func ecSplit(start *ecNode, tris *[][3]int) {
	a := start
	for {
		for b := a.next.next; b != a.prev; b = b.next {
			if a.i != b.i && ecValidDiagonal(a, b) {
				c := ecSplitPolygon(a, b)
				a = ecFilter(a, a.next)
				c = ecFilter(c, c.next)
				ecLinked(a, tris, 0)
				ecLinked(c, tris, 0)
				return
			}
		}
		a = a.next
		if a == start {
			return
		}
	}
}

func ecEliminateHole(hole, outer *ecNode) *ecNode {
	bridge := ecHoleBridge(hole, outer)
	if bridge == nil {
		return outer
	}
	rev := ecSplitPolygon(bridge, hole)
	ecFilter(rev, rev.next)
	return ecFilter(bridge, bridge.next)
}

func ecHoleBridge(hole, outer *ecNode) *ecNode {
	p := outer
	hx, hy := hole.x, hole.y
	qx := math.Inf(-1)
	var m *ecNode
	if ecEquals(hole, p) {
		return p
	}
	for {
		if ecEquals(hole, p.next) {
			return p.next
		} else if hy <= p.y && hy >= p.next.y && p.next.y != p.y {
			x := p.x + (hy-p.y)*(p.next.x-p.x)/(p.next.y-p.y)
			if x <= hx && x > qx {
				qx = x
				m = p
				if p.next.x < p.x {
					m = p.next
				}
				if x == hx {
					return m
				}
			}
		}
		p = p.next
		if p == outer {
			break
		}
	}
	if m == nil {
		return nil
	}
	stop := m
	mx, my := m.x, m.y
	tanMin := math.Inf(1)
	p = m
	for {
		ax, cx := qx, hx
		if hy < my {
			ax, cx = hx, qx
		}
		if hx >= p.x && p.x >= mx && hx != p.x && ecInTri(ax, hy, mx, my, cx, hy, p.x, p.y) {
			tan := math.Abs(hy-p.y) / (hx - p.x)
			if ecLocallyInside(p, hole) &&
				(tan < tanMin || tan == tanMin && (p.x > m.x || p.x == m.x && ecSectorContains(m, p))) {
				m = p
				tanMin = tan
			}
		}
		p = p.next
		if p == stop {
			break
		}
	}
	return m
}

func ecSectorContains(m, p *ecNode) bool {
	return ecArea(m.prev, m, p.prev) < 0 && ecArea(p.next, m, m.next) < 0
}

func ecLeftmost(start *ecNode) *ecNode {
	p, left := start, start
	for {
		if p.x < left.x || p.x == left.x && p.y < left.y {
			left = p
		}
		p = p.next
		if p == start {
			return left
		}
	}
}

func ecValidDiagonal(a, b *ecNode) bool {
	return a.next.i != b.i && a.prev.i != b.i && !ecIntersectsPolygon(a, b) &&
		(ecLocallyInside(a, b) && ecLocallyInside(b, a) && ecMiddleInside(a, b) &&
			(ecArea(a.prev, a, b.prev) != 0 || ecArea(a, b.prev, b) != 0) ||
			ecEquals(a, b) && ecArea(a.prev, a, a.next) > 0 && ecArea(b.prev, b, b.next) > 0)
}

func ecSign(v float64) int {
	switch {
	case v > 0:
		return 1
	case v < 0:
		return -1
	}
	return 0
}

func ecIntersects(p1, q1, p2, q2 *ecNode) bool {
	o1 := ecSign(ecArea(p1, q1, p2))
	o2 := ecSign(ecArea(p1, q1, q2))
	o3 := ecSign(ecArea(p2, q2, p1))
	o4 := ecSign(ecArea(p2, q2, q1))
	if o1 != o2 && o3 != o4 {
		return true
	}
	return o1 == 0 && ecOnSegment(p1, p2, q1) || o2 == 0 && ecOnSegment(p1, q2, q1) ||
		o3 == 0 && ecOnSegment(p2, p1, q2) || o4 == 0 && ecOnSegment(p2, q1, q2)
}

func ecOnSegment(p, q, r *ecNode) bool {
	return q.x <= math.Max(p.x, r.x) && q.x >= math.Min(p.x, r.x) && q.y <= math.Max(p.y, r.y) && q.y >= math.Min(p.y, r.y)
}

func ecIntersectsPolygon(a, b *ecNode) bool {
	p := a
	for {
		if p.i != a.i && p.next.i != a.i && p.i != b.i && p.next.i != b.i && ecIntersects(p, p.next, a, b) {
			return true
		}
		p = p.next
		if p == a {
			return false
		}
	}
}

func ecLocallyInside(a, b *ecNode) bool {
	if ecArea(a.prev, a, a.next) < 0 {
		return ecArea(a, b, a.next) >= 0 && ecArea(a, a.prev, b) >= 0
	}
	return ecArea(a, b, a.prev) < 0 || ecArea(a, a.next, b) < 0
}

func ecMiddleInside(a, b *ecNode) bool {
	p := a
	in := false
	px, py := (a.x+b.x)/2, (a.y+b.y)/2
	for {
		if (p.y > py) != (p.next.y > py) && p.next.y != p.y && px < (p.next.x-p.x)*(py-p.y)/(p.next.y-p.y)+p.x {
			in = !in
		}
		p = p.next
		if p == a {
			return in
		}
	}
}

// ecSplitPolygon links a to b with a bridge, duplicating both; returns b's copy.
func ecSplitPolygon(a, b *ecNode) *ecNode {
	a2 := &ecNode{i: a.i, x: a.x, y: a.y}
	b2 := &ecNode{i: b.i, x: b.x, y: b.y}
	an, bp := a.next, b.prev
	a.next, b.prev = b, a
	a2.next, an.prev = an, a2
	b2.next, a2.prev = a2, b2
	bp.next, b2.prev = b2, bp
	return b2
}

func ecInsert(i int, x, y float64, last *ecNode) *ecNode {
	p := &ecNode{i: i, x: x, y: y}
	if last == nil {
		p.prev, p.next = p, p
	} else {
		p.next = last.next
		p.prev = last
		last.next.prev = p
		last.next = p
	}
	return p
}

func ecRemove(p *ecNode) {
	p.next.prev = p.prev
	p.prev.next = p.next
}
