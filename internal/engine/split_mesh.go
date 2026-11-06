package engine

import (
	"math"
	"sort"
)

// Print splitting: the cut mesh. The skin is meshed once into an indexed,
// welded, watertight triangle mesh, and every cut then splits it exactly the
// way Blender's bisect does: the triangles crossing the cut's section are split
// at the plane, the section loop gets a separate copy of its vertices for each
// side, and each side is closed with a cap triangulated in the plane itself —
// so cut faces are exactly flat with sharp rims. Rod sockets and engraved
// numbers are pockets let into a cap as holes, with straight walls and a floor.
// Pieces are simply the connected components that remain.

type cutMesh struct {
	P   []Vec3
	T   [][3]int32 // dead triangles have T[i][0] < 0
	Tag []int32    // tagSkin, tagPocket, or a cap: cut id·2 + side

	cell   float64 // spatial grid of triangles by centroid
	grid   map[[3]int32][]int32
	maxTri float64 // longest triangle edge seen (grid query margin)
}

const (
	tagSkin   = -1
	tagPocket = -2
)

// Side 0 of a cut is its −N side, side 1 its +N side.
func capTag(id, side int) int32 { return int32(2*id + side) }

// flatField marks a field's mesh as needing face normals only.
type flatField struct{ ScalarField }

func (flatField) FlatNormals() bool { return true }

// newSkinMesh marches f over box at grid step cell and welds the triangle soup
// into an indexed mesh (marching cubes computes each shared edge vertex from
// both sides, so copies agree to rounding).
func newSkinMesh(f ScalarField, box AABB, cell float64, progress func(float64)) *cutMesh {
	var n [3]int
	for i := 0; i < 3; i++ {
		n[i] = maxI(int(math.Ceil((box.Max[i]-box.Min[i])/cell))+1, 3)
	}
	m := &cutMesh{}
	q := 1e-9
	weld := map[[3]int64]int32{}
	lookup := func(p Vec3) int32 {
		var k, near [3]int64
		for i := 0; i < 3; i++ {
			x := p[i] / q
			fl := math.Floor(x)
			k[i] = int64(fl)
			switch fr := x - fl; {
			case fr < 1e-3:
				near[i] = -1
			case fr > 1-1e-3:
				near[i] = 1
			}
		}
		if v, ok := weld[k]; ok {
			return v
		}
		for mask := 1; mask < 8; mask++ {
			kk, ok := k, true
			for i := 0; i < 3 && ok; i++ {
				if mask&(1<<i) != 0 {
					ok = near[i] != 0
					kk[i] += near[i]
				}
			}
			if ok {
				if v, hit := weld[kk]; hit {
					return v
				}
			}
		}
		v := int32(len(m.P))
		m.P = append(m.P, p)
		weld[k] = v
		return v
	}
	marchStreamN(flatField{f}, box, n[0], n[1], n[2], progress, func(tris []Triangle) {
		for _, t := range tris {
			a, b, c := lookup(t.A), lookup(t.B), lookup(t.C)
			if a == b || b == c || a == c {
				continue
			}
			m.T = append(m.T, [3]int32{a, b, c})
			m.Tag = append(m.Tag, tagSkin)
		}
	})
	m.cell = 0.1
	m.grid = map[[3]int32][]int32{}
	for t := range m.T {
		m.register(int32(t))
	}
	return m
}

func (m *cutMesh) clone() *cutMesh {
	c := &cutMesh{
		P:    append([]Vec3(nil), m.P...),
		T:    append([][3]int32(nil), m.T...),
		Tag:  append([]int32(nil), m.Tag...),
		cell: m.cell, maxTri: m.maxTri,
		grid: make(map[[3]int32][]int32, len(m.grid)),
	}
	for k, v := range m.grid {
		c.grid[k] = append([]int32(nil), v...)
	}
	return c
}

func (m *cutMesh) centroid(t int32) Vec3 {
	tr := m.T[t]
	return m.P[tr[0]].Add(m.P[tr[1]]).Add(m.P[tr[2]]).Mul(1.0 / 3)
}

func (m *cutMesh) cellOf(p Vec3) [3]int32 {
	return [3]int32{int32(math.Floor(p[0] / m.cell)), int32(math.Floor(p[1] / m.cell)), int32(math.Floor(p[2] / m.cell))}
}

func (m *cutMesh) register(t int32) {
	tr := m.T[t]
	for k := 0; k < 3; k++ {
		if l := m.P[tr[k]].DistTo(m.P[tr[(k+1)%3]]); l > m.maxTri {
			m.maxTri = l
		}
	}
	c := m.cellOf(m.centroid(t))
	m.grid[c] = append(m.grid[c], t)
}

func (m *cutMesh) addTri(a, b, c int32, tag int32) {
	m.T = append(m.T, [3]int32{a, b, c})
	m.Tag = append(m.Tag, tag)
	m.register(int32(len(m.T) - 1))
}

func (m *cutMesh) addVert(p Vec3) int32 {
	m.P = append(m.P, p)
	return int32(len(m.P) - 1)
}

// near lists the live triangles whose centroid lies within r of center.
func (m *cutMesh) near(center Vec3, r float64) []int32 {
	reach := r + m.maxTri
	lo := m.cellOf(center.Sub(V(reach, reach, reach)))
	hi := m.cellOf(center.Add(V(reach, reach, reach)))
	var out []int32
	for x := lo[0]; x <= hi[0]; x++ {
		for y := lo[1]; y <= hi[1]; y++ {
			for z := lo[2]; z <= hi[2]; z++ {
				for _, t := range m.grid[[3]int32{x, y, z}] {
					if m.T[t][0] >= 0 && m.centroid(t).DistTo(center) <= reach {
						out = append(out, t)
					}
				}
			}
		}
	}
	return out
}

// cutFrame is a cut plane with an in-plane frame: U × V = N, origin Q.
type cutFrame struct{ Q, N, U, V Vec3 }

func newCutFrame(q, n Vec3) cutFrame {
	n = n.Normalize()
	ref := V(1, 0, 0)
	if math.Abs(n[0]) > 0.8 {
		ref = V(0, 1, 0)
	}
	u := ref.Cross(n).Normalize()
	return cutFrame{Q: q, N: n, U: u, V: n.Cross(u)}
}

func (f cutFrame) to2(p Vec3) vec2 {
	d := p.Sub(f.Q)
	return vec2{d.Dot(f.U), d.Dot(f.V)}
}

func (f cutFrame) to3(p vec2, z float64) Vec3 {
	return f.Q.Add(f.U.Mul(p.X)).Add(f.V.Mul(p.Y)).Add(f.N.Mul(z))
}

// side is a vertex's signed distance to the plane; points on it count as +.
func (m *cutMesh) side(f cutFrame, v int32) float64 {
	s := m.P[v].Sub(f.Q).Dot(f.N)
	if math.Abs(s) < 1e-12 {
		s = 1e-12
	}
	return s
}

// secEdge is a mesh edge crossing the plane, by its two end vertices.
type secEdge struct{ neg, pos int32 }

func (m *cutMesh) edgePoint(f cutFrame, e secEdge) Vec3 {
	sn, sp := m.side(f, e.neg), m.side(f, e.pos)
	return Lerp(m.P[e.neg], m.P[e.pos], sn/(sn-sp))
}

// secLoop is one closed loop of a plane section: the crossed edges in order
// (counter-clockwise around the material seen from +N for an outer boundary,
// clockwise for a hole) and the triangle between each edge and the next.
type secLoop struct {
	edges []secEdge
	tris  []int32
	pts   []vec2
	area  float64 // signed: > 0 outer boundary, < 0 hole
}

// sectionLoops finds the closed loops where the plane crosses the candidate
// triangles. Chains that leave the candidate set are dropped.
func (m *cutMesh) sectionLoops(f cutFrame, cands []int32) []secLoop {
	type seg struct {
		to  secEdge
		tri int32
	}
	next := map[secEdge]seg{}
	var starts []secEdge
	for _, t := range cands {
		tr := m.T[t]
		s := [3]float64{m.side(f, tr[0]), m.side(f, tr[1]), m.side(f, tr[2])}
		var exit, entry secEdge
		crossing := false
		for k := 0; k < 3; k++ {
			a, b := tr[k], tr[(k+1)%3]
			sa, sb := s[k], s[(k+1)%3]
			if sa > 0 && sb < 0 {
				exit, crossing = secEdge{neg: b, pos: a}, true
			}
			if sa < 0 && sb > 0 {
				entry = secEdge{neg: a, pos: b}
			}
		}
		if crossing {
			next[exit] = seg{entry, t}
			starts = append(starts, exit)
		}
	}
	sort.Slice(starts, func(i, j int) bool {
		if starts[i].neg != starts[j].neg {
			return starts[i].neg < starts[j].neg
		}
		return starts[i].pos < starts[j].pos
	})
	done := map[secEdge]bool{}
	var loops []secLoop
	for _, e := range starts {
		if done[e] {
			continue
		}
		var lp secLoop
		cur, closed := e, false
		for {
			done[cur] = true
			sg, ok := next[cur]
			if !ok {
				break
			}
			lp.edges = append(lp.edges, cur)
			lp.tris = append(lp.tris, sg.tri)
			cur = sg.to
			if cur == e {
				closed = true
				break
			}
			if done[cur] {
				break
			}
		}
		if !closed || len(lp.edges) < 3 {
			continue
		}
		for _, ed := range lp.edges {
			lp.pts = append(lp.pts, f.to2(m.edgePoint(f, ed)))
		}
		lp.area = ringArea(lp.pts)
		loops = append(loops, lp)
	}
	return loops
}

// loopGroups pairs each outer loop with the hole loops directly inside it.
func loopGroups(loops []secLoop) [][]int {
	var groups [][]int
	for i, l := range loops {
		if l.area <= 0 {
			continue
		}
		g := []int{i}
		for j, h := range loops {
			if h.area < 0 && inRing(h.pts[0], l.pts) {
				g = append(g, j)
			}
		}
		groups = append(groups, g)
	}
	return groups
}

// pocket is a recess let into a cap: rings at the face (ring 0 the outline,
// counter-clockwise; any others are islands left standing inside it,
// clockwise) and the same rings at the bottom, depth below the face.
type pocket struct {
	top, bottom [][]vec2
	depth       float64
}

// cut splits the mesh along loops (outer boundary first, then its holes) of
// the plane f, caps both sides and lets in each side's pockets. It returns the
// (side 0, side 1) vertex pairs along the loops, for undoing the cut.
func (m *cutMesh) cut(id int, f cutFrame, loops []secLoop, pockets [2][]pocket) [][2]int32 {
	twin := map[secEdge][2]int32{}
	var twins [][2]int32
	for _, l := range loops {
		for _, e := range l.edges {
			p := m.edgePoint(f, e)
			tw := [2]int32{m.addVert(p), m.addVert(p)}
			twin[e] = tw
			twins = append(twins, tw)
		}
	}
	edgeOf := func(a, b int32) secEdge {
		if m.side(f, a) < 0 {
			return secEdge{neg: a, pos: b}
		}
		return secEdge{neg: b, pos: a}
	}
	for _, l := range loops {
		for _, t := range l.tris {
			tr := m.T[t]
			s := [3]bool{m.side(f, tr[0]) > 0, m.side(f, tr[1]) > 0, m.side(f, tr[2]) > 0}
			k := 0
			for ; k < 3; k++ {
				if s[k] != s[(k+1)%3] && s[k] != s[(k+2)%3] {
					break
				}
			}
			L, A, B := tr[k], tr[(k+1)%3], tr[(k+2)%3]
			x, y := twin[edgeOf(L, A)], twin[edgeOf(B, L)]
			sl, so := 0, 1 // L's side, the other side
			if s[k] {
				sl, so = 1, 0
			}
			tag := m.Tag[t]
			m.T[t] = [3]int32{L, x[sl], y[sl]}
			m.addTri(x[so], A, B, tag)
			m.addTri(x[so], B, y[so], tag)
		}
	}
	for side := 0; side < 2; side++ {
		m.capSide(id, side, f, loops, twin, pockets[side])
	}
	return twins
}

// capSide closes one side of a cut. Cap triangles come out of triangulate
// counter-clockwise in (U, V), i.e. facing +N: right for side 0, whose
// material is on −N, and flipped for side 1.
func (m *cutMesh) capSide(id, side int, f cutFrame, loops []secLoop, twin map[secEdge][2]int32, pockets []pocket) {
	tag := capTag(id, side)
	flip := side == 1
	emit := func(a, b, c int32, tag int32) {
		if flip {
			b, c = c, b
		}
		m.addTri(a, b, c, tag)
	}
	z := -1.0 // pocket depth direction along N
	if side == 1 {
		z = 1
	}
	var rings [][]vec2
	var idx []int32
	for _, l := range loops {
		rings = append(rings, l.pts)
		for _, e := range l.edges {
			idx = append(idx, twin[e][side])
		}
	}
	type ringVerts struct{ top, bot []int32 }
	pv := make([][]ringVerts, len(pockets))
	for pi, pk := range pockets {
		for ri := range pk.top {
			var rv ringVerts
			for i := range pk.top[ri] {
				rv.top = append(rv.top, m.addVert(f.to3(pk.top[ri][i], 0)))
				rv.bot = append(rv.bot, m.addVert(f.to3(pk.bottom[ri][i], z*pk.depth)))
			}
			pv[pi] = append(pv[pi], rv)
		}
		rings = append(rings, pk.top[0])
		idx = append(idx, pv[pi][0].top...)
	}
	for _, tr := range triangulate(rings) {
		emit(idx[tr[0]], idx[tr[1]], idx[tr[2]], tag)
	}
	for pi, pk := range pockets {
		// Islands standing inside the pocket get their own cap.
		for ri := 1; ri < len(pk.top); ri++ {
			for _, tr := range triangulate([][]vec2{pk.top[ri]}) {
				emit(pv[pi][ri].top[tr[0]], pv[pi][ri].top[tr[1]], pv[pi][ri].top[tr[2]], tag)
			}
		}
		// Walls: each ring keeps the pocket on its left, so the quad
		// (top i, top i+1, bottom i+1) faces into the pocket on side 0.
		for ri, rv := range pv[pi] {
			n := len(pk.top[ri])
			for i := 0; i < n; i++ {
				j := (i + 1) % n
				emit(rv.top[i], rv.top[j], rv.bot[j], tagPocket)
				emit(rv.top[i], rv.bot[j], rv.bot[i], tagPocket)
			}
		}
		// Floor, facing back out of the material.
		var bidx []int32
		for _, rv := range pv[pi] {
			bidx = append(bidx, rv.bot...)
		}
		for _, tr := range triangulate(pk.bottom) {
			emit(bidx[tr[0]], bidx[tr[1]], bidx[tr[2]], tagPocket)
		}
	}
}

// components labels every live triangle with its connected component.
func (m *cutMesh) components() ([]int32, int) {
	parent := make([]int32, len(m.P))
	for i := range parent {
		parent[i] = int32(i)
	}
	var find func(int32) int32
	find = func(x int32) int32 {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	for _, tr := range m.T {
		if tr[0] < 0 {
			continue
		}
		a, b, c := find(tr[0]), find(tr[1]), find(tr[2])
		parent[b] = a
		parent[find(c)] = a
	}
	label := map[int32]int32{}
	comp := make([]int32, len(m.T))
	for t, tr := range m.T {
		if tr[0] < 0 {
			comp[t] = -1
			continue
		}
		r := find(tr[0])
		l, ok := label[r]
		if !ok {
			l = int32(len(label))
			label[r] = l
		}
		comp[t] = l
	}
	return comp, len(label)
}

// uncut re-joins the given cuts: their caps go and each side's loop vertices
// are welded back onto the other side's.
func (m *cutMesh) uncut(ids map[int]bool, twins map[int][][2]int32) {
	if len(ids) == 0 {
		return
	}
	remap := map[int32]int32{}
	for id := range ids {
		for _, tw := range twins[id] {
			remap[tw[1]] = tw[0]
		}
	}
	for t, tr := range m.T {
		if tr[0] < 0 {
			continue
		}
		if tg := m.Tag[t]; tg >= 0 && ids[int(tg>>1)] {
			m.T[t][0] = -1
			continue
		}
		for k := 0; k < 3; k++ {
			if v, ok := remap[tr[k]]; ok {
				m.T[t][k] = v
			}
		}
	}
}

// boundaryRings returns the boundary of a set of coplanar triangles in the
// frame's 2D coordinates: outer rings counter-clockwise, holes clockwise.
func (m *cutMesh) boundaryRings(tris []int32, f cutFrame) [][]vec2 {
	type edge struct{ a, b int32 }
	has := map[edge]bool{}
	for _, t := range tris {
		tr := m.T[t]
		for k := 0; k < 3; k++ {
			has[edge{tr[k], tr[(k+1)%3]}] = true
		}
	}
	next := map[int32][]int32{}
	var starts []int32
	for e := range has {
		if !has[edge{e.b, e.a}] {
			next[e.a] = append(next[e.a], e.b)
			starts = append(starts, e.a)
		}
	}
	sort.Slice(starts, func(i, j int) bool { return starts[i] < starts[j] })
	var rings [][]vec2
	for _, s := range starts {
		if len(next[s]) == 0 {
			continue
		}
		var ring []vec2
		cur := s
		for len(next[cur]) > 0 {
			nb := next[cur]
			n := nb[len(nb)-1]
			next[cur] = nb[:len(nb)-1]
			ring = append(ring, f.to2(m.P[cur]))
			cur = n
			if cur == s {
				break
			}
		}
		if len(ring) >= 3 {
			rings = append(rings, ring)
		}
	}
	orientRings(rings)
	return rings
}

// orientRings makes rings at even nesting depth counter-clockwise and the
// others clockwise.
func orientRings(rings [][]vec2) {
	for i, r := range rings {
		d := 0
		for j, o := range rings {
			if i != j && inRing(r[0], o) {
				d++
			}
		}
		if (ringArea2(r) > 0) != (d%2 == 0) {
			reverseRing(r)
		}
	}
}

// triangles returns the live triangles with the given component label as
// world-space triangles with face normals.
func (m *cutMesh) triangles(sel []int32) []Triangle {
	out := make([]Triangle, 0, len(sel))
	for _, t := range sel {
		tr := m.T[t]
		a, b, c := m.P[tr[0]], m.P[tr[1]], m.P[tr[2]]
		n := b.Sub(a).Cross(c.Sub(a)).Normalize()
		out = append(out, Triangle{A: a, B: b, C: c, NA: n, NB: n, NC: n})
	}
	return out
}
