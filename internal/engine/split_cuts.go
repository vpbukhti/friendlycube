package engine

import (
	"math"
	"sort"
	"strings"
	"sync"
)

// Print splitting, stage 3: every cut left between pieces is a numbered glue
// joint. It gets one or two guide-rod sockets (the main rod if it fits with a
// wall around it, else the thin rod) and its number engraved on both faces,
// all laid out on the exact 2D outline of the cut. Where a joint's split plane
// crosses a cut face, the face is still one joint: its sockets and number may
// straddle the seam between the two halves (glued together first).

// SplitCut is one glued joint.
type SplitCut struct {
	Num      int // 0 = glue-only sliver, too small for a rod or a number
	A, B     int // main piece on the −N side and on the +N side
	Sides    [2][]int
	Cut      int // the plan cut it lies on
	Q, N     Vec3
	U, V     Vec3 // in-plane frame, U × V = N
	Keys     []Vec3
	Rod      string // "main" | "thin" | "" (no rod fits)
	Label    Vec3
	LabelH   float64
	HasLabel bool

	f        cutFrame
	region   [][]vec2 // the cut's outline in (U, V)
	seams    map[int]bool
	keys2    []vec2
	label2   vec2
	labelRot bool    // label turned 90°
	labelSW  float64 // engraving stroke width
	dash2    vec2    // dash key: groove centre, along U (V if labelRot)
	area     float64
}

// Dash key: where a cut has no room for a rod and its number together, the
// bar under the number becomes a groove in both faces, and a loose printed
// key — one size for every such cut — sits half in each.
const (
	dashWidthMM = 1.0
	dashLenMM   = 3.0
	dashDepthMM = 1.0 // key half-height: how far it reaches into each face
	dashGlueMM  = 0.2 // extra groove depth for glue
)

func (pl *SplitPlan) dashBar() labelBar {
	return labelBar{W: dashWidthMM / pl.MMPerUnit, L: dashLenMM / pl.MMPerUnit}
}

// dashKey is the key's size in world units (width, length, height across
// the cut): its grooves less ClearanceMM on every side, like the rods.
func (pl *SplitPlan) dashKey() (w, l, h float64) {
	c := 2 * pl.Print.ClearanceMM
	mm := pl.MMPerUnit
	return (dashWidthMM - c) / mm, (dashLenMM - c) / mm, 2 * dashDepthMM / mm
}

// boxMesh is an axis-aligned box w × l × h centred on the origin, outward.
func boxMesh(w, l, h float64) []Triangle {
	c := [8]Vec3{}
	for i := range c {
		c[i] = V(w/2*float64(2*(i&1)-1), l/2*float64(2*(i>>1&1)-1), h/2*float64(2*(i>>2&1)-1))
	}
	quads := [6][4]int{{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}}
	var out []Triangle
	for _, q := range quads {
		for _, t := range [2][3]int{{q[0], q[1], q[2]}, {q[0], q[2], q[3]}} {
			a, b, cc := c[t[0]], c[t[1]], c[t[2]]
			n := b.Sub(a).Cross(cc.Sub(a)).Normalize()
			out = append(out, Triangle{A: a, B: b, C: cc, NA: n, NB: n, NC: n})
		}
	}
	return out
}

// rodShape is a rod's (or its socket's) cross-section in world units: base and
// top widths, height, distance from base to the section's centroid, half-length.
type rodShape struct {
	B, T, H, Yc, Lh, Taper float64
	Base                   float64 // the base line's offset from the key point (across V)
}

// rodShape is a socket as specified, or its rod: the socket's cross-section
// inset by ClearanceMM on every side (base, top and both slopes).
func (pl *SplitPlan) rodShape(r RodSpec, socket bool) rodShape {
	mm := pl.MMPerUnit
	B, T, H := r.BaseMM/mm, r.TopMM/mm, r.HeightMM/mm
	yc := func(B, T, H float64) float64 { return H * (2*T + B) / (3 * (B + T)) }
	lh := r.LengthMM / 2 / mm
	if socket {
		lh += 0.3 / mm // room for glue at the bottom of each socket
		return rodShape{B: B, T: T, H: H, Yc: yc(B, T, H), Base: -yc(B, T, H), Lh: lh, Taper: r.Taper}
	}
	c := pl.Print.ClearanceMM / mm
	k := (B - T) / (2 * H)        // side slope
	in := c * math.Sqrt(1+k*k)    // horizontal inset of a slope moved c along its normal
	b := 2 * (B/2 - k*c - in)     // width at the rod's base (c above the socket's)
	t := 2 * (B/2 - k*(H-c) - in) // width at its top (c below the socket's)
	return rodShape{B: b, T: t, H: H - 2*c, Yc: yc(b, t, H-2*c), Base: -yc(B, T, H) + c, Lh: lh, Taper: r.Taper}
}

// footprint is the radius around the key point the cross-section reaches.
func (s rodShape) footprint() float64 {
	return math.Max(math.Hypot(s.B/2, s.Yc), math.Hypot(s.T/2, s.H-s.Yc))
}

// outline is the cross-section at key k in the cut plane: the base along U,
// counter-clockwise; scale shrinks it toward its base line (the rod's taper).
func (s rodShape) outline(k vec2, scale float64) []vec2 {
	y0 := k.Y + s.Base
	return []vec2{
		{k.X - scale*s.B/2, y0}, {k.X + scale*s.B/2, y0},
		{k.X + scale*s.T/2, y0 + scale*s.H}, {k.X - scale*s.T/2, y0 + scale*s.H},
	}
}

// rodMesh is the exact rod in its own frame, oriented for printing: base on
// z = 0, length along y, in millimetres (scale mm per unit).
func rodMesh(s rodShape, mm float64) []Triangle {
	sec := func(z, sc float64) [4]Vec3 {
		return [4]Vec3{
			V(-sc*s.B/2, 0, z), V(sc*s.B/2, 0, z),
			V(sc*s.T/2, sc*s.H, z), V(-sc*s.T/2, sc*s.H, z),
		}
	}
	secs := [3][4]Vec3{sec(-s.Lh, s.Taper), sec(0, 1), sec(s.Lh, s.Taper)}
	center := V(0, s.H/3, 0)
	var tris []Triangle
	tri := func(a, b, c Vec3) {
		n := b.Sub(a).Cross(c.Sub(a))
		if n.Dot(a.Add(b).Add(c).Mul(1.0/3).Sub(center)) < 0 {
			b, c = c, b
			n = n.Neg()
		}
		n = n.Normalize()
		// print frame: x, length → y, height → z
		m := func(v Vec3) Vec3 { return V(v[0]*mm, -v[2]*mm, v[1]*mm) }
		nn := V(n[0], -n[2], n[1])
		tris = append(tris, Triangle{A: m(a), B: m(b), C: m(c), NA: nn, NB: nn, NC: nn})
	}
	for k := 0; k < 2; k++ {
		a, b := secs[k], secs[k+1]
		for i := 0; i < 4; i++ {
			j := (i + 1) % 4
			tri(a[i], a[j], b[j])
			tri(a[i], b[j], b[i])
		}
	}
	for _, e := range []int{0, 2} {
		q := secs[e]
		tri(q[0], q[1], q[2])
		tri(q[0], q[2], q[3])
	}
	return tris
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	s := ""
	for ; n > 0; n /= 10 {
		s = string(rune('0'+n%10)) + s
	}
	return s
}

// strokeWidth is the engraving stroke for character height h, never under
// 0.25 mm (a 0.25 mm nozzle's line; thinner grooves vanish in the slicer).
func (pl *SplitPlan) strokeWidth(h float64) float64 {
	return math.Max(0.18*h, 0.25/pl.MMPerUnit)
}

// ---- layout ----

// layoutCuts turns every cut left between pieces into a SplitCut with its
// rods and number, then builds each cut's pockets.
func (pl *SplitPlan) layoutCuts(m, base *cutMesh) {
	byTag := map[int32][]int32{}
	for t, tg := range m.Tag {
		if tg >= 0 && m.T[t][0] >= 0 {
			byTag[tg] = append(byTag[tg], int32(t))
		}
	}
	type side struct {
		pieces map[int]float64 // piece → contact area
		atoms  map[int]bool
	}
	kept := map[int]*[2]side{}
	for _, c := range pl.contacts {
		pa, pb := pl.pieceOf[c.A], pl.pieceOf[c.B]
		if pa == pb && !c.Part {
			continue
		}
		k := kept[c.Cut]
		if k == nil {
			k = &[2]side{{map[int]float64{}, map[int]bool{}}, {map[int]float64{}, map[int]bool{}}}
			kept[c.Cut] = k
		}
		k[0].pieces[pa] += c.Area
		k[1].pieces[pb] += c.Area
		k[0].atoms[c.A], k[1].atoms[c.B] = true, true
	}
	pl.sections = map[int][][]vec2{}
	var ids []int
	for id := range kept {
		ids = append(ids, id)
		pl.sections[id] = m.boundaryRings(byTag[capTag(id, 0)], pl.cuts[id].F)
	}
	// The split planes crossing a cut face: sockets may cross those.
	seams := map[int]map[int]bool{}
	for _, id := range ids {
		for s := 0; s < 2; s++ {
			if len(kept[id][s].atoms) < 2 {
				continue
			}
			for _, c := range pl.contacts {
				if pl.cuts[c.Cut].Split && kept[id][s].atoms[c.A] && kept[id][s].atoms[c.B] {
					if seams[id] == nil {
						seams[id] = map[int]bool{}
					}
					seams[id][c.Cut] = true
				}
			}
		}
	}
	main := func(ps map[int]float64) (int, []int) {
		best, ba := -1, -1.0
		var all []int
		for p, a := range ps {
			all = append(all, p)
			if a > ba || a == ba && p < best {
				best, ba = p, a
			}
		}
		sort.Ints(all)
		return best, all
	}
	cuts := make([]*SplitCut, len(ids))
	for i, id := range ids {
		f := pl.cuts[id].F
		sc := &SplitCut{Cut: id, Q: f.Q, N: f.N, U: f.U, V: f.V, f: f, region: pl.sections[id], seams: seams[id]}
		sc.A, sc.Sides[0] = main(kept[id][0].pieces)
		sc.B, sc.Sides[1] = main(kept[id][1].pieces)
		for _, r := range sc.region {
			sc.area += ringArea(r)
		}
		cuts[i] = sc
	}
	sort.SliceStable(cuts, func(i, j int) bool {
		if cuts[i].A != cuts[j].A {
			return cuts[i].A < cuts[j].A
		}
		if cuts[i].B != cuts[j].B {
			return cuts[i].B < cuts[j].B
		}
		return cuts[i].Cut < cuts[j].Cut
	})
	digits := len(itoa(len(cuts)))
	layout := func(split bool) {
		var wg sync.WaitGroup
		for _, sc := range cuts {
			if pl.cuts[sc.Cut].Split != split {
				continue
			}
			wg.Add(1)
			go func(sc *SplitCut) {
				defer wg.Done()
				pl.layoutCut(sc, digits)
			}(sc)
		}
		wg.Wait()
	}
	// Branch cuts first. Their sockets and numbers may cross a joint's split
	// plane, so split faces are laid out on a mesh that already has them.
	layout(false)
	pl.Cuts = cuts
	pl.buildPockets()
	if m2 := base.clone(); true {
		if _, _, err := pl.replay(m2); err == nil {
			faces := map[int32][]int32{}
			for t, tg := range m2.Tag {
				if tg >= 0 && tg&1 == 0 && m2.T[t][0] >= 0 && pl.cuts[tg>>1].Split {
					faces[tg] = append(faces[tg], int32(t))
				}
			}
			for _, sc := range cuts {
				if pl.cuts[sc.Cut].Split {
					sc.region = m2.boundaryRings(faces[capTag(sc.Cut, 0)], sc.f)
					pl.sections[sc.Cut] = sc.region
				}
			}
		}
	}
	layout(true)

	// Every joint is numbered, even one too small for a rod.
	pl.Cuts = cuts
	for i, c := range cuts {
		c.Num = i + 1
	}
	for _, c := range pl.Cuts {
		pl.fitLabel(c)
		if c.HasLabel {
			c.Label = c.f.to3(c.label2, 0)
		}
	}
	pl.buildPockets()
}

// clrGrid samples a contact outline: each cell's distance to its edge
// (0 outside).
type clrGrid struct {
	lo     vec2
	gs     float64
	nx, ny int
	clr    []float64
}

func newClrGrid(rings [][]vec2, gs float64) *clrGrid {
	lo := vec2{math.Inf(1), math.Inf(1)}
	hi := vec2{math.Inf(-1), math.Inf(-1)}
	for _, r := range rings {
		for _, p := range r {
			lo.X, lo.Y = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y)
			hi.X, hi.Y = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y)
		}
	}
	g := &clrGrid{lo: vec2{lo.X - gs, lo.Y - gs}, gs: gs}
	g.nx = int((hi.X-lo.X)/gs) + 3
	g.ny = int((hi.Y-lo.Y)/gs) + 3
	g.clr = make([]float64, g.nx*g.ny)
	inf := math.MaxFloat64 / 4
	// Inside: scanline crossings of every ring edge (even-odd).
	for j := 0; j < g.ny; j++ {
		y := g.lo.Y + float64(j)*gs
		var xs []float64
		for _, r := range rings {
			for i := range r {
				a, b := r[i], r[(i+1)%len(r)]
				if (a.Y > y) != (b.Y > y) {
					xs = append(xs, a.X+(y-a.Y)*(b.X-a.X)/(b.Y-a.Y))
				}
			}
		}
		sort.Float64s(xs)
		for k := 0; k+1 < len(xs); k += 2 {
			i0 := int(math.Ceil((xs[k] - g.lo.X) / gs))
			i1 := int(math.Floor((xs[k+1] - g.lo.X) / gs))
			for i := maxI(i0, 0); i <= i1 && i < g.nx; i++ {
				g.clr[j*g.nx+i] = inf
			}
		}
	}
	// Every outline edge blocks the cells it passes, so holes smaller than a
	// cell (where a neighbour's pocket grazes this plane) still count.
	for _, r := range rings {
		for i := range r {
			a, b := r[i], r[(i+1)%len(r)]
			k := int(a.dist(b)/(0.5*gs)) + 1
			for j := 0; j <= k; j++ {
				p := lerp2(a, b, float64(j)/float64(k))
				x := int(math.Round((p.X - g.lo.X) / gs))
				y := int(math.Round((p.Y - g.lo.Y) / gs))
				if x >= 0 && y >= 0 && x < g.nx && y < g.ny {
					g.clr[y*g.nx+x] = 0
				}
			}
		}
	}
	// Chamfer distance to the nearest outside cell.
	n := g.nx
	d1, d2 := gs, gs*math.Sqrt2
	at := func(x, y int) float64 {
		if x < 0 || y < 0 || x >= g.nx || y >= g.ny {
			return 0
		}
		return g.clr[y*n+x]
	}
	for pass := 0; pass < 2; pass++ {
		s := 1
		if pass == 1 {
			s = -1
		}
		for k := 0; k < len(g.clr); k++ {
			i := k
			if pass == 1 {
				i = len(g.clr) - 1 - k
			}
			if g.clr[i] == 0 {
				continue
			}
			x, y := i%n, i/n
			v := g.clr[i]
			v = math.Min(v, at(x-s, y)+d1)
			v = math.Min(v, at(x, y-s)+d1)
			v = math.Min(v, at(x-s, y-s)+d2)
			v = math.Min(v, at(x+s, y-s)+d2)
			g.clr[i] = v
		}
	}
	for i, v := range g.clr {
		if v > 0 {
			g.clr[i] = math.Max(v-gs, 0) // the edge may lie anywhere in its blocked cell
		}
	}
	return g
}

func (g *clrGrid) pos(i int) vec2 {
	return vec2{g.lo.X + float64(i%g.nx)*g.gs, g.lo.Y + float64(i/g.nx)*g.gs}
}

func (g *clrGrid) at(p vec2) float64 {
	i := int(math.Round((p.X - g.lo.X) / g.gs))
	j := int(math.Round((p.Y - g.lo.Y) / g.gs))
	if i < 0 || j < 0 || i >= g.nx || j >= g.ny {
		return 0
	}
	return g.clr[j*g.nx+i]
}

// fits reports whether the outline ring stays at least margin inside.
func (g *clrGrid) fits(ring []vec2, margin float64) bool {
	for i := range ring {
		a, b := ring[i], ring[(i+1)%len(ring)]
		n := int(a.dist(b)/g.gs) + 1
		for k := 0; k <= n; k++ {
			if g.at(lerp2(a, b, float64(k)/float64(n))) < margin {
				return false
			}
		}
	}
	return true
}

// order lists the cells with at least min clearance, deepest first.
func (g *clrGrid) order(min float64) []int {
	var idx []int
	for i, v := range g.clr {
		if v >= min {
			idx = append(idx, i)
		}
	}
	sort.SliceStable(idx, func(a, b int) bool { return g.clr[idx[a]] > g.clr[idx[b]] })
	return idx
}

func (pl *SplitPlan) layoutCut(sc *SplitCut, digits int) {
	if len(sc.region) == 0 {
		return
	}
	cg := pl.clrGrid(sc.region)
	for _, rod := range []struct {
		name string
		spec RodSpec
	}{{"main", pl.Print.MainRod}, {"thin", pl.Print.ThinRod}} {
		if keys := pl.placeKeys(sc, cg, rod.spec, digits); len(keys) > 0 {
			sc.keys2, sc.Rod = keys, rod.name
			for _, k := range keys {
				sc.Keys = append(sc.Keys, sc.f.to3(k, 0))
			}
			return
		}
		sc.HasLabel = false
	}
}

func (pl *SplitPlan) clrGrid(region [][]vec2) *clrGrid {
	lo := vec2{math.Inf(1), math.Inf(1)}
	hi := vec2{math.Inf(-1), math.Inf(-1)}
	for _, r := range region {
		for _, p := range r {
			lo.X, lo.Y = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y)
			hi.X, hi.Y = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y)
		}
	}
	return newClrGrid(region, math.Max(0.25/pl.MMPerUnit, math.Max(hi.X-lo.X, hi.Y-lo.Y)/120))
}

// labelRect is the label footprint at center c (turned 90° if rot).
func labelRect(c vec2, w, h float64, rot bool) []vec2 {
	if rot {
		w, h = h, w
	}
	return []vec2{{c.X - w/2, c.Y - h/2}, {c.X + w/2, c.Y - h/2}, {c.X + w/2, c.Y + h/2}, {c.X - w/2, c.Y + h/2}}
}

// clearOf reports whether ring a stays gap away from ring b (sampled).
func clearOf(a, b []vec2, gap, step float64) bool {
	for i := range a {
		p, q := a[i], a[(i+1)%len(a)]
		n := int(p.dist(q)/step) + 1
		for k := 0; k <= n; k++ {
			x := lerp2(p, q, float64(k)/float64(n))
			if inRing(x, b) {
				return false
			}
			for j := range b {
				if segDist2(x, b[j], b[(j+1)%len(b)]) < gap {
					return false
				}
			}
		}
	}
	return !inRing(b[0], a)
}

// overlapsAny reports whether ring r comes within gap of any of the rings
// (by bounding boxes: sockets and labels are both near-rectangles).
func overlapsAny(r []vec2, rings [][]vec2, gap float64) bool {
	box := func(q []vec2) (vec2, vec2) {
		lo, hi := q[0], q[0]
		for _, p := range q[1:] {
			lo.X, lo.Y = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y)
			hi.X, hi.Y = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y)
		}
		return lo, hi
	}
	rl, rh := box(r)
	for _, k := range rings {
		kl, kh := box(k)
		if rl.X < kh.X+gap && kl.X < rh.X+gap && rl.Y < kh.Y+gap && kl.Y < rh.Y+gap {
			return true
		}
	}
	return false
}

// placeKeys finds up to two key spots for rod r together with a spot for the
// cut's number: the first key among the deepest spots, chosen to leave room
// for the largest number; the second as far from the first as still fits.
func (pl *SplitPlan) placeKeys(sc *SplitCut, cg *clrGrid, r RodSpec, digits int) []vec2 {
	sh := pl.rodShape(r, true)
	wall := r.WallMM / pl.MMPerUnit
	idx := cg.order(wall + 0.5*math.Min(sh.H, sh.T))
	if len(idx) == 0 {
		return nil
	}
	fits := func(p vec2) bool { return cg.fits(sh.outline(p, 1), wall) && pl.socketFits(sc, p, sh, wall) }
	stride := maxI(1, len(idx)/60)
	var key vec2
	found := false
	bestH := -1.0
	for t := 0; t < len(idx); t += stride {
		p := cg.pos(idx[t])
		if !fits(p) {
			continue
		}
		if !found {
			key, found = p, true // fallback: deepest fitting spot
		}
		if !pl.Print.Engrave {
			break
		}
		if h, at, rot, ok := pl.labelSpot(cg, digits, [][]vec2{sh.outline(p, 1)}, false); ok && h > bestH+1e-12 {
			bestH, key = h, p
			sc.label2, sc.labelRot, sc.LabelH, sc.HasLabel = at, rot, h, true
			if h >= 2/pl.MMPerUnit-1e-12 {
				break
			}
		}
	}
	if !found {
		return nil
	}
	keys := []vec2{key}
	var lr []vec2
	if sc.HasLabel {
		sw := pl.strokeWidth(sc.LabelH)
		w, ht, _, _ := labelBox(strings.Repeat("8", digits), sc.LabelH, sw, strokeBar(sw))
		lr = labelRect(sc.label2, w, ht, sc.labelRot)
	}
	sort.SliceStable(idx, func(i, j int) bool { return cg.pos(idx[i]).dist(key) > cg.pos(idx[j]).dist(key) })
	for t, i := range idx {
		if t > 400 {
			break
		}
		p := cg.pos(i)
		o := sh.outline(p, 1)
		if !clearOf(o, sh.outline(key, 1), wall, cg.gs) {
			break // nearer ones won't clear either
		}
		if lr != nil && !clearOf(o, lr, 0.5/pl.MMPerUnit, cg.gs) {
			continue
		}
		if fits(p) {
			keys = append(keys, p)
			break
		}
	}
	return keys
}

// labelSpot finds the largest character height (2 mm down to 1 mm) at which
// a number of the given digit count fits in the contact clear of the keys,
// upright or turned 90°.
func (pl *SplitPlan) labelSpot(cg *clrGrid, digits int, keys [][]vec2, guide bool) (float64, vec2, bool, bool) {
	mm := pl.MMPerUnit
	margin := 0.4 / mm
	text := strings.Repeat("8", digits)
	for _, hmm := range []float64{2, 1.75, 1.5, 1.25, 1} {
		h := hmm / mm
		sw := pl.strokeWidth(h)
		bar := strokeBar(sw)
		if guide {
			bar = pl.dashBar()
		}
		w, ht, _, _ := labelBox(text, h, sw, bar)
		for _, rot := range []bool{false, true} {
			// Deepest spots first. Those overlapping a socket are skipped
			// without counting toward the budget: on a round face with its
			// socket in the middle, every deep spot is taken and the room
			// left is out by the rim.
			tries := 0
			for _, i := range cg.order(0.5*math.Min(w, ht) + margin) {
				c := cg.pos(i)
				r := labelRect(c, w, ht, rot)
				if overlapsAny(r, keys, 0.5/mm) {
					continue
				}
				if tries++; tries > 300 {
					break
				}
				if cg.fits(r, margin) {
					return h, c, rot, true
				}
			}
		}
	}
	return 0, vec2{}, false, false
}

// fitLabel places the number on a numbered cut that got no label alongside
// its rods (or has none): now that the number is known it may be shorter.
func (pl *SplitPlan) fitLabel(c *SplitCut) {
	if !pl.Print.Engrave || c.Num == 0 || c.HasLabel || len(c.region) == 0 {
		return
	}
	var keys [][]vec2
	if c.Rod != "" {
		spec := pl.Print.MainRod
		if c.Rod == "thin" {
			spec = pl.Print.ThinRod
		}
		sh := pl.rodShape(spec, true)
		for _, k := range c.keys2 {
			keys = append(keys, sh.outline(k, 1))
		}
	}
	cg := pl.clrGrid(c.region)
	digits := len(itoa(c.Num))
	if c.Rod == "" {
		// No rod: a dash key gives it alignment and a number at once.
		if pl.dashGuide(c, cg, digits); c.HasLabel {
			return
		}
	}
	if h, at, rot, ok := pl.labelSpot(cg, digits, keys, false); ok {
		c.label2, c.labelRot, c.LabelH, c.HasLabel = at, rot, h, true
		return
	}
	// Still no room: lay the rods out again for the real number (shorter
	// than the widest one they made room for), falling back to the thin rod
	// if the main one leaves no room at all, and then to a guide dash. Not on
	// a face a split plane crosses: the split faces were laid out around
	// these sockets.
	defer pl.dashGuide(c, cg, digits)
	if len(c.seams) > 0 || pl.cuts[c.Cut].Split {
		return
	}
	specs := []struct {
		name string
		spec RodSpec
	}{{"main", pl.Print.MainRod}, {"thin", pl.Print.ThinRod}}
	if c.Rod == "thin" {
		specs = specs[1:]
	}
	for _, r := range specs {
		trial := *c
		trial.HasLabel = false
		if keys := pl.placeKeys(&trial, cg, r.spec, digits); len(keys) > 0 && trial.HasLabel {
			c.keys2, c.Rod, c.Keys = keys, r.name, nil
			for _, k := range keys {
				c.Keys = append(c.Keys, c.f.to3(k, 0))
			}
			c.label2, c.labelRot, c.LabelH, c.HasLabel = trial.label2, trial.labelRot, trial.LabelH, true
			return
		}
	}
}

// dashGuide gives a cut with no room for a rod and its number together a
// dash key in place of the rod, if its grooves reach solid material.
func (pl *SplitPlan) dashGuide(c *SplitCut, cg *clrGrid, digits int) {
	if c.HasLabel {
		return
	}
	h, at, rot, ok := pl.labelSpot(cg, digits, nil, true)
	if !ok {
		return
	}
	sw := pl.strokeWidth(h)
	text := itoa(c.Num)
	r := dashRect(text, h, sw, pl.dashBar())
	ctr := vec2{0.5 * (r[0].X + r[2].X), 0.5 * (r[0].Y + r[2].Y)}
	if rot {
		ctr = vec2{-ctr.Y, ctr.X}
	}
	ctr = vec2{at.X + ctr.X, at.Y + ctr.Y}
	// The groove's corners and centre, at full depth on both sides, crossing
	// no other cut — not even a split seam: the faces on a joint's split
	// plane were laid out before this groove existed.
	seams := c.seams
	c.seams = nil
	defer func() { c.seams = seams }()
	depth := (dashDepthMM + dashGlueMM) / pl.MMPerUnit
	for _, g := range append(r, vec2{0.5 * (r[0].X + r[2].X), 0.5 * (r[0].Y + r[2].Y)}) {
		if rot {
			g = vec2{-g.Y, g.X}
		}
		p := vec2{at.X + g.X, at.Y + g.Y}
		for _, d := range []float64{-depth, depth} {
			q := c.f.to3(p, 1.3*d)
			if pl.skin.Eval(q) >= 0 || pl.crossesCut(c, c.f.to3(p, 0), q) {
				return
			}
		}
	}
	c.Rod, c.Keys, c.keys2 = "dash", []Vec3{c.f.to3(ctr, 0)}, nil
	c.dash2 = ctr
	c.label2, c.labelRot, c.LabelH, c.HasLabel = at, rot, h, true
}

// socketFits checks the socket at key p reaches into solid material on both
// sides, with a wall around it, crossing no other cut.
func (pl *SplitPlan) socketFits(sc *SplitCut, p vec2, sh rodShape, wall float64) bool {
	rx := sh.footprint() + 0.5*wall
	for _, dir := range []float64{-1, 1} {
		for _, z := range []float64{0.35, 0.7, 1} {
			depth := dir * z * (sh.Lh + 0.5*wall)
			for k := 0; k < 9; k++ {
				q2 := p
				if k > 0 {
					th := 2 * math.Pi * float64(k) / 8
					q2 = vec2{p.X + rx*math.Cos(th), p.Y + rx*math.Sin(th)}
				}
				q := sc.f.to3(q2, depth)
				if pl.skin.Eval(q) >= 0 || pl.crossesCut(sc, sc.f.to3(q2, 0), q) {
					return false
				}
			}
		}
	}
	return true
}

// crossesCut reports whether segment ab passes through the section of
// another kept cut (other than the seams of sc's own face).
func (pl *SplitPlan) crossesCut(sc *SplitCut, a, b Vec3) bool {
	for id, rings := range pl.sections {
		if id == sc.Cut || sc.seams[id] {
			continue
		}
		f := pl.cuts[id].F
		sa, sb := a.Sub(f.Q).Dot(f.N), b.Sub(f.Q).Dot(f.N)
		if (sa > 0) == (sb > 0) {
			continue
		}
		x := f.to2(Lerp(a, b, sa/(sa-sb)))
		in := false
		for _, r := range rings {
			if inRing(x, r) {
				in = !in
			}
		}
		if in {
			return true
		}
	}
	return false
}

// buildPockets turns every cut's rods and number into the pockets let into
// each side of it.
func (pl *SplitPlan) buildPockets() {
	for _, c := range pl.cuts {
		c.pockets = [2][]pocket{}
	}
	for _, sc := range pl.Cuts {
		pc := pl.cuts[sc.Cut]
		spec := pl.Print.MainRod
		if sc.Rod == "thin" {
			spec = pl.Print.ThinRod
		}
		sh := pl.rodShape(spec, true)
		for _, k := range sc.keys2 {
			pk := pocket{top: [][]vec2{sh.outline(k, 1)}, bottom: [][]vec2{sh.outline(k, sh.Taper)}, depth: sh.Lh}
			pc.pockets[0] = append(pc.pockets[0], pk)
			pc.pockets[1] = append(pc.pockets[1], pk)
		}
		if !sc.HasLabel {
			continue
		}
		sc.labelSW = pl.strokeWidth(sc.LabelH)
		text := itoa(sc.Num)
		if sc.Num == 0 {
			text = strings.Repeat("8", len(itoa(len(pl.Cuts)))) // not numbered yet: the widest it can be
		}
		dash := sc.Rod == "dash"
		bar := strokeBar(sc.labelSW)
		if dash {
			bar = pl.dashBar()
		}
		o := numberOutline(text, sc.LabelH, sc.labelSW, bar, !dash)
		depth := pl.Print.EngraveDepthMM / pl.MMPerUnit
		for side := 0; side < 2; side++ {
			// Side 0 is read looking at it from +N (U right, V up); side 1
			// from −N, so its copy is mirrored.
			place := func(g vec2) vec2 {
				if sc.labelRot {
					g = vec2{-g.Y, g.X}
				}
				if side == 1 {
					return vec2{sc.label2.X - g.X, sc.label2.Y + g.Y}
				}
				return vec2{sc.label2.X + g.X, sc.label2.Y + g.Y}
			}
			rings := make([][]vec2, len(o.rings))
			for i, r := range o.rings {
				for _, g := range r {
					rings[i] = append(rings[i], place(g))
				}
				if side == 1 {
					reverseRing(rings[i])
				}
			}
			for i, r := range rings {
				if o.depth[i] != 0 {
					continue
				}
				pk := pocket{top: [][]vec2{r}, depth: depth}
				for j, h := range rings {
					if o.depth[j] == 1 && inRing(h[0], r) {
						pk.top = append(pk.top, h)
					}
				}
				pk.bottom = pk.top
				pc.pockets[side] = append(pc.pockets[side], pk)
			}
			if dash {
				var r []vec2
				for _, g := range dashRect(text, sc.LabelH, sc.labelSW, bar) {
					r = append(r, place(g))
				}
				if ringArea(r) < 0 {
					reverseRing(r)
				}
				d := (dashDepthMM + dashGlueMM) / pl.MMPerUnit
				pc.pockets[side] = append(pc.pockets[side], pocket{top: [][]vec2{r}, bottom: [][]vec2{r}, depth: d})
			}
		}
	}
	// Turn every pocket by a few micro-radians, each by its own amount: no
	// two outlines may share an exact line (two sockets on one grid row, the
	// feet of "12"), or the cap triangulation degenerates between them.
	for _, c := range pl.cuts {
		for side := 0; side < 2; side++ {
			for k := range c.pockets[side] {
				pk := &c.pockets[side][k]
				o := pk.top[0][0]
				a := 1.3e-4 * float64(k+1)
				turn := func(rs [][]vec2) [][]vec2 {
					out := make([][]vec2, len(rs))
					for i, r := range rs {
						for _, p := range r {
							d := p.sub(o)
							out[i] = append(out[i], vec2{o.X + d.X*math.Cos(a) - d.Y*math.Sin(a), o.Y + d.X*math.Sin(a) + d.Y*math.Cos(a)})
						}
					}
					return out
				}
				pk.top, pk.bottom = turn(pk.top), turn(pk.bottom)
			}
		}
	}
}

// RodCounts reports how many rods of each kind, and dash keys, the plan uses.
func (pl *SplitPlan) RodCounts() (main, thin, dash int) {
	for _, c := range pl.Cuts {
		switch c.Rod {
		case "main":
			main += len(c.Keys)
		case "thin":
			thin += len(c.Keys)
		case "dash":
			dash++
		}
	}
	return
}

// rodTris places the rod mesh in every socket of cut c (assembled), or its
// dash key in its grooves.
func (pl *SplitPlan) rodTris(c *SplitCut) []Triangle {
	var out []Triangle
	if c.Rod == "dash" {
		w, l, h := pl.dashKey()
		ax, ay := c.U, c.V // across, along; ax × ay = N keeps the box outward
		if !c.labelRot {
			ax, ay = c.V.Neg(), c.U
		}
		k := c.f.to3(c.dash2, 0)
		for _, t := range boxMesh(w, l, h) {
			m := func(v Vec3) Vec3 { return k.Add(ax.Mul(v[0])).Add(ay.Mul(v[1])).Add(c.N.Mul(v[2])) }
			n := ax.Mul(t.NA[0]).Add(ay.Mul(t.NA[1])).Add(c.N.Mul(t.NA[2]))
			out = append(out, Triangle{A: m(t.A), B: m(t.B), C: m(t.C), NA: n, NB: n, NC: n})
		}
		return out
	}
	spec := pl.Print.MainRod
	if c.Rod == "thin" {
		spec = pl.Print.ThinRod
	}
	sh := pl.rodShape(spec, false)
	local := rodMesh(sh, 1)
	for _, k := range c.Keys {
		// rodMesh's print frame: x → U, −y (length) → N, z (height above the base) → V.
		m := func(v Vec3) Vec3 {
			return k.Add(c.U.Mul(v[0])).Add(c.N.Mul(-v[1])).Add(c.V.Mul(v[2] + sh.Base))
		}
		mn := func(v Vec3) Vec3 { return c.U.Mul(v[0]).Add(c.N.Mul(-v[1])).Add(c.V.Mul(v[2])) }
		for _, t := range local {
			out = append(out, Triangle{A: m(t.A), B: m(t.B), C: m(t.C), NA: mn(t.NA), NB: mn(t.NB), NC: mn(t.NC)})
		}
	}
	return out
}
