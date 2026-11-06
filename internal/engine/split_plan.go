package engine

import (
	"fmt"
	"math"
	"sort"
	"sync"
)

// Print splitting, stage 2: cutting, deciding joints, merging.
//
// The plan is a cut program run on the skin mesh: one cut near each end of
// every branch (validated on the mesh: its section must be one clean loop that
// touches no other cut's section), then — on every joint that can't print
// whole — one split plane. The connected components left are the atoms. Each
// atom stands on the cut face that leaves it the least overhang; neighbouring
// atoms are greedily merged (their cut undone) while the union still prints.
// The same program replays on a finer mesh for export, so the preview and the
// exported parts are the same pieces.

type planCut struct {
	ID      int
	F       cutFrame // N points from side 0 into side 1 (a branch cut: from the joint into the branch)
	Split   bool     // a joint split loop (else a branch cut)
	Branch  int
	Side    int     // branch end; -1 for the single cut of a Mid branch
	Reach   float64 // branch cut: radius around Q holding its section
	Limit   float64 // branch cut: widest its section may be around Q
	At      vec2    // split loop: its centroid and area, to find it again
	Area    float64
	pockets [2][]pocket
}

// splitOp is one joint split: a plane through the atom carrying cap Target.
type splitOp struct {
	F      cutFrame
	Target int32
	Cuts   []int
}

type planTri struct {
	C, N  Vec3
	Area  float64
	Tag   int32
	Joint bool // judged against the joint overhang limit
}

type splitAtom struct {
	Kind   string // "strut" | "joint" | "half"
	Thin   bool
	Tags   []int32
	tris   []planTri
	R      float64
	center Vec3
	area   float64
}

// contact is where two atoms meet on a cut.
type contact struct {
	Cut  int
	A, B int  // atoms on side 0 and side 1
	Part bool // a side of the cut is shared out among several atoms
	Area float64
	Thin bool
}

// stubZone is where a support meets a tube: the stub left on the tube and
// the support's own flare, both held to the joint overhang limit. It is a
// capsule along the support's axis.
type stubZone struct {
	A, B Vec3
	R    float64
}

func (z stubZone) has(p Vec3) bool { return pointSegDist(p, z.A, z.B) < z.R }

// SplitPlan is a fully planned split: the cut program, the pieces it leaves,
// and every glued joint between two pieces with its guide rods and number.
type SplitPlan struct {
	Settings  Settings
	Print     PrintParams
	MMPerUnit float64
	Pieces    []*SplitPiece
	Cuts      []*SplitCut

	g        *splitGraph
	skin     *Field
	box      AABB
	vox      float64
	cuts     []*planCut
	ops      []*splitOp
	atoms    []*splitAtom
	atomOf   []int32 // work mesh triangle → atom
	contacts []contact
	pieceOf  []int // atom → piece
	merged   map[int]bool
	stubs    []stubZone
	sections map[int][][]vec2 // kept cut → its full section, for socket checks
}

// SplitPiece is one printed part.
type SplitPiece struct {
	ID       int
	Atoms    []int
	BedQ     Vec3 // print face: a point on it and its outward normal
	BedN     Vec3
	Overhang float64 // residual overhanging area (mm²) in the chosen orientation
	Flagged  bool    // could not be made printable within the limits
	Centroid Vec3
	Tris     []Triangle // world-space mesh at the planning resolution

	tags map[int32]bool // caps of kept cuts on this piece
}

// PlanSplit cuts the skeleton's skin into printable pieces.
func PlanSplit(s Settings, sk *Skeleton, sp SkinParams, pp PrintParams, progress func(float64)) *SplitPlan {
	report := func(f float64) {
		if progress != nil {
			progress(f)
		}
	}
	field, box, _, _ := skinField(s, sk, sp)
	field.Build()
	mm := pp.CubeMM / s.CubeSize
	pl := &SplitPlan{Settings: s, Print: pp, MMPerUnit: mm, skin: field, box: box, vox: pp.PlanVoxelMM / mm}

	reach := 2*sp.BlendK + 0.5*s.StrutR
	if sp.Cap > 0 {
		reach += sp.Cap
	}
	pl.g = buildSplitGraph(s, sk, reach)
	pl.g.placeCuts(field, pp.StrutTilt, pp.JointTilt, pp.MinPieceMM/mm)
	report(0.1)
	base := newSkinMesh(field, box, pl.vox, func(f float64) { report(0.1 + 0.15*f) })
	pl.fitCuts(base)
	report(0.35)

	work := base.clone()
	for _, c := range pl.cuts {
		loops, _ := pl.branchSection(work, c)
		work.cut(c.ID, c.F, loops, [2][]pocket{})
	}
	pl.buildAtoms(work)
	pl.decideJoints(work)
	report(0.6)
	pl.buildAtoms(work)
	pl.buildContacts()
	pl.mergePieces(pp.Merge)
	report(0.7)
	pl.layoutCuts(work, base)
	report(0.8)
	if parts, err := pl.realize(base); err == nil {
		for i, p := range pl.Pieces {
			p.Tris = base.triangles(parts[i])
			p.Centroid = trisCentroid(p.Tris)
		}
	}
	report(1)
	return pl
}

func trisCentroid(tris []Triangle) Vec3 {
	var sum Vec3
	w := 0.0
	for _, t := range tris {
		a := 0.5 * t.B.Sub(t.A).Cross(t.C.Sub(t.A)).Len()
		sum = sum.Add(t.A.Add(t.B).Add(t.C).Mul(a / 3))
		w += a
	}
	if w == 0 {
		return sum
	}
	return sum.Mul(1 / w)
}

// ---- branch cuts ----

// makeCuts lists the branch cuts the graph currently asks for.
func (pl *SplitPlan) makeCuts() {
	g := pl.g
	pl.cuts = nil
	pl.stubs = nil
	add := func(bi, side int, q, n Vec3) {
		b := &g.branches[bi]
		lim := math.Max(b.footR(), 1.5*b.R)*1.25 + pl.vox
		pl.cuts = append(pl.cuts, &planCut{ID: len(pl.cuts), F: newCutFrame(q, n), Branch: bi, Side: side,
			Reach: 1.5*lim + 2*pl.vox, Limit: lim})
	}
	for bi := range g.branches {
		b := &g.branches[bi]
		for s := 0; s < 2; s++ {
			if !b.Thin || b.J[s] < 0 {
				continue
			}
			l := b.Cut[s] + 2*b.R
			switch {
			case b.Absorbed:
				l = b.length()
			case b.Mid:
				l = b.Cut[s] + 0.5*b.keep()
			}
			pl.stubs = append(pl.stubs, stubZone{A: b.P[s], B: b.P[s].Add(b.dir(s).Mul(l)), R: 3*b.R + pl.vox})
		}
		switch {
		case b.Absorbed:
		case b.Mid:
			add(bi, -1, b.midPoint(), b.dir(0))
		default:
			for s := 0; s < 2; s++ {
				if b.J[s] >= 0 {
					add(bi, s, b.cutPoint(s), b.dir(s))
				}
			}
		}
	}
}

// branchSection finds a branch cut's section on m: the loop around the axis
// point (with any holes inside it). ok is false if there is none, or if it is
// wider than the cut's limit (it has run into another tube).
func (pl *SplitPlan) branchSection(m *cutMesh, c *planCut) ([]secLoop, bool) {
	loops := m.sectionLoops(c.F, m.near(c.F.Q, c.Reach))
	best := -1
	for i, l := range loops {
		if l.area > 0 && inRing(vec2{}, l.pts) && (best < 0 || l.area < loops[best].area) {
			best = i
		}
	}
	if best < 0 {
		return nil, false
	}
	group := []secLoop{loops[best]}
	for i, h := range loops {
		if i != best && h.area < 0 && inRing(h.pts[0], loops[best].pts) {
			group = append(group, h)
		}
	}
	for _, p := range loops[best].pts {
		if math.Hypot(p.X, p.Y) > c.Limit {
			return group, false
		}
	}
	// Another tube's axis through the section: it has cut into that tube.
	own := pl.g.branches[c.Branch].Tube
	for ti, t := range pl.g.tubes {
		sa, sb := t.A.Sub(c.F.Q).Dot(c.F.N), t.B.Sub(c.F.Q).Dot(c.F.N)
		if ti == own || (sa > 0) == (sb > 0) {
			continue
		}
		if inRing(c.F.to2(Lerp(t.A, t.B, sa/(sa-sb))), loops[best].pts) {
			return group, false
		}
	}
	return group, true
}

// fitCuts moves branch cuts away from their joints until every section is a
// clean loop and no two sections touch; branches that run out of length get
// a single middle cut or none.
func (pl *SplitPlan) fitCuts(base *cutMesh) {
	g := pl.g
	minLen := pl.Print.MinPieceMM / pl.MMPerUnit
	push := func(c *planCut) {
		b := &g.branches[c.Branch]
		if c.Side < 0 {
			b.Mid, b.Absorbed = false, true
			if b.J[0] >= 0 && b.J[1] >= 0 {
				g.union(b.J[0], b.J[1])
			}
			return
		}
		b.Cut[c.Side] += 0.15 * b.R
		g.shorten(c.Branch, minLen)
	}
	for iter := 0; iter < 100; iter++ {
		pl.makeCuts()
		secs := make([][]vec2, len(pl.cuts))
		valid := make([]bool, len(pl.cuts))
		var wg sync.WaitGroup
		for i, c := range pl.cuts {
			wg.Add(1)
			go func(i int, c *planCut) {
				defer wg.Done()
				loops, ok := pl.branchSection(base, c)
				if loops != nil {
					secs[i] = loops[0].pts
				}
				valid[i] = ok
			}(i, c)
		}
		wg.Wait()
		moved := map[int]bool{}
		for i, c := range pl.cuts {
			if !valid[i] {
				moved[i] = true
				push(c)
			}
		}
		if len(moved) == 0 {
			for i, a := range pl.cuts {
				for j := i + 1; j < len(pl.cuts); j++ {
					b := pl.cuts[j]
					if a.F.Q.DistTo(b.F.Q) > a.Limit+b.Limit || moved[i] && moved[j] {
						continue
					}
					if sectionsOverlap(a.F, secs[i], b.F, secs[j], pl.vox) {
						for _, k := range []int{i, j} {
							if !moved[k] {
								moved[k] = true
								push(pl.cuts[k])
							}
						}
					}
				}
			}
		}
		if len(moved) == 0 {
			// Every branch piece must come off on its own: one whose skin
			// still runs into a neighbour's (a support leaving a tube at a
			// shallow angle) has the cuts nearest that neighbour pushed out.
			for _, pair := range pl.fused(base) {
				for _, ci := range pair {
					if !moved[ci] {
						moved[ci] = true
						push(pl.cuts[ci])
					}
				}
			}
		}
		if len(moved) == 0 {
			break
		}
	}
	pl.makeCuts()
}

// fused cuts every branch on a copy of base and returns, for each branch
// piece left connected to another branch's piece, the pair of their cuts
// nearest each other.
func (pl *SplitPlan) fused(base *cutMesh) [][2]int {
	m := base.clone()
	for _, c := range pl.cuts {
		if loops, _ := pl.branchSection(m, c); loops != nil {
			m.cut(c.ID, c.F, loops, [2][]pocket{})
		}
	}
	comp, _ := m.components()
	owner := map[int32]map[int]bool{} // component → branches whose piece side is in it
	for t, tg := range m.Tag {
		if tg < 0 || tg&1 == 0 || m.T[t][0] < 0 {
			continue
		}
		c := pl.cuts[tg>>1]
		if c.Side < 0 {
			continue // a Mid cut has no piece of its own
		}
		if owner[comp[t]] == nil {
			owner[comp[t]] = map[int]bool{}
		}
		owner[comp[t]][c.Branch] = true
	}
	var out [][2]int
	for _, bs := range owner {
		if len(bs) < 2 {
			continue
		}
		var ids []int
		for b := range bs {
			ids = append(ids, b)
		}
		sort.Ints(ids)
		for i := 0; i < len(ids); i++ {
			for j := i + 1; j < len(ids); j++ {
				best, bd := [2]int{-1, -1}, math.MaxFloat64
				for _, ca := range pl.cuts {
					if ca.Branch != ids[i] || ca.Split {
						continue
					}
					for _, cb := range pl.cuts {
						if cb.Branch != ids[j] || cb.Split {
							continue
						}
						if d := ca.F.Q.DistTo(cb.F.Q); d < bd {
							best, bd = [2]int{ca.ID, cb.ID}, d
						}
					}
				}
				if best[0] >= 0 {
					out = append(out, best)
				}
			}
		}
	}
	return out
}

// sectionsOverlap reports whether two planar sections share a point (within
// gap): two plane regions can only meet along the planes' common line.
func sectionsOverlap(fa cutFrame, ra []vec2, fb cutFrame, rb []vec2, gap float64) bool {
	if len(ra) == 0 || len(rb) == 0 {
		return false
	}
	d := fa.N.Cross(fb.N)
	l2 := d.Len2()
	if l2 < 1e-8 {
		return math.Abs(fb.Q.Sub(fa.Q).Dot(fa.N)) < gap && inRing(fa.to2(fb.to3(rb[0], 0)), ra)
	}
	da, db := fa.N.Dot(fa.Q), fb.N.Dot(fb.Q)
	p0 := fb.N.Mul(da).Sub(fa.N.Mul(db)).Cross(d).Mul(1 / l2)
	dir := d.Mul(1 / math.Sqrt(l2))
	ia := lineIntervals(fa, ra, p0, dir)
	ib := lineIntervals(fb, rb, p0, dir)
	for _, x := range ia {
		for _, y := range ib {
			if x[0] < y[1]+gap && y[0] < x[1]+gap {
				return true
			}
		}
	}
	return false
}

// lineIntervals returns the parameter intervals of the line p0 + t·dir (lying
// in f's plane) inside ring.
func lineIntervals(f cutFrame, ring []vec2, p0, dir Vec3) [][2]float64 {
	o := f.to2(p0)
	v := vec2{dir.Dot(f.U), dir.Dot(f.V)}
	var ts []float64
	for i := range ring {
		a, b := ring[i], ring[(i+1)%len(ring)]
		e := b.sub(a)
		den := cross2(v, e)
		if math.Abs(den) < 1e-15 {
			continue
		}
		w := a.sub(o)
		t := cross2(w, e) / den
		s := cross2(w, v) / den
		if s >= 0 && s < 1 {
			ts = append(ts, t)
		}
	}
	sort.Float64s(ts)
	var out [][2]float64
	for i := 0; i+1 < len(ts); i += 2 {
		out = append(out, [2]float64{ts[i], ts[i+1]})
	}
	return out
}

// ---- atoms ----

// buildAtoms makes every connected component of the work mesh an atom.
func (pl *SplitPlan) buildAtoms(m *cutMesh) {
	comp, n := m.components()
	pl.atomOf = comp
	atoms := make([]*splitAtom, n)
	tagSet := make([]map[int32]bool, n)
	for i := range atoms {
		atoms[i] = &splitAtom{}
		tagSet[i] = map[int32]bool{}
	}
	for t, tr := range m.T {
		if tr[0] < 0 {
			continue
		}
		a, b, c := m.P[tr[0]], m.P[tr[1]], m.P[tr[2]]
		cr := b.Sub(a).Cross(c.Sub(a))
		ar := 0.5 * cr.Len()
		if ar == 0 {
			continue
		}
		at := atoms[comp[t]]
		pt := planTri{C: a.Add(b).Add(c).Mul(1.0 / 3), N: cr.Normalize(), Area: ar, Tag: m.Tag[t]}
		at.tris = append(at.tris, pt)
		at.center = at.center.Add(pt.C.Mul(ar))
		at.area += ar
		if pt.Tag >= 0 {
			tagSet[comp[t]][pt.Tag] = true
		}
	}
	for i, at := range atoms {
		if at.area > 0 {
			at.center = at.center.Mul(1 / at.area)
		}
		for tg := range tagSet[i] {
			at.Tags = append(at.Tags, tg)
		}
		sort.Slice(at.Tags, func(x, y int) bool { return at.Tags[x] < at.Tags[y] })
		joint, half, thin, own := false, false, true, false
		for _, tg := range at.Tags {
			c := pl.cuts[tg>>1]
			if c.Split {
				half = true
				continue
			}
			b := &pl.g.branches[c.Branch]
			at.R = math.Max(at.R, b.R)
			switch {
			case tg&1 == 1:
				own = true
				thin = thin && b.Thin
			case c.Side >= 0 && pl.g.isVertex(b.J[c.Side]):
				joint = true
			}
		}
		switch {
		case half:
			at.Kind = "half"
		case joint:
			at.Kind = "joint"
		default:
			at.Kind = "strut"
			at.Thin = own && thin
		}
		for k := range at.tris {
			t := &at.tris[k]
			t.Joint = at.Kind != "strut"
			for _, z := range pl.stubs {
				if !t.Joint && z.has(t.C) {
					t.Joint = true
				}
			}
		}
	}
	pl.atoms = atoms
}

// bedPick is a print orientation: standing on the plane (q, n), n outward.
type bedPick struct {
	q, n Vec3
	cost float64
	ok   bool
}

// capPlane is the plane of cap tag tg with its outward normal.
func (pl *SplitPlan) capPlane(tg int32) (Vec3, Vec3) {
	f := pl.cuts[tg>>1].F
	if tg&1 == 0 {
		return f.Q, f.N
	}
	return f.Q, f.N.Neg()
}

// cost is the overhanging area of the atoms in set standing on (q, n), with
// the caps in internal (faces between atoms of the set) left out. ok is false
// if any of it would sit below the bed.
func (pl *SplitPlan) cost(set []int, internal map[int32]bool, q, n Vec3) (float64, bool) {
	up := n.Neg()
	sinS := math.Sin(pl.Print.StrutTilt*math.Pi/180) + 0.03
	sinJ := math.Sin(pl.Print.JointTilt*math.Pi/180) + 0.03
	c := 0.0
	for _, ai := range set {
		for k := range pl.atoms[ai].tris {
			t := &pl.atoms[ai].tris[k]
			if t.Tag >= 0 && internal[t.Tag] {
				continue
			}
			h := t.C.Sub(q).Dot(up)
			if h < -1.5*pl.vox {
				return 0, false
			}
			if h < 1.5*pl.vox {
				continue // on or just above the bed: first layers, not an overhang
			}
			lim := sinS
			if t.Joint {
				lim = sinJ
			}
			if -t.N.Dot(up) > lim {
				c += t.Area
			}
		}
	}
	return c, true
}

// tol is the overhang area a piece may keep: meshing noise at face rims.
func (pl *SplitPlan) tol(set []int) float64 {
	a := 0.0
	for _, ai := range set {
		a += pl.atoms[ai].area
	}
	return pl.tolArea(a)
}

func (pl *SplitPlan) tolArea(a float64) float64 {
	return math.Max(6/(pl.MMPerUnit*pl.MMPerUnit), 0.006*a)
}

// internalCaps lists the caps between atoms of set.
func (pl *SplitPlan) internalCaps(set []int) map[int32]bool {
	in := map[int]bool{}
	for _, a := range set {
		in[a] = true
	}
	out := map[int32]bool{}
	for _, c := range pl.contacts {
		if !c.Part && in[c.A] && in[c.B] {
			out[capTag(c.Cut, 0)] = true
			out[capTag(c.Cut, 1)] = true
		}
	}
	return out
}

// bestBed tries every external cap of the set as its print face.
func (pl *SplitPlan) bestBed(set []int) bedPick {
	internal := pl.internalCaps(set)
	best := bedPick{cost: math.MaxFloat64}
	seen := map[int32]bool{}
	for _, ai := range set {
		for _, tg := range pl.atoms[ai].Tags {
			if internal[tg] || seen[tg] {
				continue
			}
			seen[tg] = true
			q, n := pl.capPlane(tg)
			if c, ok := pl.cost(set, internal, q, n); ok && c < best.cost {
				best = bedPick{q: q, n: n, cost: c, ok: true}
			}
		}
	}
	return best
}

// ---- joints ----

type splitCand struct {
	n     Vec3
	d     float64
	cost  float64 // overhang of both halves, plus penalties
	ok    bool    // both halves print standing on the cut
	joins int     // neighbours a half can take in without overhanging
}

// better ranks split candidates: printable halves first, then those that let
// more neighbours join a half (fewer pieces), then the cheapest.
func (a splitCand) better(b splitCand) bool {
	if a.ok != b.ok {
		return a.ok
	}
	if a.ok && a.joins != b.joins {
		return a.joins > b.joins
	}
	return a.cost < b.cost
}

// decideJoints keeps each joint whole when it prints on one of its faces, and
// otherwise cuts it with the best plane that leaves two printable halves.
func (pl *SplitPlan) decideJoints(m *cutMesh) {
	cands := make([][]splitCand, len(pl.atoms))
	owner := map[int32]int{}
	for ai, at := range pl.atoms {
		for _, tg := range at.Tags {
			if _, dup := owner[tg]; dup {
				owner[tg] = -1
			} else {
				owner[tg] = ai
			}
		}
	}
	var wg sync.WaitGroup
	for ai, at := range pl.atoms {
		if at.Kind != "joint" {
			continue
		}
		wg.Add(1)
		go func(ai int) {
			defer wg.Done()
			whole := pl.bestBed([]int{ai})
			if whole.ok && whole.cost <= pl.tol([]int{ai}) {
				return
			}
			cs := pl.searchSplit(ai, owner)
			if len(cs) > 0 && (!whole.ok || cs[0].cost < whole.cost) {
				cands[ai] = cs
			}
		}(ai)
	}
	wg.Wait()

	minArea := 8 / (pl.MMPerUnit * pl.MMPerUnit)
	nTris := len(m.T)
	for ai, cs := range cands {
		if len(cs) == 0 {
			continue
		}
		var mine []int32
		for t := 0; t < nTris; t++ {
			if pl.atomOf[t] == int32(ai) && m.T[t][0] >= 0 {
				mine = append(mine, int32(t))
			}
		}
		for _, c := range cs {
			f := newCutFrame(c.n.Mul(c.d), c.n)
			loops := m.sectionLoops(f, mine)
			groups := loopGroups(loops)
			ok := len(groups) > 0
			for _, g := range groups {
				if loops[g[0]].area < minArea {
					ok = false
				}
			}
			if !ok {
				continue
			}
			op := &splitOp{F: f, Target: pl.atoms[ai].Tags[0]}
			for _, g := range groups {
				id := len(pl.cuts)
				var gl []secLoop
				for _, i := range g {
					gl = append(gl, loops[i])
				}
				pl.cuts = append(pl.cuts, &planCut{ID: id, F: f, Split: true, Branch: -1, Side: -1,
					At: ringCentroid(gl[0].pts), Area: gl[0].area})
				op.Cuts = append(op.Cuts, id)
				m.cut(id, f, gl, [2][]pocket{})
			}
			pl.ops = append(pl.ops, op)
			break
		}
	}
}

func ringCentroid(r []vec2) vec2 {
	var cx, cy, a float64
	for i := range r {
		p, q := r[i], r[(i+1)%len(r)]
		w := p.X*q.Y - q.X*p.Y
		cx += (p.X + q.X) * w
		cy += (p.Y + q.Y) * w
		a += w
	}
	if a == 0 {
		return r[0]
	}
	return vec2{cx / (3 * a), cy / (3 * a)}
}

// searchSplit samples plane normals over a hemisphere (plus the axes of the
// struts around the joint) and offsets through the joint, scoring the overhang
// of both halves (each standing on the cut), and refines around the best.
// owner maps a cap tag to the one atom carrying it. Returns candidates, best
// first.
func (pl *SplitPlan) searchSplit(ai int, owner map[int32]int) []splitCand {
	a := pl.atoms[ai]
	c, R := a.center, math.Max(a.R, 2*pl.vox)
	tagIdx := map[int32]int{}
	for i, tg := range a.Tags {
		tagIdx[tg] = i
	}
	nf := len(a.Tags)
	// The atom across each face, and its cut's normal: cutting square to a
	// strut lets the half stand on the cut with the strut rising from it.
	nbr := make([]int, nf)
	var axes []Vec3
	for i, tg := range a.Tags {
		nbr[i] = -1
		if nb, ok := owner[tg^1]; ok && nb >= 0 {
			nbr[i] = nb
			axes = append(axes, pl.cuts[tg>>1].F.N)
		}
	}
	sinJ := math.Sin(pl.Print.JointTilt*math.Pi/180) + 0.03
	eval := func(n Vec3, d float64) splitCand {
		faceP := make([]float64, nf)
		faceN := make([]float64, nf)
		ovP := make([]float64, nf)
		ovN := make([]float64, nf)
		var costP, costN, areaP, areaN float64
		for i := range a.tris {
			t := &a.tris[i]
			h := t.C.Dot(n) - d
			nu := t.N.Dot(n)
			fi := -1
			if t.Tag >= 0 {
				fi = tagIdx[t.Tag]
			}
			if h > 0 {
				areaP += t.Area
				if fi >= 0 {
					faceP[fi] += t.Area
				}
				if h >= 1.5*pl.vox && -nu > sinJ {
					costP += t.Area
					if fi >= 0 {
						ovP[fi] += t.Area
					}
				}
			} else {
				areaN += t.Area
				if fi >= 0 {
					faceN[fi] += t.Area
				}
				if -h >= 1.5*pl.vox && nu > sinJ {
					costN += t.Area
					if fi >= 0 {
						ovN[fi] += t.Area
					}
				}
			}
		}
		if areaP < 0.12*a.area || areaN < 0.12*a.area {
			return splitCand{cost: math.MaxFloat64}
		}
		cost := costP + costN
		// Slicing through a cut face splits its contact in two: weaker, and
		// each part too small for a rod. Penalise the smaller part.
		for i := range faceP {
			cost += math.Min(faceP[i], faceN[i])
		}
		cost += 0.02 * a.area * math.Abs(d-c.Dot(n)) / R
		sc := splitCand{n: n, d: d, cost: cost}
		sc.ok = costP <= pl.tolArea(areaP) && costN <= pl.tolArea(areaN)
		if !sc.ok {
			return sc
		}
		// Count the neighbours that could join the half they hang off: the
		// fewer of them left on their own, the fewer pieces.
		for i, nb := range nbr {
			switch {
			case nb < 0:
			case faceN[i] == 0 && pl.canJoin(nb, a.Tags[i]^1, n, d, costP-ovP[i], areaP):
				sc.joins++
			case faceP[i] == 0 && pl.canJoin(nb, a.Tags[i]^1, n.Neg(), -d, costN-ovN[i], areaN):
				sc.joins++
			}
		}
		return sc
	}
	var all []splitCand
	for _, n := range append(fibHemisphere(500), axes...) {
		for _, o := range []float64{-0.6, -0.3, 0, 0.3, 0.6} {
			if sc := eval(n, c.Dot(n)+o*R); sc.cost < math.MaxFloat64 {
				all = append(all, sc)
			}
		}
	}
	sort.Slice(all, func(i, j int) bool { return all[i].better(all[j]) })
	if len(all) > 40 {
		all = all[:40]
	}
	// Local refinement of the best few: small tilts and shifts.
	for k := 0; k < len(all) && k < 4; k++ {
		best := all[k]
		for _, scale := range []float64{0.12, 0.05, 0.02} {
			ref := V(1, 0, 0)
			if math.Abs(best.n[0]) > 0.8 {
				ref = V(0, 1, 0)
			}
			e1 := best.n.Cross(ref).Normalize()
			e2 := best.n.Cross(e1)
			n0, d0 := best.n, best.d
			for i := -3; i <= 3; i++ {
				for j := -3; j <= 3; j++ {
					n := n0.Add(e1.Mul(scale * float64(i) / 3)).Add(e2.Mul(scale * float64(j) / 3)).Normalize()
					for _, o := range []float64{-0.1, 0, 0.1} {
						d := d0 + n.Sub(n0).Dot(c) + o*scale*R
						if sc := eval(n, d); sc.better(best) {
							best = sc
						}
					}
				}
			}
		}
		all[k] = best
	}
	sort.SliceStable(all, func(i, j int) bool { return all[i].better(all[j]) })
	return all
}

// canJoin reports whether atom nb, attached to a joint half by its face tag,
// still prints once joined to that half standing on the plane C·up = d. The
// half itself overhangs by base over area.
func (pl *SplitPlan) canJoin(nb int, tag int32, up Vec3, d, base, area float64) bool {
	c, ok := pl.cost([]int{nb}, map[int32]bool{tag: true}, up.Mul(d), up.Neg())
	return ok && base+c <= pl.tolArea(area+pl.atoms[nb].area)
}

// fibHemisphere returns n roughly uniform unit vectors on the upper hemisphere
// (a plane and its flip are the same split).
func fibHemisphere(n int) []Vec3 {
	out := make([]Vec3, 0, n)
	ga := math.Pi * (3 - math.Sqrt(5))
	for i := 0; i < n; i++ {
		z := 1 - (float64(i)+0.5)/float64(n)
		r := math.Sqrt(1 - z*z)
		th := ga * float64(i)
		out = append(out, V(r*math.Cos(th), r*math.Sin(th), z))
	}
	return out
}

// ---- pieces ----

// buildContacts lists, for every cut, which atoms meet across it.
func (pl *SplitPlan) buildContacts() {
	byTag := map[int32][]int{}
	area := map[[2]int64]float64{}
	for ai, at := range pl.atoms {
		for _, tg := range at.Tags {
			byTag[tg] = append(byTag[tg], ai)
		}
		for _, t := range at.tris {
			if t.Tag >= 0 {
				area[[2]int64{int64(t.Tag), int64(ai)}] += t.Area
			}
		}
	}
	pl.contacts = nil
	for _, c := range pl.cuts {
		s0, s1 := byTag[capTag(c.ID, 0)], byTag[capTag(c.ID, 1)]
		part := len(s0) > 1 || len(s1) > 1
		thin := !c.Split && pl.g.branches[c.Branch].Thin
		for _, x := range s0 {
			for _, y := range s1 {
				a := math.Min(area[[2]int64{int64(capTag(c.ID, 0)), int64(x)}], area[[2]int64{int64(capTag(c.ID, 1)), int64(y)}])
				pl.contacts = append(pl.contacts, contact{Cut: c.ID, A: x, B: y, Part: part, Area: a, Thin: thin})
			}
		}
	}
}

// mergePieces greedily joins neighbouring atoms when the union still prints:
// across thin cuts first (the weakest to glue), then smaller faces first.
func (pl *SplitPlan) mergePieces(merge bool) {
	n := len(pl.atoms)
	group := make([]int, n)
	members := make([][]int, n)
	beds := make([]bedPick, n)
	for i := range group {
		group[i] = i
		members[i] = []int{i}
		beds[i] = pl.bestBed(members[i])
	}
	bad := func(g int) bool { return !beds[g].ok || beds[g].cost > pl.tol(members[g]) }
	type cand struct {
		bed   bedPick
		valid bool
	}
	cache := map[[2]int]cand{}
	for merge {
		bi := -1
		var bestBed bedPick
		for ci, c := range pl.contacts {
			ga, gb := group[c.A], group[c.B]
			if c.Part || ga == gb || bad(ga) || bad(gb) {
				continue
			}
			key := [2]int{min(ga, gb), max(ga, gb)}
			cd, ok := cache[key]
			if !ok {
				set := append(append([]int{}, members[ga]...), members[gb]...)
				b := pl.bestBed(set)
				cd = cand{b, b.ok && b.cost <= pl.tol(set)}
				cache[key] = cd
			}
			if !cd.valid {
				continue
			}
			if bi < 0 || c.Thin && !pl.contacts[bi].Thin || c.Thin == pl.contacts[bi].Thin && c.Area < pl.contacts[bi].Area {
				bi, bestBed = ci, cd.bed
			}
		}
		if bi < 0 {
			break
		}
		ga, gb := group[pl.contacts[bi].A], group[pl.contacts[bi].B]
		for _, a := range members[gb] {
			group[a] = ga
		}
		members[ga] = append(members[ga], members[gb]...)
		members[gb] = nil
		beds[ga] = bestBed
		for k := range cache {
			if k[0] == ga || k[1] == ga || k[0] == gb || k[1] == gb {
				delete(cache, k)
			}
		}
	}

	pl.pieceOf = make([]int, n)
	pl.Pieces = nil
	idx := map[int]int{}
	for a := 0; a < n; a++ {
		g := group[a]
		pi, ok := idx[g]
		if !ok {
			pi = len(pl.Pieces)
			idx[g] = pi
			p := &SplitPiece{ID: pi + 1, Atoms: append([]int{}, members[g]...), tags: map[int32]bool{}}
			sort.Ints(p.Atoms)
			b := beds[g]
			p.BedQ, p.BedN = b.q, b.n
			p.Overhang = b.cost * pl.MMPerUnit * pl.MMPerUnit
			p.Flagged = bad(g)
			if !b.ok {
				p.Overhang = 0
			}
			pl.Pieces = append(pl.Pieces, p)
		}
		pl.pieceOf[a] = pi
	}
	// A cut is undone when every contact across it is inside one piece.
	pl.merged = map[int]bool{}
	open := map[int]bool{}
	for _, c := range pl.contacts {
		if pl.pieceOf[c.A] == pl.pieceOf[c.B] && !c.Part {
			pl.merged[c.Cut] = true
		} else {
			open[c.Cut] = true
		}
	}
	for id := range open {
		delete(pl.merged, id)
	}
	for ai, at := range pl.atoms {
		for _, tg := range at.Tags {
			if !pl.merged[int(tg>>1)] {
				pl.Pieces[pl.pieceOf[ai]].tags[tg] = true
			}
		}
	}
}

// ---- replay ----

// realize runs the plan's cut program on a fresh skin mesh m (any resolution):
// branch cuts and joint splits with their pockets, then re-joins the merged
// cuts. It returns each piece's triangles.
func (pl *SplitPlan) realize(m *cutMesh) ([][]int32, error) {
	twins, extra, err := pl.replay(m)
	if err != nil {
		return nil, err
	}
	m.uncut(pl.merged, twins)

	for iter := 0; ; iter++ {
		comp, n := m.components()
		tags := make([]map[int32]bool, n)
		for i := range tags {
			tags[i] = map[int32]bool{}
		}
		for t, tg := range m.Tag {
			if tg >= 0 && m.T[t][0] >= 0 {
				tags[comp[t]][tg] = true
			}
		}
		// Match shells to pieces by their cut faces: shared faces count for,
		// a piece's faces the shell lacks against (a finer mesh can let a
		// split plane graze one more face), best pairs first, one to one.
		type match struct {
			c, p  int
			score int
		}
		var ms []match
		for ci := range tags {
			for pi, p := range pl.Pieces {
				k := 0
				for tg := range tags[ci] {
					if p.tags[tg] {
						k++
					}
				}
				if k > 0 {
					ms = append(ms, match{ci, pi, 2*k - (len(p.tags) - k)})
				}
			}
		}
		sort.SliceStable(ms, func(i, j int) bool { return ms[i].score > ms[j].score })
		owner := make([]int, n)
		for ci := range owner {
			owner[ci] = -1
		}
		taken := map[int]bool{}
		for pass := 0; pass < 2; pass++ {
			for _, x := range ms {
				if owner[x.c] < 0 && (pass == 1 || !taken[x.p]) {
					owner[x.c] = x.p
					taken[x.p] = true
				}
			}
		}
		orphans := map[int]bool{}
		for ci := range tags {
			if owner[ci] < 0 {
				for tg := range tags[ci] {
					if extra[int(tg>>1)] {
						orphans[int(tg>>1)] = true
					}
				}
			}
		}
		// A sliver only this resolution produced: glue it back on.
		if len(orphans) > 0 && iter < 3 {
			m.uncut(orphans, twins)
			for id := range orphans {
				delete(extra, id)
			}
			continue
		}
		if len(pl.Pieces) == 1 {
			for ci := range owner {
				owner[ci] = 0
			}
		}
		parts := make([][]int32, len(pl.Pieces))
		for t, c := range comp {
			if c >= 0 && owner[c] >= 0 {
				parts[owner[c]] = append(parts[owner[c]], int32(t))
			}
		}
		return parts, nil
	}
}

// replay runs every cut of the program on m — branch cuts, then joint splits,
// each with its pockets — finding each split's loops again by position. It
// returns every cut's loop vertex pairs and the ids it gave loops the plan
// never had (slivers of a finer mesh).
func (pl *SplitPlan) replay(m *cutMesh) (map[int][][2]int32, map[int]bool, error) {
	twins := map[int][][2]int32{}
	for _, c := range pl.cuts {
		if c.Split {
			continue
		}
		loops, _ := pl.branchSection(m, c)
		if loops == nil {
			return nil, nil, fmt.Errorf("print split: cut %d has no section at this resolution", c.ID)
		}
		twins[c.ID] = m.cut(c.ID, c.F, loops, c.pockets)
	}
	comp, _ := m.components()
	next := len(pl.cuts)
	extra := map[int]bool{}
	for _, op := range pl.ops {
		target := int32(-1)
		for t, tg := range m.Tag[:len(comp)] {
			if tg == op.Target && m.T[t][0] >= 0 {
				target = comp[t]
				break
			}
		}
		if target < 0 {
			return nil, nil, fmt.Errorf("print split: joint to split not found")
		}
		var mine []int32
		for t, c := range comp {
			if c == target && m.T[t][0] >= 0 {
				mine = append(mine, int32(t))
			}
		}
		loops := m.sectionLoops(op.F, mine)
		used := map[int]bool{}
		for _, g := range loopGroups(loops) {
			outer := loops[g[0]]
			at := ringCentroid(outer.pts)
			id, bd := -1, math.MaxFloat64
			for _, cid := range op.Cuts {
				c := pl.cuts[cid]
				d := at.dist(c.At)
				r := outer.area / c.Area
				if !used[cid] && d < bd && d < math.Max(3*pl.vox, 0.3*math.Sqrt(c.Area)) && r > 0.5 && r < 2 {
					id, bd = cid, d
				}
			}
			var pk [2][]pocket
			if id >= 0 {
				used[id] = true
				pk = pl.cuts[id].pockets
			} else {
				id = next
				next++
				extra[id] = true
			}
			var gl []secLoop
			for _, i := range g {
				gl = append(gl, loops[i])
			}
			twins[id] = m.cut(id, op.F, gl, pk)
		}
	}
	return twins, extra, nil
}
