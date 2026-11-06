package engine

import (
	"math"
	"sort"
	"sync"
)

// Print splitting, stage 1: the skeleton as a graph of tube branches meeting at
// junctions, and where along each branch to cut.
//
// Thick tubes (the 12 cube edges and the struts) are only cut at vertices —
// junctions where thick tubes meet. A thin support landing on a thick tube
// mid-span does not cut it: the support is cut off close to the tube's surface
// and its short stub stays on the tube. Supports themselves are cut near both
// ends. Each cut's distance along its branch comes from probing the skin's
// radius around the axis: the branch piece must print standing on one of its
// cuts with no surface overhanging more than StrutTilt, so the cut sits where
// the joint's flare has died down (deeper at the end the piece stands on, where
// a flare is just a wider foot).

// PrintParams configures splitting the sculpture into printable pieces. Fields
// ending in MM are physical millimetres; CubeMM fixes the mm-per-unit scale.
type PrintParams struct {
	CubeMM         float64 // physical cube edge length
	StrutTilt      float64 // max overhang on strut pieces, degrees from vertical
	JointTilt      float64 // max overhang on joint pieces, degrees from vertical
	ClearanceMM    float64 // gap left on every side between a rod or key and its socket
	PlanVoxelMM    float64 // grid cell of the planning / preview meshes
	VoxelMM        float64 // grid cell of the exported piece meshes
	MinPieceMM     float64 // shortest standalone strut piece; shorter ones join the joint
	Merge          bool    // re-merge neighbouring pieces whenever the result still prints
	Engrave        bool    // engrave each cut's number on both of its faces
	EngraveDepthMM float64
	MainRod        RodSpec // guide rod for thick cuts
	ThinRod        RodSpec // smaller guide rod for cuts too small for MainRod
}

// RodSpec is a guide rod: a trapezoid cross-section (flat base, sloped sides)
// tapering toward both ends, half in each part. It prints lying on its base.
type RodSpec struct {
	BaseMM   float64 `json:"baseMM"`   // trapezoid base width at the middle
	TopMM    float64 `json:"topMM"`    // trapezoid top width at the middle
	HeightMM float64 `json:"heightMM"` // trapezoid height at the middle
	LengthMM float64 `json:"lengthMM"` // total length (half goes into each part)
	Taper    float64 `json:"taper"`    // end cross-section scale relative to the middle
	WallMM   float64 `json:"wallMM"`   // minimum material left around the socket
}

func DefaultPrintParams() PrintParams {
	return PrintParams{
		CubeMM:         150,
		StrutTilt:      15,
		JointTilt:      35,
		ClearanceMM:    0.1,
		PlanVoxelMM:    1.0,
		VoxelMM:        0.4,
		MinPieceMM:     4,
		Merge:          true,
		Engrave:        true,
		EngraveDepthMM: 0.6,
		MainRod:        RodSpec{BaseMM: 4, TopMM: 2, HeightMM: 2.6, LengthMM: 8, Taper: 0.7, WallMM: 1.6},
		ThinRod:        RodSpec{BaseMM: 2.86, TopMM: 1.43, HeightMM: 1.82, LengthMM: 6, Taper: 0.7, WallMM: 0.9},
	}
}

type splitTube struct {
	A, B Vec3
	R    float64
	Thin bool
	Kind string // "edge" | "strut" | "support"
}

func (t splitTube) at(u float64) Vec3 { return Lerp(t.A, t.B, u) }

// splitBranch is one tube section between two junctions.
type splitBranch struct {
	Tube     int
	P        [2]Vec3    // axis points at the two end junctions
	J        [2]int     // junction at each end; -1 = no cut there (free tip, or a thick tube passing on)
	Host     [2]bool    // the end lands mid-span on another tube (its stub stays there)
	Cut      [2]float64 // axis distance from each end to its cut plane
	R        float64
	Thin     bool
	Bottom   int  // end the branch piece stands on
	Mid      bool // too short for two cuts: one cut across its middle
	Absorbed bool // too short to cut at all: part of its joint
}

func (b *splitBranch) length() float64 { return b.P[0].DistTo(b.P[1]) }

// dir is the unit axis direction from end `side` toward the other end.
func (b *splitBranch) dir(side int) Vec3 { return b.P[1-side].Sub(b.P[side]).Normalize() }

func (b *splitBranch) cutPoint(side int) Vec3 { return b.P[side].Add(b.dir(side).Mul(b.Cut[side])) }

// keep is the axis length left between the branch's two cuts.
func (b *splitBranch) keep() float64 {
	k := b.length()
	for s := 0; s < 2; s++ {
		if b.J[s] >= 0 {
			k -= b.Cut[s]
		}
	}
	return k
}

// splitJunction is a point where tubes meet. Junctions whose skins fuse are
// merged (union-find on Parent); the root stands for the whole cluster.
type splitJunction struct {
	Center Vec3
	Parent int
}

type splitGraph struct {
	tubes     []splitTube
	branches  []splitBranch
	junctions []splitJunction
	vertex    map[int]bool  // cluster root → thick tubes meet here (a joint piece)
	onTubes   map[int][]int // junction → the tubes it lies on
}

func (g *splitGraph) root(j int) int {
	for g.junctions[j].Parent != j {
		g.junctions[j].Parent = g.junctions[g.junctions[j].Parent].Parent
		j = g.junctions[j].Parent
	}
	return j
}

func (g *splitGraph) union(a, b int) {
	ra, rb := g.root(a), g.root(b)
	if ra == rb {
		return
	}
	if rb < ra {
		ra, rb = rb, ra
	}
	g.junctions[rb].Parent = ra
	if g.vertex != nil && g.vertex[rb] {
		g.vertex[ra] = true
	}
}

// isVertex reports whether junction j's cluster is a joint between thick tubes.
func (g *splitGraph) isVertex(j int) bool { return j >= 0 && g.vertex[g.root(j)] }

// buildSplitGraph turns the frame + skeleton into tubes, junctions and branches.
// reach is how close two tubes may pass before their skins fuse (they then get
// a shared crossing junction).
func buildSplitGraph(s Settings, sk *Skeleton, reach float64) *splitGraph {
	g := &splitGraph{}
	half := s.CubeSize / 2
	topo := BuildCubeTopology(half)
	for _, e := range topo.Edges {
		g.tubes = append(g.tubes, splitTube{A: e.A, B: e.B, R: s.StrutR, Kind: "edge"})
	}
	for _, st := range sk.Struts {
		g.tubes = append(g.tubes, splitTube{A: sk.EndA(st), B: sk.EndB(st), R: st.EffRadius(s), Thin: st.Kind == StrutThin, Kind: "strut"})
	}
	for _, ss := range sk.Supports {
		g.tubes = append(g.tubes, splitTube{A: sk.EndP1(ss), B: sk.EndP2(ss), R: s.ButtressR, Thin: true, Kind: "support"})
	}

	// Junction points: cube corners and every tube endpoint.
	addJunction := func(p Vec3) int {
		for i, j := range g.junctions {
			if j.Center.DistTo(p) < 1e-6 {
				return i
			}
		}
		g.junctions = append(g.junctions, splitJunction{Center: p, Parent: len(g.junctions)})
		return len(g.junctions) - 1
	}
	for _, v := range topo.Verts {
		addJunction(v)
	}
	for _, t := range g.tubes {
		addJunction(t.A)
		addJunction(t.B)
	}

	// Attach every junction lying on a tube's axis to that tube.
	type attach struct {
		u float64
		j int
	}
	atts := make([][]attach, len(g.tubes))
	nPoint := len(g.junctions)
	for ti, t := range g.tubes {
		ab := t.B.Sub(t.A)
		l2 := ab.Len2()
		for j := 0; j < nPoint; j++ {
			p := g.junctions[j].Center
			u := clampF(p.Sub(t.A).Dot(ab)/l2, 0, 1)
			if t.at(u).DistTo(p) < 0.3*t.R {
				atts[ti] = append(atts[ti], attach{u, j})
			}
		}
	}

	// Crossings: tubes that share no junction but pass close enough to fuse.
	var unions [][2]int
	sharesJunction := func(a, b int) bool {
		for _, x := range atts[a] {
			for _, y := range atts[b] {
				if x.j == y.j {
					return true
				}
			}
		}
		return false
	}
	for a := range g.tubes {
		for b := a + 1; b < len(g.tubes); b++ {
			ta, tb := g.tubes[a], g.tubes[b]
			if sharesJunction(a, b) {
				continue
			}
			hit := ClosestApproach(ta.A, ta.B, tb.A, tb.B)
			tol := ta.R + tb.R + reach
			if hit.Dist > tol {
				continue
			}
			ua := clampF(hit.POnAB.Sub(ta.A).Dot(ta.B.Sub(ta.A))/ta.B.Sub(ta.A).Len2(), 0, 1)
			ub := clampF(hit.QOnEx.Sub(tb.A).Dot(tb.B.Sub(tb.A))/tb.B.Sub(tb.A).Len2(), 0, 1)
			// A junction already on either tube near the meeting point takes the
			// crossing over (typically a tube ending right beside the other).
			near := func(ti int, p Vec3) int {
				best, bd := -1, tol
				for _, x := range atts[ti] {
					if d := g.tubes[ti].at(x.u).DistTo(p); d < bd {
						best, bd = x.j, d
					}
				}
				return best
			}
			ja, jb := near(a, hit.POnAB), near(b, hit.QOnEx)
			switch {
			case ja >= 0 && jb >= 0:
				unions = append(unions, [2]int{ja, jb})
			case ja >= 0:
				atts[b] = append(atts[b], attach{ub, ja})
			case jb >= 0:
				atts[a] = append(atts[a], attach{ua, jb})
			default:
				j := len(g.junctions)
				g.junctions = append(g.junctions, splitJunction{Center: Lerp(hit.POnAB, hit.QOnEx, 0.5), Parent: j})
				atts[a] = append(atts[a], attach{ua, j})
				atts[b] = append(atts[b], attach{ub, j})
			}
		}
	}
	for _, u := range unions {
		g.union(u[0], u[1])
	}
	g.onTubes = map[int][]int{}
	for ti := range g.tubes {
		for _, a := range atts[ti] {
			g.onTubes[a.j] = append(g.onTubes[a.j], ti)
		}
	}

	// Count the tube arms leaving each cluster (a tube passing through counts
	// twice). Thick tubes meeting — three arms, or two from different tubes —
	// make a vertex; so do supports meeting with no thick tube.
	type arms struct {
		thick, thin int
		thickTubes  map[int]bool
		total       int
	}
	ar := map[int]*arms{}
	for ti, t := range g.tubes {
		for _, a := range atts[ti] {
			r := g.root(a.j)
			x := ar[r]
			if x == nil {
				x = &arms{thickTubes: map[int]bool{}}
				ar[r] = x
			}
			n := 2
			if p := t.at(a.u); p.DistTo(t.A) < t.R || p.DistTo(t.B) < t.R {
				n = 1
			}
			x.total += n
			if t.Thin {
				x.thin += n
			} else {
				x.thick += n
				x.thickTubes[ti] = true
			}
		}
	}
	g.vertex = map[int]bool{}
	for r, x := range ar {
		// A tube passing a single support's foot stays whole; two or more
		// supports landing on one spot (or on a strut's end) make a real node.
		if x.thick >= 3 || x.thick == 2 && len(x.thickTubes) == 2 || x.thin >= 2 && x.thick <= 2 {
			g.vertex[r] = true
		}
	}

	// Branches: thick tubes split only at vertices (and their own ends),
	// supports at every junction on them.
	for ti, t := range g.tubes {
		as := atts[ti]
		sort.Slice(as, func(i, k int) bool { return as[i].u < as[k].u })
		var stops []attach
		for i, a := range as {
			if t.Thin || g.vertex[g.root(a.j)] || i == 0 || i == len(as)-1 {
				stops = append(stops, a)
			}
		}
		for i := 0; i+1 < len(stops); i++ {
			a, b := stops[i], stops[i+1]
			if g.root(a.j) == g.root(b.j) && t.at(a.u).DistTo(t.at(b.u)) < 1e-6 {
				continue
			}
			br := splitBranch{Tube: ti, P: [2]Vec3{t.at(a.u), t.at(b.u)}, J: [2]int{a.j, b.j}, R: t.R, Thin: t.Thin}
			for s, at := range []attach{a, b} {
				r := g.root(at.j)
				switch {
				case g.vertex[r]:
				case !t.Thin || ar[r] == nil || ar[r].total < 2:
					br.J[s] = -1 // a thick tube's end, or a free tip: nothing to cut off
				default:
					br.Host[s] = true
				}
			}
			g.branches = append(g.branches, br)
		}
	}
	return g
}

// branchProfile samples the skin radius around a branch's axis, walking from
// end `side` toward the middle. r[i][a] is the radius at axis distance i·h in
// direction a; clean[i] is false where the ray never left the material within
// rmax or found other material just past the surface (another tube's skin).
type branchProfile struct {
	h     float64
	r     [][]float64
	rmax  []float64
	clean []bool
}

const profileDirs = 12

func probeBranch(f *Field, b *splitBranch, side int) branchProfile {
	d := b.dir(side)
	ref := V(1, 0, 0)
	if math.Abs(d[0]) > 0.8 {
		ref = V(0, 1, 0)
	}
	u1 := d.Cross(ref).Normalize()
	u2 := d.Cross(u1)
	var us [profileDirs]Vec3
	for a := range us {
		th := 2 * math.Pi * float64(a) / profileDirs
		us[a] = u1.Mul(math.Cos(th)).Add(u2.Mul(math.Sin(th)))
	}

	h := 0.12 * b.R
	n := int(b.length() / 2 / h)
	if b.J[1-side] < 0 {
		n = int((b.length() - b.R) / h) // far end is a free tip: walk to it
	}
	rmax := 3*b.R + 0.15
	pr := branchProfile{h: h}
	for i := 0; i <= n; i++ {
		o := b.P[side].Add(d.Mul(float64(i) * h))
		rs := make([]float64, profileDirs)
		ok := f.Eval(o) < 0
		mx := 0.0
		for a, u := range us {
			r := probeRadius(f, o, u, rmax, 0.15*b.R)
			rs[a] = r
			mx = math.Max(mx, r)
			if r >= rmax {
				ok = false
				continue
			}
			// Nothing else within half a radius past the surface.
			for k := 1; k <= 3 && ok; k++ {
				if f.Eval(o.Add(u.Mul(r+0.2*b.R*float64(k)))) < 0 {
					ok = false
				}
			}
		}
		pr.r = append(pr.r, rs)
		pr.rmax = append(pr.rmax, mx)
		pr.clean = append(pr.clean, ok)
	}
	return pr
}

// probeRadius marches from axis point o along u to the first exit from the
// material and bisects it; rmax if it never exits.
func probeRadius(f *Field, o, u Vec3, rmax, step float64) float64 {
	prev := 0.0
	for t := step; t <= rmax; t += step {
		if f.Eval(o.Add(u.Mul(t))) >= 0 {
			lo, hi := prev, t
			for i := 0; i < 7; i++ {
				m := 0.5 * (lo + hi)
				if f.Eval(o.Add(u.Mul(m))) >= 0 {
					hi = m
				} else {
					lo = m
				}
			}
			return 0.5 * (lo + hi)
		}
		prev = t
	}
	return rmax
}

// cutAt walks out from the joint and returns the first axis distance (at least
// minCut) where the section is clean and stays printable for the next stretch
// of `window` samples: with the joint above (top), the skin may widen toward
// it by at most tan(tilt) per unit height; with the piece standing on this end
// (bottom), it must not widen going up, and the foot may be at most maxR wide.
// Thick tubes passing a support further along don't move the cut (the stub is
// part of the piece).
func (pr branchProfile) cutAt(tanT, minCut, maxR float64, bottom bool, window int) float64 {
	n := len(pr.r) - 1
	ok := func(k int) bool {
		if !pr.clean[k] || !pr.clean[k+1] {
			return false
		}
		for a := 0; a < profileDirs; a++ {
			w := (pr.r[k][a] - pr.r[k+1][a]) / pr.h // widening toward the joint
			if !bottom && w > tanT || bottom && w < -tanT {
				return false
			}
		}
		return true
	}
	for i := int(math.Ceil(minCut / pr.h)); i < n; i++ {
		if bottom && pr.rmax[i] > maxR {
			continue
		}
		good := true
		for k := i; k < min(i+window, n); k++ {
			if !ok(k) {
				good = false
				break
			}
		}
		if good {
			return float64(i) * pr.h
		}
	}
	return float64(n) * pr.h
}

// footR is the widest section a branch may stand on.
func (b *splitBranch) footR() float64 {
	if b.Thin {
		return 3 * b.R
	}
	return 1.6 * b.R
}

// placeCuts probes every branch, picks which end each branch piece stands on
// and sets its cut distances; branches left too short get one middle cut
// (supports) or none (absorbed into the joint). A support's fillet where it
// meets a tube is joint geometry: its flare is held to the joint limit, so the
// stub it leaves on the tube stays short.
func (g *splitGraph) placeCuts(f *Field, strutTilt, jointTilt, minLen float64) {
	var wg sync.WaitGroup
	for bi := range g.branches {
		wg.Add(1)
		go func(b *splitBranch) {
			defer wg.Done()
			var top, bot [2]float64
			window := int(math.Ceil(1.5 / 0.12))
			tanT := math.Tan(strutTilt * math.Pi / 180)
			if b.Thin {
				tanT = math.Tan(jointTilt * math.Pi / 180)
			}
			for s := 0; s < 2; s++ {
				if b.J[s] < 0 {
					continue
				}
				pr := probeBranch(f, b, s)
				lo := g.clearOf(b, s)
				top[s] = pr.cutAt(tanT, lo, 0, false, window)
				bot[s] = pr.cutAt(tanT, lo, b.footR(), true, window)
			}
			switch {
			case b.J[0] < 0 && b.J[1] < 0:
			case b.J[0] < 0:
				b.Bottom = 1
				b.Cut[1] = bot[1]
			case b.J[1] < 0:
				b.Bottom = 0
				b.Cut[0] = bot[0]
			default:
				b.Bottom = 0
				if bot[1]+top[0] < bot[0]+top[1] {
					b.Bottom = 1
				}
				b.Cut[b.Bottom] = bot[b.Bottom]
				b.Cut[1-b.Bottom] = top[1-b.Bottom]
			}
		}(&g.branches[bi])
	}
	wg.Wait()
	for bi := range g.branches {
		g.shorten(bi, minLen)
	}
}

// clearOf is how far along branch b from end s its axis needs to go to leave
// every other tube meeting it there (their cylinders), plus half a radius.
func (g *splitGraph) clearOf(b *splitBranch, s int) float64 {
	d := b.dir(s)
	need := 0.5 * b.R
	for ji := range g.junctions {
		if g.root(ji) != g.root(b.J[s]) {
			continue
		}
		for _, ti := range g.onTubes[ji] {
			if ti == b.Tube {
				continue
			}
			t := g.tubes[ti]
			sin := math.Max(d.Cross(t.B.Sub(t.A).Normalize()).Len(), 0.25)
			need = math.Max(need, t.R/sin+0.5*b.R)
		}
	}
	return need
}

// shorten absorbs or single-cuts branch bi if its piece would be shorter
// than minLen. It reports whether the branch lost its own piece.
func (g *splitGraph) shorten(bi int, minLen float64) bool {
	b := &g.branches[bi]
	if b.Absorbed || b.Mid {
		return false
	}
	if b.J[0] < 0 && b.J[1] < 0 || b.keep() >= minLen {
		return false
	}
	both := b.J[0] >= 0 && b.J[1] >= 0
	if both && b.Thin && g.root(b.J[0]) != g.root(b.J[1]) && b.keep() > 0 {
		b.Mid = true
		return true
	}
	b.Absorbed = true
	if both {
		g.union(b.J[0], b.J[1])
	}
	return true
}

// midPoint is where a Mid branch is cut: halfway between its two cut planes.
func (b *splitBranch) midPoint() Vec3 {
	return b.P[0].Add(b.dir(0).Mul(b.Cut[0] + 0.5*b.keep()))
}
