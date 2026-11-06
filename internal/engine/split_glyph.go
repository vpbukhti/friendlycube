package engine

import (
	"math"
	"sort"
)

// Engraved cut numbers. Each digit is drawn as round-capped strokes (lines and
// arcs) in a cell 0.6 wide and 1 tall; the strokes of a whole number are
// outlined into exact polygons (outer contours plus the counters of 0, 4, 6, 8
// and 9) — vector outlines, the way an SVG font would be — and cut into the
// face as pockets. A bar under the number tells 6 from 9 and which way is up.

var digitStrokes = func() [10][][]vec2 {
	arc := func(cx, cy, r, a0, a1 float64) []vec2 {
		n := int(math.Ceil(math.Abs(a1-a0) / 6))
		var out []vec2
		for i := 0; i <= n; i++ {
			a := (a0 + (a1-a0)*float64(i)/float64(n)) * math.Pi / 180
			out = append(out, vec2{cx + r*math.Cos(a), cy + r*math.Sin(a)})
		}
		return out
	}
	cat := func(parts ...[]vec2) []vec2 {
		var out []vec2
		for _, p := range parts {
			out = append(out, p...)
		}
		return out
	}
	pts := func(xy ...float64) []vec2 {
		var out []vec2
		for i := 0; i+1 < len(xy); i += 2 {
			out = append(out, vec2{xy[i], xy[i+1]})
		}
		return out
	}
	return [10][][]vec2{
		{cat(arc(0.3, 0.7, 0.3, 180, 0), arc(0.3, 0.3, 0.3, 0, -180), pts(0, 0.7))},
		{pts(0.1, 0.8, 0.34, 1, 0.34, 0), pts(0.08, 0, 0.58, 0)},
		{cat(arc(0.3, 0.7, 0.29, 165, -35), pts(0.01, 0, 0.6, 0))},
		{cat(arc(0.3, 0.76, 0.24, 160, -90), arc(0.3, 0.27, 0.27, 90, -160))},
		{pts(0.46, 0, 0.46, 1, 0, 0.3, 0.6, 0.3)},
		{cat(pts(0.56, 1, 0.07, 1, 0.04, 0.56, 0.3, 0.58), arc(0.3, 0.29, 0.29, 90, -150))},
		{arc(0.3, 0.29, 0.29, 0, 360), pts(0.04, 0.4, 0.42, 1)},
		{pts(0, 1, 0.6, 1, 0.18, 0)},
		{arc(0.3, 0.76, 0.24, 0, 360), arc(0.3, 0.27, 0.27, 0, 360)},
		{arc(0.3, 0.71, 0.29, 0, 360), pts(0.56, 0.6, 0.18, 0)},
	}
}()

// glyphGap is the material left between strokes, in stroke widths: digits
// and the bar stay this far apart however small the number gets.
const glyphGap = 1.3

// glyphPitch is the advance from one digit to the next; glyphBar the centre
// line of a bar barW wide, below the baseline.
func glyphPitch(h, sw float64) float64  { return 0.6*h + (1+glyphGap)*sw }
func glyphBar(sw, barW float64) float64 { return sw/2 + glyphGap*sw + barW/2 }

// labelBar is the bar under a number: the stroke-wide underline of an
// ordinary label, or the groove of a dash key (a loose printed key that
// guides a cut with no room for a rod), which has one fixed size.
type labelBar struct {
	W, L float64 // width; length (0 = span the digits)
}

// labelBox is the size (width, height) of number text at character height h
// with stroke width sw over bar b, and the offset of the box's center from
// the text origin (left end of the baseline).
func labelBox(text string, h, sw float64, b labelBar) (w, ht, cx, cy float64) {
	d := float64(len(text))
	run := 0.6*h + glyphPitch(h, sw)*(d-1)
	bar := glyphBar(sw, b.W)
	return math.Max(run+sw, b.L), h + sw/2 + bar + b.W/2, run / 2, (h + sw/2 - bar - b.W/2) / 2
}

// strokeBar is an ordinary label's underline.
func strokeBar(sw float64) labelBar { return labelBar{W: sw} }

// dashRect is a dash key groove's rectangle in label coordinates, centred
// under the digits.
func dashRect(text string, h, sw float64, b labelBar) []vec2 {
	run := 0.6*h + glyphPitch(h, sw)*float64(len(text)-1)
	_, _, cx, cy := labelBox(text, h, sw, b)
	x, y := run/2-cx, -glyphBar(sw, b.W)-cy
	return []vec2{{x - b.L/2, y - b.W/2}, {x + b.L/2, y - b.W/2}, {x + b.L/2, y + b.W/2}, {x - b.L/2, y + b.W/2}}
}

// glyphOutline is the engraved shape of text: contours in label coordinates
// (centered on the label box, y up), each with its nesting depth — even depth
// is an outer contour (counter-clockwise), odd a counter (clockwise).
type glyphOutline struct {
	rings [][]vec2
	depth []int
}

// numberOutline outlines text's digits, with the stroke underline unless
// the bar is a dash key groove (cut separately).
func numberOutline(text string, h, sw float64, b labelBar, underline bool) glyphOutline {
	var segs [][2]vec2
	_, _, cx, cy := labelBox(text, h, sw, b)
	add := func(a, b vec2) { segs = append(segs, [2]vec2{{a.X - cx, a.Y - cy}, {b.X - cx, b.Y - cy}}) }
	for k, ch := range text {
		ox := float64(k) * glyphPitch(h, sw)
		for _, s := range digitStrokes[ch-'0'] {
			for i := 0; i+1 < len(s); i++ {
				add(vec2{ox + s[i].X*h, s[i].Y * h}, vec2{ox + s[i+1].X*h, s[i+1].Y * h})
			}
		}
	}
	if underline {
		add(vec2{0, -glyphBar(sw, sw)}, vec2{2 * cx, -glyphBar(sw, sw)})
	}

	half := sw / 2
	dist := func(p vec2) float64 {
		d := math.MaxFloat64
		for _, s := range segs {
			if x := segDist2(p, s[0], s[1]); x < d {
				d = x
			}
		}
		return d - half
	}
	w, ht, _, _ := labelBox(text, h, sw, b)
	o := contourField(dist, vec2{-w/2 - sw, -ht/2 - sw}, vec2{w/2 + sw, ht/2 + sw}, sw/8, sw/60)
	// Counters too small to print (small 8s and 9s) are filled in.
	var out glyphOutline
	for i, r := range o.rings {
		if o.depth[i] == 1 && -ringArea(r) < 2*sw*sw {
			continue
		}
		out.rings = append(out.rings, r)
		out.depth = append(out.depth, o.depth[i])
	}
	return out
}

// contourField traces the zero contours of a 2D field (negative inside) over
// the box lo..hi with marching squares at grid step `step`, simplifies them to
// within tol, and orients them by nesting depth.
func contourField(f func(vec2) float64, lo, hi vec2, step, tol float64) glyphOutline {
	nx := int(math.Ceil((hi.X-lo.X)/step)) + 1
	ny := int(math.Ceil((hi.Y-lo.Y)/step)) + 1
	val := make([]float64, nx*ny)
	at := func(i, j int) vec2 { return vec2{lo.X + float64(i)*step, lo.Y + float64(j)*step} }
	for j := 0; j < ny; j++ {
		for i := 0; i < nx; i++ {
			v := f(at(i, j))
			if v == 0 {
				v = 1e-12
			}
			val[j*nx+i] = v
		}
	}
	// Crossing points on grid edges: horizontal edge (i,j)-(i+1,j) has id
	// 2(j·nx+i), vertical (i,j)-(i,j+1) has id 2(j·nx+i)+1.
	point := func(id int) vec2 {
		c := id / 2
		i, j := c%nx, c/nx
		a, va := at(i, j), val[c]
		var b vec2
		var vb float64
		if id%2 == 0 {
			b, vb = at(i+1, j), val[c+1]
		} else {
			b, vb = at(i, j+1), val[c+nx]
		}
		return lerp2(a, b, va/(va-vb))
	}
	link := map[int][]int{}
	connect := func(a, b int) {
		link[a] = append(link[a], b)
		link[b] = append(link[b], a)
	}
	for j := 0; j+1 < ny; j++ {
		for i := 0; i+1 < nx; i++ {
			c := j*nx + i
			v := [4]float64{val[c], val[c+1], val[c+nx+1], val[c+nx]}
			// cell edges: bottom, right, top, left
			e := [4]int{2 * c, 2*(c+1) + 1, 2 * (c + nx), 2*c + 1}
			code := 0
			for k := 0; k < 4; k++ {
				if v[k] < 0 {
					code |= 1 << k
				}
			}
			switch code {
			case 0, 15:
			case 5, 10: // saddle: resolve by the cell center
				center := 0.25 * (v[0] + v[1] + v[2] + v[3])
				if (center < 0) == (code == 5) {
					connect(e[0], e[1])
					connect(e[2], e[3])
				} else {
					connect(e[0], e[3])
					connect(e[1], e[2])
				}
			default:
				var cr []int
				for k := 0; k < 4; k++ {
					if (v[k] < 0) != (v[(k+1)%4] < 0) {
						cr = append(cr, e[k])
					}
				}
				connect(cr[0], cr[1])
			}
		}
	}
	ids := make([]int, 0, len(link))
	for id := range link {
		ids = append(ids, id)
	}
	sort.Ints(ids)
	seen := map[int]bool{}
	var out glyphOutline
	for _, start := range ids {
		if seen[start] {
			continue
		}
		var ring []vec2
		prev, cur := -1, start
		for {
			seen[cur] = true
			ring = append(ring, point(cur))
			nb := link[cur]
			next := nb[0]
			if next == prev && len(nb) > 1 {
				next = nb[1]
			}
			prev, cur = cur, next
			if cur == start {
				break
			}
		}
		ring = simplifyRing(ring, tol)
		if len(ring) >= 3 {
			out.rings = append(out.rings, ring)
		}
	}
	for i, r := range out.rings {
		d := 0
		for j, o := range out.rings {
			if i != j && inRing(r[0], o) {
				d++
			}
		}
		out.depth = append(out.depth, d)
		if (ringArea2(r) > 0) != (d%2 == 0) {
			reverseRing(r)
		}
	}
	return out
}

func reverseRing(r []vec2) {
	for i, j := 0, len(r)-1; i < j; i, j = i+1, j-1 {
		r[i], r[j] = r[j], r[i]
	}
}

// simplifyRing is Douglas-Peucker on a closed ring, split at its two most
// distant points.
func simplifyRing(r []vec2, tol float64) []vec2 {
	if len(r) < 8 {
		return r
	}
	far, fd := 0, 0.0
	for i, p := range r {
		if d := p.dist(r[0]); d > fd {
			far, fd = i, d
		}
	}
	keep := make([]bool, len(r))
	keep[0], keep[far] = true, true
	var dp func(a, b int)
	dp = func(a, b int) {
		if b-a < 2 {
			return
		}
		pb := r[b%len(r)]
		best, bd := -1, tol
		for i := a + 1; i < b; i++ {
			if d := segDist2(r[i], r[a], pb); d > bd {
				best, bd = i, d
			}
		}
		if best >= 0 {
			keep[best] = true
			dp(a, best)
			dp(best, b)
		}
	}
	dp(0, far)
	dp(far, len(r))
	var out []vec2
	for i, k := range keep {
		if k {
			out = append(out, r[i])
		}
	}
	return out
}
