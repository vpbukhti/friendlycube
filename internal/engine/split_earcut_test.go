package engine

import (
	"math"
	"math/rand"
	"testing"
)

// Random star polygons with random circular holes: the triangles must be
// counter-clockwise and add up to the polygon's area.
func TestTriangulateArea(t *testing.T) {
	r := rand.New(rand.NewSource(7))
	for it := 0; it < 300; it++ {
		n := 6 + r.Intn(120)
		var outer []vec2
		for i := 0; i < n; i++ {
			a := 2 * math.Pi * float64(i) / float64(n)
			rad := 10 + 6*r.Float64()
			outer = append(outer, vec2{rad * math.Cos(a), rad * math.Sin(a)})
		}
		if r.Intn(2) == 0 {
			for i, j := 0, len(outer)-1; i < j; i, j = i+1, j-1 {
				outer[i], outer[j] = outer[j], outer[i]
			}
		}
		rings := [][]vec2{outer}
		want := math.Abs(ringArea(outer))
		var centers []vec2
		for h := 0; h < r.Intn(5); h++ {
			c := vec2{(r.Float64() - 0.5) * 12, (r.Float64() - 0.5) * 12}
			ok := math.Hypot(c.X, c.Y) < 7
			for _, o := range centers {
				if o.dist(c) < 3.2 {
					ok = false
				}
			}
			if !ok {
				continue
			}
			centers = append(centers, c)
			m := 3 + r.Intn(20)
			var hole []vec2
			for i := 0; i < m; i++ {
				a := 2 * math.Pi * float64(i) / float64(m)
				hole = append(hole, vec2{c.X + 1.5*math.Cos(a), c.Y + 1.5*math.Sin(a)})
			}
			rings = append(rings, hole)
			want -= math.Abs(ringArea(hole))
		}
		var pts []vec2
		for _, rg := range rings {
			pts = append(pts, rg...)
		}
		got := 0.0
		for _, tr := range triangulate(rings) {
			a := 0.5 * cross2(pts[tr[1]].sub(pts[tr[0]]), pts[tr[2]].sub(pts[tr[0]]))
			if a < -1e-9 {
				t.Fatalf("iteration %d: clockwise triangle (%g)", it, a)
			}
			got += a
		}
		if math.Abs(got-want) > 1e-6*want {
			t.Fatalf("iteration %d (%d holes): area %g, want %g", it, len(rings)-1, got, want)
		}
	}
}

// Collinear and repeated ring points must still be used by the triangles:
// a cap that skips one leaves a crack beside it.
func TestTriangulateKeepsEveryPoint(t *testing.T) {
	outer := []vec2{{0, 0}, {1, 0}, {2, 0}, {2, 0}, {3, 0}, {3, 1}, {3, 2}, {1.5, 2}, {0, 2}, {0, 1}}
	hole := []vec2{{1, 0.5}, {1, 1.5}, {2, 1.5}, {2, 1}, {2, 0.5}, {1.5, 0.5}}
	rings := [][]vec2{outer, hole}
	tris := triangulate(rings)
	used := map[int]bool{}
	area := 0.0
	pts := append(append([]vec2{}, outer...), hole...)
	for _, tr := range tris {
		used[tr[0]], used[tr[1]], used[tr[2]] = true, true, true
		area += 0.5 * cross2(pts[tr[1]].sub(pts[tr[0]]), pts[tr[2]].sub(pts[tr[0]]))
	}
	for i := range pts {
		if !used[i] {
			t.Errorf("point %d %v unused", i, pts[i])
		}
	}
	if math.Abs(area-5) > 1e-9 {
		t.Errorf("area %g, want 5", area)
	}
}

// Every digit outlines into closed contours; 0, 4, 6, 8 and 9 have counters.
func TestNumberOutline(t *testing.T) {
	counters := map[rune]int{'0': 1, '4': 1, '6': 1, '8': 2, '9': 1}
	for d := '0'; d <= '9'; d++ {
		o := numberOutline(string(d), 3, 0.45, strokeBar(0.45), true)
		holes := 0
		for i, r := range o.rings {
			if (ringArea(r) > 0) != (o.depth[i]%2 == 0) {
				t.Errorf("%c: ring %d oriented against its depth", d, i)
			}
			if o.depth[i] == 1 {
				holes++
			}
		}
		if holes != counters[d] {
			t.Errorf("%c: %d counters, want %d", d, holes, counters[d])
		}
	}
}
