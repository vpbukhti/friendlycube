package engine

import (
	"math"
	"testing"
	"time"
)

func testPlan(t *testing.T) *SplitPlan {
	s := DefaultSettings()
	sk := BakeSkeleton(s, 0x005a08)
	sp := DefaultSkinParams()
	sp.BlendK, sp.Gamma, sp.Cap, sp.Corner, sp.ClipBlend = 0.05, 0.8, 0.12, 0.62, 0.034
	t0 := time.Now()
	pl := PlanSplit(s, sk, sp, DefaultPrintParams(), nil)
	t.Logf("plan: %v, %d cuts (%d undone), %d atoms, %d pieces, %d glued joints",
		time.Since(t0), len(pl.cuts), len(pl.merged), len(pl.atoms), len(pl.Pieces), len(pl.Cuts))
	return pl
}

// The pieces must partition the skin: each one a single closed shell, their
// volumes adding up to the whole sculpture's (less the sockets and numbers).
func TestSplitPartitionsSkin(t *testing.T) {
	if testing.Short() {
		t.Skip("meshes the whole sculpture")
	}
	pl := testPlan(t)
	m := newSkinMesh(pl.skin, pl.box, pl.vox, nil)
	whole := math.Abs(meshTrisVolume(m.triangles(liveTris(m))))
	parts, err := pl.realize(m)
	if err != nil {
		t.Fatal(err)
	}
	comp, _ := m.components()
	sum := 0.0
	for i, part := range parts {
		if len(part) == 0 {
			t.Errorf("piece %d is empty", i+1)
			continue
		}
		shells := map[int32]bool{}
		edges := map[[2]int32]int{}
		for _, tr := range part {
			shells[comp[tr]] = true
			v := m.T[tr]
			for k := 0; k < 3; k++ {
				edges[[2]int32{v[k], v[(k+1)%3]}]++
			}
		}
		if len(shells) != 1 {
			t.Errorf("piece %d is %d separate shells", i+1, len(shells))
		}
		open := 0
		for e, n := range edges {
			if n != 1 || edges[[2]int32{e[1], e[0]}] != 1 {
				open++
			}
		}
		if open > 0 {
			t.Errorf("piece %d: %d unpaired edges", i+1, open)
		}
		vol := meshTrisVolume(m.triangles(part))
		if vol <= 0 {
			t.Errorf("piece %d: inside out (volume %g)", i+1, vol)
		}
		sum += vol
	}
	mm3 := math.Pow(pl.MMPerUnit, 3)
	// The loose guides themselves: every rod and dash key closed and outward.
	for _, c := range pl.Cuts {
		if tr := pl.rodTris(c); len(tr) > 0 && meshTrisVolume(tr) <= 0 {
			t.Errorf("cut %d: %s guide is inside out", c.Num, c.Rod)
		}
	}
	t.Logf("volume: whole %.0f mm³, pieces %.0f mm³", whole*mm3, sum*mm3)
	if d := (whole - sum) / whole; d < 0 || d > 0.02 {
		t.Errorf("pieces don't partition the skin: %.4f vs %.4f", sum, whole)
	}

	main, thin, dash := pl.RodCounts()
	nolabel, norod, glue := 0, 0, 0
	for _, c := range pl.Cuts {
		if c.Num == 0 {
			glue++
		}
		if !c.HasLabel {
			nolabel++
		}
		if c.Rod == "" {
			norod++
		}
	}
	flagged := 0
	for _, p := range pl.Pieces {
		if p.Flagged {
			flagged++
			t.Logf("%s %s: flagged, %.1f mm² overhang", pl.PieceName(p), pl.Describe(p), p.Overhang)
		}
	}
	t.Logf("rods: %d main, %d thin, %d dash keys; joints without rod %d, without number %d, glue-only %d; flagged pieces %d",
		main, thin, dash, norod, nolabel, glue, flagged)
}

func liveTris(m *cutMesh) []int32 {
	var out []int32
	for t, tr := range m.T {
		if tr[0] >= 0 {
			out = append(out, int32(t))
		}
	}
	return out
}
