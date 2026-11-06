package engine

import (
	"archive/tar"
	"bufio"
	"bytes"
	"fmt"
	"math"
	"os"
	"strings"
	"time"
)

// Print splitting, stage 4: export. The cut program is replayed on a skin mesh
// at the export resolution. The combined STL holds every piece (and every rod,
// in its sockets) exactly where it sits in the cube — separate shells, left
// for the slicer to arrange. The archive holds each piece on its own, turned to
// stand on its print face at z = 0, plus the two rods and a parts list.

// printFrame maps a piece from world space onto the print bed (mm, bed at z=0).
type printFrame struct {
	rot    Quat
	mm     float64
	offset Vec3
}

func (f printFrame) apply(p Vec3) Vec3 { return f.rot.RotateVec(p).Mul(f.mm).Add(f.offset) }

func (pl *SplitPlan) printFrame(p *SplitPiece) printFrame {
	rot := QuatFromUnitVectors(p.BedN.Normalize(), V(0, 0, -1))
	f := printFrame{rot: rot, mm: pl.MMPerUnit}
	f.offset = V(0, 0, -rot.RotateVec(p.BedQ)[2]*pl.MMPerUnit)
	return f
}

func transformTris(tris []Triangle, f printFrame) []Triangle {
	out := make([]Triangle, len(tris))
	for i, t := range tris {
		n := f.rot.RotateVec(t.B.Sub(t.A).Cross(t.C.Sub(t.A))).Normalize()
		out[i] = Triangle{A: f.apply(t.A), B: f.apply(t.B), C: f.apply(t.C), NA: n, NB: n, NC: n}
	}
	return out
}

func trisBounds(tris []Triangle) AABB {
	b := AABB{Min: V(math.Inf(1), math.Inf(1), math.Inf(1)), Max: V(math.Inf(-1), math.Inf(-1), math.Inf(-1))}
	for _, t := range tris {
		for _, p := range [3]Vec3{t.A, t.B, t.C} {
			for i := 0; i < 3; i++ {
				b.Min[i] = math.Min(b.Min[i], p[i])
				b.Max[i] = math.Max(b.Max[i], p[i])
			}
		}
	}
	return b
}

// PieceName is the piece's file / label name, zero-padded to the plan's size.
func (pl *SplitPlan) PieceName(p *SplitPiece) string {
	w := len(itoa(len(pl.Pieces)))
	return fmt.Sprintf("P%0*d", w, p.ID)
}

// Describe summarises what a piece is made of, e.g. "joint + 2 struts".
func (pl *SplitPlan) Describe(p *SplitPiece) string {
	var joints, halves, thick, thin int
	for _, ai := range p.Atoms {
		a := pl.atoms[ai]
		switch {
		case a.Kind == "strut" && a.Thin:
			thin++
		case a.Kind == "strut":
			thick++
		case a.Kind == "half":
			halves++
		default:
			joints++
		}
	}
	var parts []string
	plural := func(n int, one, many string) {
		switch {
		case n == 1:
			parts = append(parts, one)
		case n > 1:
			parts = append(parts, fmt.Sprintf("%d %s", n, many))
		}
	}
	plural(joints, "joint", "joints")
	plural(halves, "half joint", "half joints")
	plural(thick, "strut", "struts")
	plural(thin, "thin strut", "thin struts")
	return strings.Join(parts, " + ")
}

// ExportSplit replays the plan at voxelMM and writes the combined STL to
// stlPath and the per-piece archive to tarPath (either may be empty).
func (pl *SplitPlan) ExportSplit(stlPath, tarPath string, voxelMM float64, progress func(float64)) error {
	report := func(f float64) {
		if progress != nil {
			progress(f)
		}
	}
	m := newSkinMesh(pl.skin, pl.box, voxelMM/pl.MMPerUnit, func(f float64) { report(0.8 * f) })
	parts, err := pl.realize(m)
	if err != nil {
		return err
	}
	report(0.85)

	var sink *stlSink
	if stlPath != "" {
		if sink, err = newSTLSink(stlPath); err != nil {
			return err
		}
	}
	var tw *tar.Writer
	var tf *os.File
	var tbuf *bufio.Writer
	if tarPath != "" {
		if tf, err = os.Create(tarPath); err != nil {
			return err
		}
		tbuf = bufio.NewWriterSize(tf, 1<<20)
		tw = tar.NewWriter(tbuf)
	}
	now := time.Now()
	addFile := func(name string, data []byte) error {
		if tw == nil {
			return nil
		}
		if err := tw.WriteHeader(&tar.Header{Name: name, Mode: 0o644, Size: int64(len(data)), ModTime: now}); err != nil {
			return err
		}
		_, err := tw.Write(data)
		return err
	}

	// Assembled: millimetres, the sculpture resting on z = 0.
	low := math.Inf(1)
	for _, part := range parts {
		for _, t := range part {
			for _, v := range m.T[t] {
				low = math.Min(low, m.P[v][2])
			}
		}
	}
	assembled := printFrame{rot: QuatIdentity(), mm: pl.MMPerUnit, offset: V(0, 0, -low*pl.MMPerUnit)}
	for i, p := range pl.Pieces {
		tris := m.triangles(parts[i])
		if sink != nil {
			sink.writeTris(transformTris(tris, assembled))
		}
		if tw != nil {
			f := pl.printFrame(p)
			placed := transformTris(tris, f)
			b := trisBounds(placed)
			f.offset = f.offset.Sub(V(b.Min[0], b.Min[1], 0))
			placed = transformTris(tris, f)
			snapToBed(placed, 1e-6)
			if err := addFile(pl.PieceName(p)+".stl", stlBytes(placed)); err != nil {
				return err
			}
		}
		report(0.85 + 0.15*float64(i+1)/float64(len(pl.Pieces)+1))
	}
	if sink != nil {
		for _, c := range pl.Cuts {
			if len(c.Keys) > 0 {
				sink.writeTris(transformTris(pl.rodTris(c), assembled))
			}
		}
	}
	mainN, thinN, dashN := pl.RodCounts()
	if dashN > 0 {
		// The dash key prints lying on its long side: length along y,
		// its height across the cut along x, its width up.
		w, l, h := pl.dashKey()
		mm := pl.MMPerUnit
		var key []Triangle
		for _, t := range boxMesh(h*mm, l*mm, w*mm) {
			up := V(0, 0, w*mm/2)
			key = append(key, Triangle{A: t.A.Add(up), B: t.B.Add(up), C: t.C.Add(up), NA: t.NA, NB: t.NB, NC: t.NC})
		}
		if err := addFile("key-dash.stl", stlBytes(key)); err != nil {
			return err
		}
	}
	for _, rod := range []struct {
		name string
		spec RodSpec
		n    int
	}{{"main", pl.Print.MainRod, mainN}, {"thin", pl.Print.ThinRod, thinN}} {
		if rod.n > 0 {
			if err := addFile("rod-"+rod.name+".stl", stlBytes(rodMesh(pl.rodShape(rod.spec, false), pl.MMPerUnit))); err != nil {
				return err
			}
		}
	}
	if err := addFile("parts.txt", []byte(pl.PartsList())); err != nil {
		return err
	}
	report(1)

	var firstErr error
	if sink != nil {
		firstErr = sink.Close()
	}
	if tw != nil {
		if err := tw.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
		if err := tbuf.Flush(); err != nil && firstErr == nil {
			firstErr = err
		}
		if err := tf.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
	}
	return firstErr
}

// snapToBed puts vertices left a rounding error under the bed onto z = 0.
func snapToBed(tris []Triangle, tol float64) {
	for i := range tris {
		for _, p := range []*Vec3{&tris[i].A, &tris[i].B, &tris[i].C} {
			if p[2] < 0 && p[2] > -tol {
				p[2] = 0
			}
		}
	}
}

func stlBytes(tris []Triangle) []byte {
	var buf bytes.Buffer
	buf.Grow(84 + 50*len(tris))
	_ = stlHeader(&buf, uint32(len(tris)))
	var rec [50]byte
	for _, t := range tris {
		encodeTriangleSTL(t, rec[:])
		buf.Write(rec[:])
	}
	return buf.Bytes()
}

// meshTrisVolume is the signed volume a closed triangle mesh encloses.
func meshTrisVolume(tris []Triangle) float64 {
	v := 0.0
	for _, t := range tris {
		v += t.A.Dot(t.B.Cross(t.C)) / 6
	}
	return v
}

// fitCheck names, for each kind of guide, the joint whose two pieces are
// the smallest to print: a cheap test of the fit before printing the rest.
func (pl *SplitPlan) fitCheck() string {
	size := func(pi int) float64 { return math.Abs(meshTrisVolume(pl.Pieces[pi].Tris)) }
	var b strings.Builder
	b.WriteString("to check the fit first, print these pairs and try their guides:\n")
	for _, kind := range []string{"main", "thin", "dash"} {
		var best *SplitCut
		bv := math.Inf(1)
		for _, c := range pl.Cuts {
			if c.Rod != kind || len(c.Sides[0]) != 1 || len(c.Sides[1]) != 1 {
				continue
			}
			if v := size(c.A) + size(c.B); v < bv {
				best, bv = c, v
			}
		}
		if best != nil {
			name := map[string]string{"main": "main rod", "thin": "thin rod", "dash": "dash key"}[kind]
			fmt.Fprintf(&b, "    %-9s cut %d: %s + %s\n", name, best.Num, pl.PieceName(pl.Pieces[best.A]), pl.PieceName(pl.Pieces[best.B]))
		}
	}
	b.WriteString("if a guide binds, raise the clearance; if it wobbles, lower it.\n")
	return b.String()
}

// PartsList is the human-readable assembly sheet shipped in the archive.
func (pl *SplitPlan) PartsList() string {
	var b strings.Builder
	mainN, thinN, dashN := pl.RodCounts()
	numbered, slivers := 0, 0
	for _, c := range pl.Cuts {
		if c.Num > 0 {
			numbered++
		} else {
			slivers++
		}
	}
	fmt.Fprintf(&b, "friendlycube print split — %d pieces, %d numbered cuts, %d glue-only contacts\n", len(pl.Pieces), numbered, slivers)
	fmt.Fprintf(&b, "scale %.1f mm cube; strut overhang ≤ %.0f°, joint overhang ≤ %.0f°\n", pl.Print.CubeMM, pl.Print.StrutTilt, pl.Print.JointTilt)
	mm := pl.MMPerUnit
	for _, r := range []struct {
		name string
		spec RodSpec
		n    int
	}{{"main", pl.Print.MainRod, mainN}, {"thin", pl.Print.ThinRod, thinN}} {
		rs := pl.rodShape(r.spec, false)
		fmt.Fprintf(&b, "rods: %d × rod-%s.stl (base %.2f, top %.2f, height %.2f, %.1f mm long)\n",
			r.n, r.name, rs.B*mm, rs.T*mm, rs.H*mm, 2*rs.Lh*mm)
	}
	if dashN > 0 {
		w, l, h := pl.dashKey()
		fmt.Fprintf(&b, "dash keys: %d × key-dash.stl (%.2f×%.2f×%.2f mm), for cuts with no room for a rod: it sits in the groove\n", dashN, w*mm, l*mm, h*mm)
		b.WriteString("under the number on both faces.\n")
	}
	fmt.Fprintf(&b, "rods and keys leave %.2f mm on every side in their sockets; print rods lying on their wide flat side.\n", pl.Print.ClearanceMM)
	b.WriteString(pl.fitCheck())
	b.WriteString("the combined STL has every piece where it sits in the cube, rods in their sockets; in this archive each piece\n")
	b.WriteString("stands on its print face (z = 0). Cut numbers are engraved on both faces of a cut; the bar under a number\n")
	b.WriteString("marks its bottom edge. Cuts marked (no number) had no room to engrave.\n\n")

	for i, p := range pl.Pieces {
		flag := ""
		if p.Flagged {
			flag = fmt.Sprintf("  !! %.0f mm² overhang above the limit — needs support", p.Overhang)
		}
		fmt.Fprintf(&b, "%s  %s%s\n", pl.PieceName(p), pl.Describe(p), flag)
		var lines []string
		for _, c := range pl.Cuts {
			var others []int
			for s := 0; s < 2; s++ {
				for _, x := range c.Sides[s] {
					if x == i {
						others = c.Sides[1-s]
					}
				}
			}
			if others == nil {
				continue
			}
			var names []string
			for _, o := range others {
				names = append(names, pl.PieceName(pl.Pieces[o]))
			}
			num := "glue only"
			if c.Num > 0 {
				num = fmt.Sprintf("cut %d", c.Num)
				if !c.HasLabel {
					num += " (no number)"
				}
			}
			rods := "no rod"
			switch {
			case c.Rod == "dash":
				rods = "1 dash key"
			case c.Rod != "":
				rods = fmt.Sprintf("%d %s rod", len(c.Keys), c.Rod)
				if len(c.Keys) > 1 {
					rods += "s"
				}
			}
			lines = append(lines, fmt.Sprintf("    %-22s → %s, %s", num, strings.Join(names, " + "), rods))
		}
		b.WriteString(strings.Join(lines, "\n"))
		b.WriteString("\n")
	}
	return b.String()
}

// ---- live preview ----

// splitPalette gives neighbouring pieces distinguishable colors.
var splitPalette = []int{0xeae3d4, 0x85b7eb, 0x97c459, 0xfac775, 0xed93b1, 0x7f77dd,
	0x5dcaa5, 0xf0997b, 0xb5b0e8, 0xd4c18a, 0x8ad0d4, 0xc9a0dc}

// PreviewBuffers flattens the plan for the viewer: per vertex a position,
// normal, color, piece ID (0 = guide rod) and an explode direction (the
// piece's centroid; rods take the midpoint of their two pieces). Rods come
// first so the viewer can draw them opaque before the see-through pieces.
func (pl *SplitPlan) PreviewBuffers() (pos, nrm, col, pid, off []float32) {
	push := func(t Triangle, c [3]float32, id float32, o Vec3) {
		n := t.B.Sub(t.A).Cross(t.C.Sub(t.A)).Normalize()
		for _, p := range [3]Vec3{t.A, t.B, t.C} {
			pos = append(pos, float32(p[0]), float32(p[1]), float32(p[2]))
			nrm = append(nrm, float32(n[0]), float32(n[1]), float32(n[2]))
			col = append(col, c[0], c[1], c[2])
			pid = append(pid, id)
			off = append(off, float32(o[0]), float32(o[1]), float32(o[2]))
		}
	}
	rodCol := hex2rgb(0x9aa4b0)
	for _, c := range pl.Cuts {
		if len(c.Keys) == 0 {
			continue
		}
		mid := Lerp(pl.Pieces[c.A].Centroid, pl.Pieces[c.B].Centroid, 0.5)
		for _, t := range pl.rodTris(c) {
			push(t, rodCol, 0, mid)
		}
	}
	for i, p := range pl.Pieces {
		c := hex2rgb(splitPalette[i%len(splitPalette)])
		if p.Flagged {
			c = hex2rgb(Colors.CollisionSnap)
		}
		for _, t := range p.Tris {
			push(t, c, float32(p.ID), p.Centroid)
		}
	}
	return
}

// SplitInfo is the plan summary the viewer shows beside the pieces.
type SplitInfo struct {
	Pieces   []SplitPieceInfo `json:"pieces"`
	Cuts     []SplitCutInfo   `json:"cuts"`
	RodsMain int              `json:"rodsMain"`
	RodsThin int              `json:"rodsThin"`
	DashKeys int              `json:"dashKeys"`
	Flagged  int              `json:"flagged"`
}

type SplitPieceInfo struct {
	ID       int        `json:"id"`
	Name     string     `json:"name"`
	Desc     string     `json:"desc"`
	Centroid [3]float64 `json:"centroid"`
	Flagged  bool       `json:"flagged"`
	Overhang float64    `json:"overhang"`
}

type SplitCutInfo struct {
	Num      int        `json:"num"`
	A        int        `json:"a"` // piece IDs: the main piece on each side
	B        int        `json:"b"`
	SideA    []int      `json:"sideA"` // every piece on each side (a joint's two halves can share a face)
	SideB    []int      `json:"sideB"`
	Pos      [3]float64 `json:"pos"`
	Rod      string     `json:"rod"`
	Keys     int        `json:"keys"`
	HasLabel bool       `json:"hasLabel"`
}

func (pl *SplitPlan) Info() SplitInfo {
	var info SplitInfo
	info.RodsMain, info.RodsThin, info.DashKeys = pl.RodCounts()
	for _, p := range pl.Pieces {
		if p.Flagged {
			info.Flagged++
		}
		info.Pieces = append(info.Pieces, SplitPieceInfo{ID: p.ID, Name: pl.PieceName(p), Desc: pl.Describe(p),
			Centroid: p.Centroid, Flagged: p.Flagged, Overhang: math.Round(p.Overhang*10) / 10})
	}
	for _, c := range pl.Cuts {
		at := c.Q
		if c.HasLabel {
			at = c.Label
		} else if len(c.Keys) > 0 {
			at = c.Keys[0]
		}
		ids := func(ps []int) []int {
			out := make([]int, len(ps))
			for i, p := range ps {
				out[i] = pl.Pieces[p].ID
			}
			return out
		}
		info.Cuts = append(info.Cuts, SplitCutInfo{Num: c.Num, A: pl.Pieces[c.A].ID, B: pl.Pieces[c.B].ID,
			SideA: ids(c.Sides[0]), SideB: ids(c.Sides[1]), Pos: at, Rod: c.Rod, Keys: len(c.Keys), HasLabel: c.HasLabel})
	}
	return info
}
