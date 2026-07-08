package main

import "math"

// tube is a segment + radius used as a hard constraint during relaxation: the
// mesh (a soap film) may shrink toward its neighbors' average but must never
// pass inside the tubes, which act as the film's rigid wire frame.
type tube struct {
	A, B Vec3
	R    float64
}

// collectTubes pulls the segment+radius out of every Capsule primitive in the
// field (struts, supports, and the 12 wireframe edges). Non-capsule primitives
// are ignored — the reprojection constraint only understands tubes.
func collectTubes(groups ...[]Primitive) []tube {
	var out []tube
	for _, g := range groups {
		for _, p := range g {
			if c, ok := p.(Capsule); ok {
				out = append(out, tube{A: c.A, B: c.B, R: c.Radius})
			}
		}
	}
	return out
}

// closestOnSeg returns the closest point on segment A→B to p.
func closestOnSeg(a, b, p Vec3) Vec3 {
	ab := b.Sub(a)
	ab2 := ab.Dot(ab)
	t := 0.0
	if ab2 > 1e-18 {
		t = clampF(p.Sub(a).Dot(ab)/ab2, 0, 1)
	}
	return a.Add(ab.Mul(t))
}

// reprojectOutOfTubes pushes p to the surface of the tube it penetrates
// deepest, if any. Returns the corrected point. Using the single deepest
// penetration (rather than sweeping every tube) avoids oscillation; repeated
// relaxation passes clean up any residual shallow overlaps.
func reprojectOutOfTubes(p Vec3, tubes []tube) Vec3 {
	worstPen := 0.0 // penetration depth = R - |p-c|, positive means inside
	var worstC Vec3
	var worstR float64
	found := false
	for _, tb := range tubes {
		c := closestOnSeg(tb.A, tb.B, p)
		d := p.Sub(c).Len()
		pen := tb.R - d
		if pen > worstPen {
			worstPen = pen
			worstC = c
			worstR = tb.R
			found = true
		}
	}
	if !found {
		return p
	}
	dir := p.Sub(worstC)
	l := dir.Len()
	if l < 1e-12 {
		// On the axis: no defined outward direction; nudge along +Y so the
		// next pass has a gradient to work with.
		return worstC.Add(V(0, worstR, 0))
	}
	return worstC.Add(dir.Mul(worstR / l))
}

// relaxSurfaceTension runs constrained mean-curvature flow on the marching-
// cubes mesh (organic_sleeve_method.md Step 6): each pass moves every vertex a
// fraction λ toward the average of its neighbors (surface-area gradient
// descent → surface tension), reprojects any vertex that fell inside a tube
// back onto the tube surface, then re-inflates along the normals to conserve
// the enclosed volume so bulges redistribute into the low-curvature spans
// instead of the whole sleeve shrinking.
func relaxSurfaceTension(tris []Triangle, tubes []tube, passes int, lambda float64) []Triangle {
	if passes <= 0 || len(tris) == 0 {
		return tris
	}
	verts, faces := weldMesh(tris)
	neighbors := buildAdjacency(verts, faces)
	v0 := meshVolume(verts, faces)

	next := make([]Vec3, len(verts))
	for pass := 0; pass < passes; pass++ {
		// 1. Laplacian smoothing toward the neighbor centroid.
		for i, p := range verts {
			nb := neighbors[i]
			if len(nb) == 0 {
				next[i] = p
				continue
			}
			var sum Vec3
			for _, j := range nb {
				sum = sum.Add(verts[j])
			}
			avg := sum.Mul(1.0 / float64(len(nb)))
			next[i] = p.Add(avg.Sub(p).Mul(lambda))
		}
		// 2. Reproject back out of the tubes (the rigid wire frame).
		for i := range next {
			next[i] = reprojectOutOfTubes(next[i], tubes)
		}
		verts, next = next, verts

		// 3. Volume conservation: offset every vertex along its normal so the
		// enclosed volume returns to v0 (material squeezed out of bulges
		// re-inflates the flat spans). Skipped if the mesh has no volume.
		vol := meshVolume(verts, faces)
		area := meshArea(verts, faces)
		if area > 1e-9 && math.Abs(v0) > 1e-9 {
			offset := (v0 - vol) / area
			normals := vertexNormals(verts, faces)
			for i := range verts {
				verts[i] = verts[i].Add(normals[i].Mul(offset))
			}
			// Re-clamp: the inflation may have pushed a vertex back into a tube.
			for i := range verts {
				verts[i] = reprojectOutOfTubes(verts[i], tubes)
			}
		}
	}

	// Rebuild triangles with fresh per-vertex normals.
	normals := vertexNormals(verts, faces)
	out := make([]Triangle, 0, len(faces))
	for _, f := range faces {
		out = append(out, Triangle{
			A: verts[f[0]], B: verts[f[1]], C: verts[f[2]],
			NA: normals[f[0]], NB: normals[f[1]], NC: normals[f[2]],
		})
	}
	return out
}

// weldMesh merges coincident triangle vertices into a shared index buffer.
// Marching cubes emits the same crossing point (bit-identical) for the (up to
// four) cells sharing a grid edge, so quantizing to a fine grid welds them
// exactly without collapsing genuinely distinct vertices.
func weldMesh(tris []Triangle) ([]Vec3, [][3]int32) {
	const q = 1e6 // quantization: 1e-6 world units
	key := func(v Vec3) [3]int64 {
		return [3]int64{
			int64(math.Round(v[0] * q)),
			int64(math.Round(v[1] * q)),
			int64(math.Round(v[2] * q)),
		}
	}
	index := make(map[[3]int64]int32, len(tris)*3)
	var verts []Vec3
	get := func(v Vec3) int32 {
		k := key(v)
		if id, ok := index[k]; ok {
			return id
		}
		id := int32(len(verts))
		index[k] = id
		verts = append(verts, v)
		return id
	}
	faces := make([][3]int32, 0, len(tris))
	for _, t := range tris {
		ia, ib, ic := get(t.A), get(t.B), get(t.C)
		if ia == ib || ib == ic || ia == ic {
			continue // degenerate after welding
		}
		faces = append(faces, [3]int32{ia, ib, ic})
	}
	return verts, faces
}

// buildAdjacency returns, for each vertex, the sorted unique set of vertices it
// shares a triangle edge with.
func buildAdjacency(verts []Vec3, faces [][3]int32) [][]int32 {
	sets := make([]map[int32]struct{}, len(verts))
	for i := range sets {
		sets[i] = map[int32]struct{}{}
	}
	link := func(a, b int32) {
		sets[a][b] = struct{}{}
		sets[b][a] = struct{}{}
	}
	for _, f := range faces {
		link(f[0], f[1])
		link(f[1], f[2])
		link(f[2], f[0])
	}
	out := make([][]int32, len(verts))
	for i, s := range sets {
		lst := make([]int32, 0, len(s))
		for j := range s {
			lst = append(lst, j)
		}
		out[i] = lst
	}
	return out
}

// meshVolume: signed volume via the divergence theorem, Σ a·(b×c)/6.
func meshVolume(verts []Vec3, faces [][3]int32) float64 {
	vol := 0.0
	for _, f := range faces {
		a, b, c := verts[f[0]], verts[f[1]], verts[f[2]]
		vol += a.Dot(b.Cross(c))
	}
	return vol / 6.0
}

// meshArea: total triangle area.
func meshArea(verts []Vec3, faces [][3]int32) float64 {
	area := 0.0
	for _, f := range faces {
		a, b, c := verts[f[0]], verts[f[1]], verts[f[2]]
		area += 0.5 * b.Sub(a).Cross(c.Sub(a)).Len()
	}
	return area
}

// vertexNormals: area-weighted (unnormalized cross products naturally weight by
// twice the triangle area) accumulation, then normalized per vertex.
func vertexNormals(verts []Vec3, faces [][3]int32) []Vec3 {
	acc := make([]Vec3, len(verts))
	for _, f := range faces {
		a, b, c := verts[f[0]], verts[f[1]], verts[f[2]]
		n := b.Sub(a).Cross(c.Sub(a))
		acc[f[0]] = acc[f[0]].Add(n)
		acc[f[1]] = acc[f[1]].Add(n)
		acc[f[2]] = acc[f[2]].Add(n)
	}
	for i := range acc {
		acc[i] = acc[i].Normalize()
	}
	return acc
}
