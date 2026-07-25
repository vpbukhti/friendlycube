package engine

import (
	"runtime"
	"sync"
	"sync/atomic"
)

// Marching cubes. Standard Lorensen-Cline tables (Paul Bourke's well-known
// public-domain transcription). 256 entries; each row in mcTriTable lists
// triangles as vertex-edge indices (terminated by -1). mcEdgeTable[i] tells
// which of the 12 cube edges are crossed by the isosurface for case i.
//
// Grid convention used here: 8 cube corners, indexed 0..7 with v0..v3 on
// the bottom (y=ymin) and v4..v7 on the top (y=ymax). Edges 0..11 connect
// the corners in the order matching the tables below.
//
// The volume is swept in Z-slabs so only a small window of grid slices is ever
// resident (memory is O(res²), not O(res³)). Each slab's slices are sampled in
// parallel over rows, its cell layers extracted in parallel, and the triangles
// emitted in ascending (k, j, i) order — identical to a full-grid pass.

var mcEdges = [12][2]int{
	{0, 1}, {1, 2}, {2, 3}, {3, 0},
	{4, 5}, {5, 6}, {6, 7}, {7, 4},
	{0, 4}, {1, 5}, {2, 6}, {3, 7},
}

// mcSlabBudget bounds the bytes of grid slices resident at once (a slab holds
// slabZ+1 z-slices, each res² float64). ~256 MiB keeps even a 2048³ export in a
// few hundred MB instead of the ~69 GB a full grid would need.
const mcSlabBudget = 256 << 20

// MarchingCubes triangulates field's zero isosurface over box on a res³ grid
// and returns all triangles. progress, if non-nil, is called periodically with
// the fraction [0,1] of work completed. For large exports prefer marchStream,
// which never holds the whole triangle set in memory.
func MarchingCubes(field *Field, box AABB, res int, progress func(float64)) []Triangle {
	var out []Triangle
	marchStream(field, box, res, progress, func(slab []Triangle) {
		out = append(out, slab...)
	})
	return out
}

// marchStream is the streaming core: it sweeps Z in slabs and calls emit with
// each slab's triangles, in ascending (k, j, i) order. emit is called serially
// (never concurrently), so the caller can append or write without locking.
func marchStream(field *Field, box AABB, res int, progress func(float64), emit func([]Triangle)) {
	if res < 2 {
		res = 2
	}
	field.Build()
	nx, ny, nz := res, res, res
	dx := (box.Max[0] - box.Min[0]) / float64(nx-1)
	dy := (box.Max[1] - box.Min[1]) / float64(ny-1)
	dz := (box.Max[2] - box.Min[2]) / float64(nz-1)
	gradEps := 0.5 * minF(minF(dx, dy), dz)
	cellNz := nz - 1

	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers < 1 {
		nWorkers = 1
	}

	// Progress mirrors the old full-grid split: sampling → 0..0.8, extraction →
	// 0.8..1.0. Each z-slice is sampled exactly once (window slides), each cell
	// layer extracted once.
	var doneSample, doneLayer atomic.Int64
	report := func() {
		if progress == nil {
			return
		}
		s := float64(doneSample.Load()) / float64(nz)
		c := float64(doneLayer.Load()) / float64(maxI(cellNz, 1))
		progress(0.8*s + 0.2*c)
	}

	// Slab thickness (in cell layers), bounded by the memory budget.
	sliceBytes := nx * ny * 8
	slabZ := mcSlabBudget / maxI(sliceBytes, 1)
	if slabZ < 1 {
		slabZ = 1
	}
	if slabZ > cellNz {
		slabZ = cellNz
	}

	// Reusable z-slice buffers (each nx*ny float64), recycled as the window slides.
	var pool [][]float64
	getBuf := func() []float64 {
		if n := len(pool); n > 0 {
			b := pool[n-1]
			pool = pool[:n-1]
			return b
		}
		return make([]float64, nx*ny)
	}
	putBuf := func(b []float64) { pool = append(pool, b) }

	// sampleSlice fills dst with the field at z-slice k, parallel over rows.
	sampleSlice := func(k int, dst []float64) {
		z := box.Min[2] + float64(k)*dz
		var wg sync.WaitGroup
		chunk := (ny + nWorkers - 1) / nWorkers
		for w := 0; w < nWorkers; w++ {
			j0 := w * chunk
			j1 := j0 + chunk
			if j1 > ny {
				j1 = ny
			}
			if j0 >= ny {
				break
			}
			wg.Add(1)
			go func(j0, j1 int) {
				defer wg.Done()
				for j := j0; j < j1; j++ {
					y := box.Min[1] + float64(j)*dy
					base := j * nx
					for i := 0; i < nx; i++ {
						x := box.Min[0] + float64(i)*dx
						dst[base+i] = field.Eval(V(x, y, z))
					}
				}
			}(j0, j1)
		}
		wg.Wait()
		doneSample.Add(1)
		report()
	}

	if cellNz < 1 {
		return
	}

	// bottom holds the current window's lowest slice (shared with the next slab).
	bottom := getBuf()
	sampleSlice(0, bottom)

	for base := 0; base < cellNz; base += slabZ {
		L := slabZ
		if base+L > cellNz {
			L = cellNz - base
		}
		// Window slices base..base+L (L+1 of them); slices[0] is already sampled.
		slices := make([][]float64, L+1)
		slices[0] = bottom
		for t := 1; t <= L; t++ {
			b := getBuf()
			sampleSlice(base+t, b)
			slices[t] = b
		}

		// Extract the L cell layers in parallel over contiguous t-chunks, then
		// concatenate in worker order so the output stays k-ascending.
		parts := make([][]Triangle, nWorkers)
		var wg sync.WaitGroup
		chunk := (L + nWorkers - 1) / nWorkers
		for w := 0; w < nWorkers; w++ {
			t0 := w * chunk
			t1 := t0 + chunk
			if t1 > L {
				t1 = L
			}
			if t0 >= L {
				break
			}
			wg.Add(1)
			go func(wid, t0, t1 int) {
				defer wg.Done()
				local := make([]Triangle, 0, 1024)
				for t := t0; t < t1; t++ {
					extractLayer(field, box, nx, ny, base+t, dx, dy, dz, gradEps, slices[t], slices[t+1], &local)
					doneLayer.Add(1)
					report()
				}
				parts[wid] = local
			}(w, t0, t1)
		}
		wg.Wait()

		var slab []Triangle
		for _, p := range parts {
			slab = append(slab, p...)
		}
		emit(slab)

		// Slide the window: the top slice becomes the next bottom; recycle the rest.
		bottom = slices[L]
		for t := 0; t < L; t++ {
			putBuf(slices[t])
		}
	}
}

// extractLayer triangulates the single cell layer at grid-z index k, whose
// bottom (z=k) and top (z=k+1) samples are lo and hi, and appends the resulting
// triangles to out. Iterates j then i to match the canonical grid order.
func extractLayer(field *Field, box AABB, nx, ny, k int, dx, dy, dz, gradEps float64, lo, hi []float64, out *[]Triangle) {
	x0base := box.Min[0]
	z0 := box.Min[2] + float64(k)*dz
	for j := 0; j < ny-1; j++ {
		row0 := j * nx
		row1 := (j + 1) * nx
		y0 := box.Min[1] + float64(j)*dy
		for i := 0; i < nx-1; i++ {
			v := [8]float64{
				lo[row0+i], lo[row0+i+1], hi[row0+i+1], hi[row0+i],
				lo[row1+i], lo[row1+i+1], hi[row1+i+1], hi[row1+i],
			}
			ci := 0
			for n := 0; n < 8; n++ {
				if v[n] < 0 {
					ci |= 1 << n
				}
			}
			ec := mcEdgeTable[ci]
			if ec == 0 {
				continue
			}
			x0 := x0base + float64(i)*dx
			p := [8]Vec3{
				V(x0, y0, z0),
				V(x0+dx, y0, z0),
				V(x0+dx, y0, z0+dz),
				V(x0, y0, z0+dz),
				V(x0, y0+dy, z0),
				V(x0+dx, y0+dy, z0),
				V(x0+dx, y0+dy, z0+dz),
				V(x0, y0+dy, z0+dz),
			}
			var verts [12]Vec3
			for e := 0; e < 12; e++ {
				if ec&(1<<e) == 0 {
					continue
				}
				a := mcEdges[e][0]
				b := mcEdges[e][1]
				va := v[a]
				vb := v[b]
				t := 0.5
				if vb != va {
					t = -va / (vb - va)
					if t < 0 {
						t = 0
					} else if t > 1 {
						t = 1
					}
				}
				verts[e] = Lerp(p[a], p[b], t)
			}
			row := mcTriTable[ci]
			for t := 0; row[t] != -1; t += 3 {
				va := verts[row[t]]
				vb := verts[row[t+1]]
				vc := verts[row[t+2]]
				na := field.Gradient(va, gradEps)
				nb := field.Gradient(vb, gradEps)
				nc := field.Gradient(vc, gradEps)
				*out = append(*out, Triangle{
					A: va, B: vb, C: vc,
					NA: na, NB: nb, NC: nc,
				})
			}
		}
	}
}

func minF(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func maxI(a, b int) int {
	if a > b {
		return a
	}
	return b
}
