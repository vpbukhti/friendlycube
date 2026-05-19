package main

import (
	"runtime"
	"sync"
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
// Both phases (grid sampling and cell iteration) are parallelized over Z
// slices using GOMAXPROCS workers. The field's BVH is built before the
// parallel section — workers share it read-only.

var mcEdges = [12][2]int{
	{0, 1}, {1, 2}, {2, 3}, {3, 0},
	{4, 5}, {5, 6}, {6, 7}, {7, 4},
	{0, 4}, {1, 5}, {2, 6}, {3, 7},
}

func MarchingCubes(field *Field, box AABB, res int) []Triangle {
	if res < 2 {
		res = 2
	}
	field.Build()
	nx, ny, nz := res, res, res
	grid := make([]float64, nx*ny*nz)
	dx := (box.Max[0] - box.Min[0]) / float64(nx-1)
	dy := (box.Max[1] - box.Min[1]) / float64(ny-1)
	dz := (box.Max[2] - box.Min[2]) / float64(nz-1)
	idx := func(i, j, k int) int { return (k*ny+j)*nx + i }

	nWorkers := runtime.GOMAXPROCS(0)
	if nWorkers > nz {
		nWorkers = nz
	}
	if nWorkers < 1 {
		nWorkers = 1
	}

	// Phase 1: parallel grid sampling. Disjoint Z slices per worker.
	{
		var wg sync.WaitGroup
		chunk := (nz + nWorkers - 1) / nWorkers
		for w := 0; w < nWorkers; w++ {
			kStart := w * chunk
			kEnd := kStart + chunk
			if kEnd > nz {
				kEnd = nz
			}
			if kStart >= nz {
				break
			}
			wg.Add(1)
			go func(kStart, kEnd int) {
				defer wg.Done()
				for k := kStart; k < kEnd; k++ {
					z := box.Min[2] + float64(k)*dz
					for j := 0; j < ny; j++ {
						y := box.Min[1] + float64(j)*dy
						for i := 0; i < nx; i++ {
							x := box.Min[0] + float64(i)*dx
							grid[idx(i, j, k)] = field.Eval(V(x, y, z))
						}
					}
				}
			}(kStart, kEnd)
		}
		wg.Wait()
	}

	gradEps := 0.5 * minF(minF(dx, dy), dz)

	// Phase 2: parallel cell iteration. Each worker emits its own slice;
	// we concatenate in worker-index order so STL bytes are deterministic
	// across runs of the same seed.
	cellNz := nz - 1
	chunk := (cellNz + nWorkers - 1) / nWorkers
	perWorker := make([][]Triangle, nWorkers)
	var wg sync.WaitGroup
	for w := 0; w < nWorkers; w++ {
		kStart := w * chunk
		kEnd := kStart + chunk
		if kEnd > cellNz {
			kEnd = cellNz
		}
		if kStart >= cellNz {
			continue
		}
		wg.Add(1)
		go func(wid, kStart, kEnd int) {
			defer wg.Done()
			local := make([]Triangle, 0, 1024)
			for k := kStart; k < kEnd; k++ {
				for j := 0; j < ny-1; j++ {
					for i := 0; i < nx-1; i++ {
						v := [8]float64{
							grid[idx(i, j, k)],
							grid[idx(i+1, j, k)],
							grid[idx(i+1, j, k+1)],
							grid[idx(i, j, k+1)],
							grid[idx(i, j+1, k)],
							grid[idx(i+1, j+1, k)],
							grid[idx(i+1, j+1, k+1)],
							grid[idx(i, j+1, k+1)],
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
						x0 := box.Min[0] + float64(i)*dx
						y0 := box.Min[1] + float64(j)*dy
						z0 := box.Min[2] + float64(k)*dz
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
							local = append(local, Triangle{
								A: va, B: vb, C: vc,
								NA: na, NB: nb, NC: nc,
							})
						}
					}
				}
			}
			perWorker[wid] = local
		}(w, kStart, kEnd)
	}
	wg.Wait()

	total := 0
	for _, s := range perWorker {
		total += len(s)
	}
	tris := make([]Triangle, 0, total)
	for _, s := range perWorker {
		tris = append(tris, s...)
	}
	return tris
}

func minF(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}
