package main

// Marching cubes. Standard Lorensen-Cline tables (Paul Bourke's well-known
// public-domain transcription). 256 entries; each row in mcTriTable lists
// triangles as vertex-edge indices (terminated by -1). mcEdgeTable[i] tells
// which of the 12 cube edges are crossed by the isosurface for case i.
//
// Grid convention used here: 8 cube corners, indexed 0..7 with v0..v3 on
// the bottom (y=ymin) and v4..v7 on the top (y=ymax). Edges 0..11 connect
// the corners in the order matching the tables below.

func MarchingCubes(field *Field, box AABB, res int) []Triangle {
	if res < 2 {
		res = 2
	}
	// Pre-sample the scalar field on the full grid. Allocate once.
	nx, ny, nz := res, res, res
	grid := make([]float64, nx*ny*nz)
	dx := (box.Max[0] - box.Min[0]) / float64(nx-1)
	dy := (box.Max[1] - box.Min[1]) / float64(ny-1)
	dz := (box.Max[2] - box.Min[2]) / float64(nz-1)
	idx := func(i, j, k int) int { return (k*ny+j)*nx + i }
	for k := 0; k < nz; k++ {
		z := box.Min[2] + float64(k)*dz
		for j := 0; j < ny; j++ {
			y := box.Min[1] + float64(j)*dy
			for i := 0; i < nx; i++ {
				x := box.Min[0] + float64(i)*dx
				grid[idx(i, j, k)] = field.Eval(V(x, y, z))
			}
		}
	}
	gradEps := 0.5 * minF(minF(dx, dy), dz)
	tris := []Triangle{}
	// Per-cell, classify the 8 corner values into a case index (0..255).
	for k := 0; k < nz-1; k++ {
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
				// Vertex along each crossed edge — linear interpolation by
				// the field values at the endpoints.
				var verts [12]Vec3
				edges := [12][2]int{
					{0, 1}, {1, 2}, {2, 3}, {3, 0},
					{4, 5}, {5, 6}, {6, 7}, {7, 4},
					{0, 4}, {1, 5}, {2, 6}, {3, 7},
				}
				for e := 0; e < 12; e++ {
					if ec&(1<<e) == 0 {
						continue
					}
					a := edges[e][0]
					b := edges[e][1]
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
				// Emit triangles per the table.
				row := mcTriTable[ci]
				for t := 0; row[t] != -1; t += 3 {
					va := verts[row[t]]
					vb := verts[row[t+1]]
					vc := verts[row[t+2]]
					na := field.Gradient(va, gradEps)
					nb := field.Gradient(vb, gradEps)
					nc := field.Gradient(vc, gradEps)
					tris = append(tris, Triangle{
						A: va, B: vb, C: vc,
						NA: na, NB: nb, NC: nc,
					})
				}
			}
		}
	}
	return tris
}

func minF(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}
