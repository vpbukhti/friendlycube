package engine

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"
	"os"
)

// WriteBinarySTL flattens the scene to world-space triangles and writes a
// binary STL: 80-byte header, uint32 triangle count, then per-triangle:
// 3 normal floats, 9 vertex floats, uint16 attribute (zero).
func WriteBinarySTL(path string, root *Group) error {
	batches := root.Flatten()
	totalTris := 0
	for _, b := range batches {
		totalTris += len(b.Tris)
	}
	buf := bytes.NewBuffer(make([]byte, 0, 84+totalTris*50))
	// 80-byte header (anything non-"solid" works; some viewers are picky if
	// it starts with "solid", which would imply ASCII).
	hdr := make([]byte, 80)
	copy(hdr, []byte(fmt.Sprintf("marble-handshake-lattice tris=%d", totalTris)))
	buf.Write(hdr)
	_ = binary.Write(buf, binary.LittleEndian, uint32(totalTris))
	for _, b := range batches {
		for _, t := range b.Tris {
			// Face normal from vertex order (right-hand rule). Falls back to
			// the average vertex normal if degenerate.
			e1 := t.B.Sub(t.A)
			e2 := t.C.Sub(t.A)
			n := e1.Cross(e2)
			ln := n.Len()
			var nx, ny, nz float64
			if ln < 1e-12 {
				avg := t.NA.Add(t.NB).Add(t.NC).Mul(1.0 / 3.0).Normalize()
				nx, ny, nz = avg[0], avg[1], avg[2]
			} else {
				nx, ny, nz = n[0]/ln, n[1]/ln, n[2]/ln
			}
			tri := [12]float32{
				float32(nx), float32(ny), float32(nz),
				float32(t.A[0]), float32(t.A[1]), float32(t.A[2]),
				float32(t.B[0]), float32(t.B[1]), float32(t.B[2]),
				float32(t.C[0]), float32(t.C[1]), float32(t.C[2]),
			}
			// Guard against NaN/inf — some slicers refuse files with non-finite
			// values, so substitute zero (any degenerate triangle is harmless
			// to the slicer's merge step but a NaN spreads).
			for i := range tri {
				if math.IsNaN(float64(tri[i])) || math.IsInf(float64(tri[i]), 0) {
					tri[i] = 0
				}
			}
			_ = binary.Write(buf, binary.LittleEndian, tri)
			_ = binary.Write(buf, binary.LittleEndian, uint16(0))
		}
	}
	return os.WriteFile(path, buf.Bytes(), 0644)
}
