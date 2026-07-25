package engine

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
)

// stlHeader writes the 80-byte header + uint32 triangle count. Any header not
// starting with "solid" works; some viewers treat a "solid" prefix as ASCII.
func stlHeader(w io.Writer, totalTris uint32) error {
	var hdr [84]byte
	copy(hdr[:80], fmt.Sprintf("marble-handshake-lattice tris=%d", totalTris))
	binary.LittleEndian.PutUint32(hdr[80:], totalTris)
	_, err := w.Write(hdr[:])
	return err
}

// encodeTriangleSTL packs one triangle into a 50-byte binary-STL record: face
// normal (3 float32), the three vertices (9 float32), and a zero attribute
// uint16. dst must be at least 50 bytes.
func encodeTriangleSTL(t Triangle, dst []byte) {
	// Face normal from vertex order (right-hand rule). Falls back to the average
	// vertex normal if degenerate.
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
	vals := [12]float32{
		float32(nx), float32(ny), float32(nz),
		float32(t.A[0]), float32(t.A[1]), float32(t.A[2]),
		float32(t.B[0]), float32(t.B[1]), float32(t.B[2]),
		float32(t.C[0]), float32(t.C[1]), float32(t.C[2]),
	}
	for i, v := range vals {
		// Guard against NaN/inf — some slicers refuse files with non-finite
		// values, so substitute zero (a degenerate triangle is harmless to the
		// slicer's merge step, but a NaN spreads).
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) {
			v = 0
		}
		binary.LittleEndian.PutUint32(dst[i*4:], math.Float32bits(v))
	}
	dst[48], dst[49] = 0, 0 // attribute byte count
}

// WriteBinarySTL flattens the scene to world-space triangles and streams a
// binary STL straight to disk (through a buffered writer), so the whole file is
// never held in memory — only the triangle slices already in the scene.
func WriteBinarySTL(path string, root *Group) error {
	batches := root.Flatten()
	totalTris := 0
	for _, b := range batches {
		totalTris += len(b.Tris)
	}

	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		return err
	}
	w := bufio.NewWriterSize(f, 1<<20)
	writeErr := func() error {
		if err := stlHeader(w, uint32(totalTris)); err != nil {
			return err
		}
		var rec [50]byte
		for _, b := range batches {
			for _, t := range b.Tris {
				encodeTriangleSTL(t, rec[:])
				if _, err := w.Write(rec[:]); err != nil {
					return err
				}
			}
		}
		return w.Flush()
	}()
	if cerr := f.Close(); writeErr == nil {
		writeErr = cerr
	}
	return writeErr
}

// stlSink streams triangles to an STL file when the total count isn't known in
// advance: it writes a placeholder count, counts triangles as they stream, and
// patches the real count back into the header on Close.
type stlSink struct {
	f     *os.File
	w     *bufio.Writer
	rec   [50]byte
	count uint32
	err   error
}

func newSTLSink(path string) (*stlSink, error) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		return nil, err
	}
	s := &stlSink{f: f, w: bufio.NewWriterSize(f, 1<<20)}
	s.err = stlHeader(s.w, 0) // placeholder; patched in Close
	return s, s.err
}

func (s *stlSink) writeTris(tris []Triangle) {
	if s.err != nil {
		return
	}
	for _, t := range tris {
		encodeTriangleSTL(t, s.rec[:])
		if _, err := s.w.Write(s.rec[:]); err != nil {
			s.err = err
			return
		}
		s.count++
	}
}

// Close flushes, rewrites the header with the now-known triangle count (both the
// human-readable string and the binary count), and closes the file.
func (s *stlSink) Close() error {
	if s.err == nil {
		s.err = s.w.Flush()
	}
	if s.err == nil {
		var hdr [84]byte
		copy(hdr[:80], fmt.Sprintf("marble-handshake-lattice tris=%d", s.count))
		binary.LittleEndian.PutUint32(hdr[80:], s.count)
		_, s.err = s.f.WriteAt(hdr[:], 0)
	}
	if cerr := s.f.Close(); s.err == nil {
		s.err = cerr
	}
	return s.err
}
