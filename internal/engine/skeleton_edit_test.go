package engine

import "testing"

// A shared vertex moves every attached strut endpoint; a per-endpoint move
// detaches only that one.
func TestSharedVertexMoveAndDetach(t *testing.T) {
	sk := &Skeleton{}
	// two struts meeting at an interior point p (shared), other ends elsewhere.
	p := V(0.2, 0.1, -0.1)
	sk.Struts = []SkelStrut{
		{ID: 0, A: p, B: V(0.5, 0.5, 0.5), Kind: StrutThick},
		{ID: 1, A: p, B: V(-0.5, 0.4, 0.3), Kind: StrutThick},
	}
	sk.NextStrutID = 2
	sk.buildSharedVertices()
	if sk.Struts[0].AVert < 0 || sk.Struts[0].AVert != sk.Struts[1].AVert {
		t.Fatalf("expected a shared vertex, got %d and %d", sk.Struts[0].AVert, sk.Struts[1].AVert)
	}
	id := sk.Struts[0].AVert

	// Move the shared vertex: both A endpoints follow.
	sk.MoveVertex(id, V(0.3, 0.3, 0.3))
	if !sk.Struts[0].A.Equals(V(0.3, 0.3, 0.3)) || !sk.Struts[1].A.Equals(V(0.3, 0.3, 0.3)) {
		t.Errorf("shared move didn't move both: %v %v", sk.Struts[0].A, sk.Struts[1].A)
	}

	// Detach strut 0's A endpoint: strut 1 must stay put.
	sk.MoveStrutEndpoint(0, "a", V(0.1, 0.0, 0.0))
	if sk.Struts[0].AVert == sk.Struts[1].AVert {
		t.Errorf("endpoint move should have detached the shared vertex")
	}
	if !sk.Struts[1].A.Equals(V(0.3, 0.3, 0.3)) {
		t.Errorf("detaching strut 0 disturbed strut 1: %v", sk.Struts[1].A)
	}
}

// DeleteStrut drops the strut + its handshake and compacts the vertex table.
func TestDeleteStrutCompacts(t *testing.T) {
	sk := &Skeleton{}
	sk.Struts = []SkelStrut{
		{ID: 0, A: V(0.1, 0.1, 0.1), B: V(0.2, 0.2, 0.2), Kind: StrutThick},
		{ID: 1, A: V(-0.1, 0.1, 0.1), B: V(-0.2, 0.2, 0.2), Kind: StrutThick},
	}
	sk.Handshakes = []SkelHandshake{{StrutID: 0}, {StrutID: 1}}
	sk.buildSharedVertices()
	before := len(sk.Vertices)
	sk.DeleteStrut(0)
	if len(sk.Struts) != 1 || sk.Struts[0].ID != 1 {
		t.Fatalf("strut not removed: %+v", sk.Struts)
	}
	if len(sk.Handshakes) != 1 || sk.Handshakes[0].StrutID != 1 {
		t.Errorf("handshake not removed: %+v", sk.Handshakes)
	}
	if len(sk.Vertices) >= before {
		t.Errorf("vertices not compacted: before=%d after=%d", before, len(sk.Vertices))
	}
	// remaining strut's refs still resolve to its endpoints
	if !sk.EndA(sk.Struts[0]).Equals(V(-0.1, 0.1, 0.1)) {
		t.Errorf("remap broke endpoint: %v", sk.EndA(sk.Struts[0]))
	}
}

// SnapEditPoint keeps a dragged point inside the cube and snaps onto a nearby
// cube corner.
func TestSnapEditContainmentAndCorner(t *testing.T) {
	sk := &Skeleton{}
	s := DefaultSettings() // cube half = 1
	// way outside → clamped inside
	got := sk.SnapEditPoint(s, V(5, 5, 5), -1, -1, false)
	half := s.CubeSize / 2
	for i := range 3 {
		if got[i] > half+1e-9 || got[i] < -half-1e-9 {
			t.Fatalf("not clamped inside cube: %v", got)
		}
	}
	// near a corner → snaps exactly to it
	corner := V(half, half, half)
	got = sk.SnapEditPoint(s, V(half-0.02, half-0.02, half-0.02), -1, -1, false)
	if !got.Equals(corner) {
		t.Errorf("expected snap to corner %v, got %v", corner, got)
	}
}
