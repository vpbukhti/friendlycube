package engine

import "math"

// Skeleton is the explicit, serializable, editable description of the frame:
// the movable struts (thick or thin), the thin support web, and the handshake
// records. The cube's 12 edges + 8 corners are NOT stored here — they are the
// fixed frame, rebuilt from the topology on every build and never editable.
//
// It replaces the old "regenerate everything from the seed on every rebuild"
// model. BakeSkeleton runs the procedural generator once to produce a Skeleton;
// after that the Skeleton (possibly hand-edited) is the source of truth for both
// the debug renderer and the implicit-skin mesher.
type Skeleton struct {
	// Vertices are the shared, movable interior joints. A strut endpoint with
	// AVert/BVert >= 0 is anchored to Vertices[idx]; moving that vertex moves
	// every endpoint referencing it. Endpoints on a cube corner/edge are pinned
	// to the fixed frame and carry idx == -1 (not listed here).
	Vertices    []Vec3          `json:"vertices"`
	Struts      []SkelStrut     `json:"struts"`
	Supports    []SkelSupport   `json:"supports"`
	Handshakes  []SkelHandshake `json:"handshakes"`
	NextStrutID int             `json:"nextStrutID"`
}

// Strut kinds. Thick = StrutR by default; thin = ButtressR by default.
const (
	StrutThick = "thick"
	StrutThin  = "thin"
)

// SkelStrut is one movable strut. A/B are the resolved world endpoints (kept in
// sync with the vertex table); AVert/BVert give shared-vertex identity.
type SkelStrut struct {
	ID       int     `json:"id"`
	A        Vec3    `json:"a"`
	B        Vec3    `json:"b"`
	AVert    int     `json:"aVert"` // shared-vertex index, or -1 if frame-pinned
	BVert    int     `json:"bVert"`
	Kind     string  `json:"kind"`    // "thick" | "thin"
	Width    float64 `json:"width"`   // explicit radius; 0 = kind default
	Lf       float64 `json:"lf"`      // length factor (debug coloring)
	KindGen  string  `json:"kindGen"` // "random" | "edge" (debug coloring)
	SnappedA bool    `json:"snappedA"`
	SnappedB bool    `json:"snappedB"`
}

// SkelSupport is one thin support-web segment. The planning metadata is kept so
// Phase 2 can regenerate supports around a moved strut with the same algorithm.
type SkelSupport struct {
	ID        int     `json:"id"`
	P1        Vec3    `json:"p1"`
	P2        Vec3    `json:"p2"`
	P1Vert    int     `json:"p1Vert"` // shared-vertex index, or -1 if frame-pinned
	P2Vert    int     `json:"p2Vert"`
	Kind      string  `json:"kind"`     // "pyramid" | "webbing"
	BeadKind  string  `json:"beadKind"` // "inner" | "support" | "wire" | "landing"
	AnchorIdx int     `json:"anchorIdx"`
	ApexSkip  float64 `json:"apexSkip"`
	PadR      float64 `json:"padR"`
}

// SkelHandshake records a handshake at a strut's midpoint. Struts themselves are
// always full, watertight capsules now; the handshake is a separate overlay mesh
// rebuilt deterministically from these fields (no RNG at render time). It is
// flagged Undesirable when the strut is too short for it or another strut sits
// where the handshake would go.
type SkelHandshake struct {
	StrutID     int     `json:"strutID"`
	Mid         Vec3    `json:"mid"`
	Undesirable bool    `json:"undesirable"`
	Reason      string  `json:"reason"` // "" | "tooShort" | "occupied"
	OffsetFrac  float64 `json:"offsetFrac"`
	JitA        float64 `json:"jitA"`
	JitB        float64 `json:"jitB"`
	Grip        string  `json:"grip"`
}

// ---- endpoint resolution ----

// EndA / EndB return the resolved world position of a strut endpoint (through
// the vertex table when shared, else the stored point).
func (sk *Skeleton) EndA(st SkelStrut) Vec3 {
	if st.AVert >= 0 && st.AVert < len(sk.Vertices) {
		return sk.Vertices[st.AVert]
	}
	return st.A
}

func (sk *Skeleton) EndB(st SkelStrut) Vec3 {
	if st.BVert >= 0 && st.BVert < len(sk.Vertices) {
		return sk.Vertices[st.BVert]
	}
	return st.B
}

// EndP1 / EndP2 resolve a support segment's endpoints through the vertex table
// (so a support's joints move with any thick strut / joint they share).
func (sk *Skeleton) EndP1(ss SkelSupport) Vec3 {
	if ss.P1Vert >= 0 && ss.P1Vert < len(sk.Vertices) {
		return sk.Vertices[ss.P1Vert]
	}
	return ss.P1
}

func (sk *Skeleton) EndP2(ss SkelSupport) Vec3 {
	if ss.P2Vert >= 0 && ss.P2Vert < len(sk.Vertices) {
		return sk.Vertices[ss.P2Vert]
	}
	return ss.P2
}

// EffRadius resolves a strut's tube radius: explicit Width if set, else the
// kind default from Settings.
func (st SkelStrut) EffRadius(s Settings) float64 {
	if st.Width > 0 {
		return st.Width
	}
	if st.Kind == StrutThin {
		return s.ButtressR
	}
	return s.StrutR
}

func (sk *Skeleton) strutByID(id int) *SkelStrut {
	for i := range sk.Struts {
		if sk.Struts[i].ID == id {
			return &sk.Struts[i]
		}
	}
	return nil
}

func (sk *Skeleton) supportByID(id int) *SkelSupport {
	for i := range sk.Supports {
		if sk.Supports[i].ID == id {
			return &sk.Supports[i]
		}
	}
	return nil
}

// ---- mutation ----

func (sk *Skeleton) addVertex(p Vec3) int {
	sk.Vertices = append(sk.Vertices, p)
	return len(sk.Vertices) - 1
}

func (sk *Skeleton) refCount(idx int) int {
	n := 0
	for _, st := range sk.Struts {
		if st.AVert == idx {
			n++
		}
		if st.BVert == idx {
			n++
		}
	}
	for _, ss := range sk.Supports {
		if ss.P1Vert == idx {
			n++
		}
		if ss.P2Vert == idx {
			n++
		}
	}
	return n
}

// MoveVertex repositions a shared vertex; every strut AND support endpoint
// anchored to it follows. Callers validate `to` (snap/containment).
func (sk *Skeleton) MoveVertex(id int, to Vec3) {
	if id < 0 || id >= len(sk.Vertices) {
		return
	}
	sk.Vertices[id] = to
	for i := range sk.Struts {
		if sk.Struts[i].AVert == id {
			sk.Struts[i].A = to
		}
		if sk.Struts[i].BVert == id {
			sk.Struts[i].B = to
		}
	}
	for i := range sk.Supports {
		if sk.Supports[i].P1Vert == id {
			sk.Supports[i].P1 = to
		}
		if sk.Supports[i].P2Vert == id {
			sk.Supports[i].P2 = to
		}
	}
}

// MoveStrutEndpoint moves only one strut's endpoint, detaching it from any
// shared vertex it currently participates in. `which` is "a" or "b".
func (sk *Skeleton) MoveStrutEndpoint(strutID int, which string, to Vec3) {
	st := sk.strutByID(strutID)
	if st == nil {
		return
	}
	ref := &st.AVert
	pos := &st.A
	if which == "b" {
		ref = &st.BVert
		pos = &st.B
	}
	switch {
	case *ref < 0:
		// Was frame-pinned; becomes a standalone movable vertex.
		*ref = sk.addVertex(to)
	case sk.refCount(*ref) > 1:
		// Shared with others; split off a fresh vertex for just this endpoint.
		*ref = sk.addVertex(to)
	default:
		// Solely-owned vertex; just move it in place.
		sk.Vertices[*ref] = to
	}
	*pos = to
}

// EditMoveVertex snaps `to` then moves the shared vertex there. On commit (the
// final move of a drag), if the snap landed on another existing joint the two
// are MERGED into one shared vertex (so it re-shares). During a live drag
// (commit=false) it only repositions — no identity change — so vertex indices
// stay stable for the rest of the drag.
func (sk *Skeleton) EditMoveVertex(s Settings, id int, to Vec3, commit bool) {
	snap := sk.SnapEdit(s, to, id, -1, false)
	sk.MoveVertex(id, snap.Point)
	if commit && snap.VertID >= 0 && snap.VertID != id {
		sk.mergeVertex(id, snap.VertID)
	}
}

// EditMoveEndpoint moves one strut's endpoint (detaching it from a shared
// vertex). On commit, if the snap landed on an existing joint the endpoint
// re-attaches to THAT joint (re-sharing) instead of keeping its own vertex.
func (sk *Skeleton) EditMoveEndpoint(s Settings, strutID int, which string, to Vec3, commit bool) {
	st := sk.strutByID(strutID)
	if st == nil {
		return
	}
	thin := st.Kind == StrutThin
	exV := st.AVert
	if which == "b" {
		exV = st.BVert
	}
	snap := sk.SnapEdit(s, to, exV, strutID, thin)
	if commit && snap.VertID >= 0 {
		if which == "b" {
			st.BVert, st.B = snap.VertID, sk.Vertices[snap.VertID]
		} else {
			st.AVert, st.A = snap.VertID, sk.Vertices[snap.VertID]
		}
		sk.compactVertices() // drop the endpoint's now-orphaned own vertex, if any
		return
	}
	sk.MoveStrutEndpoint(strutID, which, snap.Point)
}

// MoveSupportEndpoint moves one support endpoint ("p1"/"p2"), detaching it from
// any shared vertex it participates in (the counterpart of MoveStrutEndpoint).
func (sk *Skeleton) MoveSupportEndpoint(supportID int, which string, to Vec3) {
	ss := sk.supportByID(supportID)
	if ss == nil {
		return
	}
	ref := &ss.P1Vert
	pos := &ss.P1
	if which == "p2" {
		ref = &ss.P2Vert
		pos = &ss.P2
	}
	switch {
	case *ref < 0:
		*ref = sk.addVertex(to)
	case sk.refCount(*ref) > 1:
		*ref = sk.addVertex(to)
	default:
		sk.Vertices[*ref] = to
	}
	*pos = to
}

// EditMoveSupportEndpoint is the support counterpart of EditMoveEndpoint: it
// moves just one support's endpoint, and on commit re-shares onto an existing
// joint if the snap landed on one.
func (sk *Skeleton) EditMoveSupportEndpoint(s Settings, supportID int, which string, to Vec3, commit bool) {
	ss := sk.supportByID(supportID)
	if ss == nil {
		return
	}
	exV := ss.P1Vert
	if which == "p2" {
		exV = ss.P2Vert
	}
	snap := sk.SnapEdit(s, to, exV, -1, false)
	if commit && snap.VertID >= 0 {
		if which == "p2" {
			ss.P2Vert, ss.P2 = snap.VertID, sk.Vertices[snap.VertID]
		} else {
			ss.P1Vert, ss.P1 = snap.VertID, sk.Vertices[snap.VertID]
		}
		sk.compactVertices()
		return
	}
	sk.MoveSupportEndpoint(supportID, which, snap.Point)
}

// mergeVertex re-points every endpoint on `from` to `into`, then compacts the
// (now unreferenced) `from` out of the table.
func (sk *Skeleton) mergeVertex(from, into int) {
	if from < 0 || into < 0 || from == into {
		return
	}
	p := sk.Vertices[into]
	for i := range sk.Struts {
		if sk.Struts[i].AVert == from {
			sk.Struts[i].AVert, sk.Struts[i].A = into, p
		}
		if sk.Struts[i].BVert == from {
			sk.Struts[i].BVert, sk.Struts[i].B = into, p
		}
	}
	for i := range sk.Supports {
		if sk.Supports[i].P1Vert == from {
			sk.Supports[i].P1Vert, sk.Supports[i].P1 = into, p
		}
		if sk.Supports[i].P2Vert == from {
			sk.Supports[i].P2Vert, sk.Supports[i].P2 = into, p
		}
	}
	sk.compactVertices()
}

// AddStrut creates a new strut of the given kind ("thick"/"thin") between two
// snapped points, using the kind's default width. Endpoints snap to existing
// features (joints/corners/edges/thick struts) per the same rules as moving.
// Returns the new strut ID, or -1 if the span is degenerate.
func (sk *Skeleton) AddStrut(s Settings, a, b Vec3, kind string) int {
	if kind != StrutThin {
		kind = StrutThick
	}
	thin := kind == StrutThin
	pa := sk.SnapEdit(s, a, -1, -1, thin).Point
	pb := sk.SnapEdit(s, b, -1, -1, thin).Point
	if pa.DistTo(pb) < 0.05*s.CubeSize {
		return -1
	}
	id := sk.NextStrutID
	sk.NextStrutID++
	sk.Struts = append(sk.Struts, SkelStrut{
		ID: id, A: pa, B: pb, Kind: kind, Width: 0, Lf: 1.0, KindGen: "random",
	})
	sk.buildSharedVertices() // re-index so the new endpoints share existing joints
	sk.RecomputeHandshakeFlags(s)
	return id
}

// SetStrutWidth sets an explicit tube radius (0 = revert to kind default).
func (sk *Skeleton) SetStrutWidth(id int, w float64) {
	if st := sk.strutByID(id); st != nil {
		st.Width = w
	}
}

// DeleteStrut removes a strut and its handshake, then compacts the vertex table
// (the client refetches the whole skeleton after a structural edit, so vertex
// index churn is safe).
func (sk *Skeleton) DeleteStrut(id int) {
	out := sk.Struts[:0]
	for _, st := range sk.Struts {
		if st.ID != id {
			out = append(out, st)
		}
	}
	sk.Struts = out
	hs := sk.Handshakes[:0]
	for _, h := range sk.Handshakes {
		if h.StrutID != id {
			hs = append(hs, h)
		}
	}
	sk.Handshakes = hs
	sk.compactVertices()
}

// DeleteSupport removes a thin support-web segment by ID, then compacts the
// vertex table (a support may have owned a joint no longer referenced).
func (sk *Skeleton) DeleteSupport(id int) {
	out := sk.Supports[:0]
	for _, ss := range sk.Supports {
		if ss.ID != id {
			out = append(out, ss)
		}
	}
	sk.Supports = out
	sk.compactVertices()
}

// compactVertices drops unreferenced vertices and remaps the endpoint indices.
func (sk *Skeleton) compactVertices() {
	used := map[int]int{}
	var kept []Vec3
	remap := func(idx int) int {
		if idx < 0 {
			return -1
		}
		if n, ok := used[idx]; ok {
			return n
		}
		n := len(kept)
		kept = append(kept, sk.Vertices[idx])
		used[idx] = n
		return n
	}
	for i := range sk.Struts {
		sk.Struts[i].AVert = remap(sk.Struts[i].AVert)
		sk.Struts[i].BVert = remap(sk.Struts[i].BVert)
	}
	for i := range sk.Supports {
		sk.Supports[i].P1Vert = remap(sk.Supports[i].P1Vert)
		sk.Supports[i].P2Vert = remap(sk.Supports[i].P2Vert)
	}
	sk.Vertices = kept
}

// Clone deep-copies the skeleton (the build worker clones under lock before
// meshing so a concurrent edit can't race the marching cubes pass).
func (sk *Skeleton) Clone() *Skeleton {
	if sk == nil {
		return nil
	}
	c := &Skeleton{NextStrutID: sk.NextStrutID}
	c.Vertices = append([]Vec3(nil), sk.Vertices...)
	c.Struts = append([]SkelStrut(nil), sk.Struts...)
	c.Supports = append([]SkelSupport(nil), sk.Supports...)
	c.Handshakes = append([]SkelHandshake(nil), sk.Handshakes...)
	return c
}

// ---- shared-vertex identity ----

// buildSharedVertices (re)builds the vertex table from every strut AND support
// endpoint. EVERY connection point is a real, grabbable vertex — including where
// a strut or support meets a cube edge or corner. The cube's 12 edges + 8
// corners remain the fixed frame (always rendered, and snap targets), but a
// connection ON the frame is still a proper joint you can select, share, and
// slide. Endpoints are merged only on EXACT coincidence (generation snaps to
// exact endpoints), so no position is ever nudged — keeping bake output
// bit-identical to the pre-vertex-table pipeline.
func (sk *Skeleton) buildSharedVertices() {
	sk.Vertices = nil
	find := func(p Vec3) int {
		for i, v := range sk.Vertices {
			if v.Equals(p) {
				return i
			}
		}
		return sk.addVertex(p)
	}
	for i := range sk.Struts {
		st := &sk.Struts[i]
		st.AVert = find(st.A)
		st.A = sk.Vertices[st.AVert]
		st.BVert = find(st.B)
		st.B = sk.Vertices[st.BVert]
	}
	for i := range sk.Supports {
		ss := &sk.Supports[i]
		ss.P1Vert = find(ss.P1)
		ss.P1 = sk.Vertices[ss.P1Vert]
		ss.P2Vert = find(ss.P2)
		ss.P2 = sk.Vertices[ss.P2Vert]
	}
}

// ---- handshake flagging ----

// handshakeGeom recomputes the handshake meeting point + arm reach from the
// strut's CURRENT endpoints and the stored offset fraction (so it follows moves).
func (sk *Skeleton) handshakeGeom(h SkelHandshake, st SkelStrut) (meet Vec3, armPortion float64, ok bool) {
	a, b := sk.EndA(st), sk.EndB(st)
	length := a.DistTo(b)
	if length < 1e-6 {
		return Vec3{}, 0, false
	}
	dirAB := b.Sub(a).Normalize()
	mid := a.Add(b).Mul(0.5)
	meet = mid.Add(dirAB.Mul(h.OffsetFrac * length))
	distA := a.DistTo(meet)
	distB := b.DistTo(meet)
	armPortion = math.Min(0.5, math.Min(distA*0.55, distB*0.55))
	return meet, armPortion, true
}

// RecomputeHandshakeFlags marks handshakes undesirable when the strut is too
// short for a handshake or another strut/vertex sits where the handshake would.
func (sk *Skeleton) RecomputeHandshakeFlags(s Settings) {
	minArm := 3.0 * s.StrutR  // below this the arms have no room
	occupyR := 2.0 * s.StrutR // another strut/joint this close = occupied
	for i := range sk.Handshakes {
		h := &sk.Handshakes[i]
		st := sk.strutByID(h.StrutID)
		if st == nil {
			h.Undesirable, h.Reason = true, "orphan"
			continue
		}
		meet, armPortion, ok := sk.handshakeGeom(*h, *st)
		if !ok || armPortion < minArm {
			h.Mid, h.Undesirable, h.Reason = meet, true, "tooShort"
			continue
		}
		occupied := false
		for _, other := range sk.Struts {
			if other.ID == st.ID {
				continue
			}
			if pointSegDist(meet, sk.EndA(other), sk.EndB(other)) < occupyR {
				occupied = true
				break
			}
		}
		if !occupied {
			for j, v := range sk.Vertices {
				// skip this strut's own shared endpoints
				if j == st.AVert || j == st.BVert {
					continue
				}
				if v.DistTo(meet) < occupyR {
					occupied = true
					break
				}
			}
		}
		h.Mid = meet
		h.Undesirable = occupied
		if occupied {
			h.Reason = "occupied"
		} else {
			h.Reason = ""
		}
	}
}

// ---- edit-time snapping / validation (authoritative; the client mirrors it) ----

// EditSnap is the result of an edit-time snap: the resolved Point, and VertID
// >= 0 when the point snapped onto an EXISTING shared joint (so the caller can
// merge/re-share). VertID is -1 for corners, edges, strut bodies, and free space
// (those are position-only snaps that keep the moved point as its own vertex).
type EditSnap struct {
	Point  Vec3
	VertID int
}

// SnapEditPoint returns just the snapped position (thin wrapper over SnapEdit).
func (sk *Skeleton) SnapEditPoint(s Settings, p Vec3, excludeVertID, excludeStrutID int, thin bool) Vec3 {
	return sk.SnapEdit(s, p, excludeVertID, excludeStrutID, thin).Point
}

// SnapEdit clamps a dragged point into the cube and snaps it to the nearest
// eligible feature. Existing joints win (a stronger, larger radius) and merge;
// cube corners reposition; cube edges / thick-strut bodies snap (slide) only
// when no joint is close. excludeVertID / excludeStrutID drop the vertex/strut
// being moved. thin restricts a thin strut's endpoint to thick struts / the
// frame (dormant in Phase 1).
func (sk *Skeleton) SnapEdit(s Settings, p Vec3, excludeVertID, excludeStrutID int, thin bool) EditSnap {
	half := s.CubeSize / 2
	p = Vec3{clampF(p[0], -half, half), clampF(p[1], -half, half), clampF(p[2], -half, half)}
	topo := BuildCubeTopology(half)

	rVert := (2.5 / 1.5) * s.StrutR
	best, bestD, bestVert, found := p, rVert, -1, false
	// Existing joints first, so snapping onto one MERGES (re-shares) — even when
	// that joint sits on a cube corner.
	if !thin { // thin endpoints don't merge onto arbitrary interior joints
		for j, v := range sk.Vertices {
			if j == excludeVertID {
				continue
			}
			if d := p.DistTo(v); d < bestD {
				best, bestD, bestVert, found = v, d, j, true
			}
		}
	}
	for _, v := range topo.Verts { // 8 fixed cube corners (reposition, not merge)
		if d := p.DistTo(v); d < bestD { // strictly closer, so a coincident joint wins
			best, bestD, bestVert, found = v, d, -1, true
		}
	}
	if found {
		return EditSnap{best, bestVert}
	}

	rEdge := (1.8 / 1.5) * s.StrutR
	best, bestD = p, rEdge
	for _, e := range topo.Edges { // 12 fixed cube edges (slide onto)
		proj := closestOnSeg(e.A, e.B, p)
		if d := p.DistTo(proj); d < bestD {
			best, bestD = proj, d
		}
	}
	for _, st := range sk.Struts { // thick strut bodies only
		if st.ID == excludeStrutID || st.Kind == StrutThin {
			continue
		}
		if st.AVert == excludeVertID || st.BVert == excludeVertID {
			continue
		}
		proj := closestOnSeg(sk.EndA(st), sk.EndB(st), p)
		if d := p.DistTo(proj); d < bestD {
			best, bestD = proj, d
		}
	}
	return EditSnap{best, -1}
}

// pointSegDist is the distance from point p to segment a→b.
func pointSegDist(p, a, b Vec3) float64 {
	ab := b.Sub(a)
	l2 := ab.Dot(ab)
	t := 0.0
	if l2 > 1e-18 {
		t = clampF(p.Sub(a).Dot(ab)/l2, 0, 1)
	}
	return p.DistTo(a.Add(ab.Mul(t)))
}
