package control

import (
	_ "embed"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"math/rand/v2"
	"net/http"
	"strconv"
	"sync"

	"friendlycube/internal/engine"
)

//go:embed panel.html
var panelHTML []byte

// App is the whole application: it owns the config (incl. the editable skeleton),
// generates geometry on a background worker, keeps the latest mesh (serialized
// for the browser), and serves the control panel + WebGL view.
type App struct {
	mu         sync.Mutex
	cfg        Config
	seed       uint32
	building   bool
	lastExport string
	configPath string

	// editor state
	editMode       bool // while true the worker renders the debug diagram
	previewSkin    bool // one-shot: build the skin once, then revert
	showHandshakes bool // composite the handshake overlay into the mesh
	skelVersion    int  // bumped on any skeleton change (client refetches)
	midDrag        bool // a live drag gesture is in progress (undo coalescing)

	// undoStack holds skeleton snapshots taken just BEFORE each mutating edit
	// (nil = "unmodified / use the generated skeleton"), newest last. Cleared
	// when the baseline changes (regen / load / discard).
	undoStack []*engine.Skeleton

	meshData    []byte // encoded mesh for /api/mesh (see encodeMesh)
	meshVersion int    // bumped after each successful rebuild
	triCount    int

	buildSignal chan struct{} // buffered(1); coalesces rebuild requests
}

// NewApp seeds the app and bakes the initial skeleton.
func NewApp(cfg Config, configPath string) *App {
	seed, ok := parseSeed(cfg.Seed)
	if !ok {
		seed = randomSeed()
	}
	cfg.Seed = fmt.Sprintf("%06x", seed)
	cfg.EnsureGeneratedSkeleton(seed)
	return &App{
		cfg:            cfg,
		seed:           seed,
		configPath:     configPath,
		showHandshakes: true,
		buildSignal:    make(chan struct{}, 1),
	}
}

// Start launches the background build worker and requests the first build.
func (a *App) Start() {
	go a.worker()
	a.signalBuild()
}

// worker turns coalesced build signals into a fresh serialized mesh. It clones
// the effective skeleton under the lock so a concurrent edit can't race the
// marching-cubes pass.
func (a *App) worker() {
	for range a.buildSignal {
		a.mu.Lock()
		a.building = true
		cfg := a.cfg
		editMode := a.editMode
		preview := a.previewSkin
		a.previewSkin = false
		showHS := a.showHandshakes
		skel := cfg.EffectiveSkeleton().Clone()
		a.mu.Unlock()

		s := cfg.ToSettings()
		mode := cfg.Mode
		if editMode {
			mode = "debug"
		}
		if preview {
			mode = "skin"
		}
		var g *engine.Group
		if mode == "debug" {
			g = engine.BuildSceneFromSkeleton(s, skel)
		} else {
			g = engine.BuildSkinFromSkeleton(s, skel, cfg.ToSkinParams(cfg.Skin.Resolution))
		}
		if showHS {
			g.Add(engine.BuildHandshakeOverlay(s, skel))
		}
		pos, nrm, col := engine.MeshBuffers(g)
		data := encodeMesh(pos, nrm, col)

		a.mu.Lock()
		a.meshData = data
		a.meshVersion++
		a.triCount = len(pos) / 9
		a.building = false
		a.mu.Unlock()
	}
}

// Regen draws a fresh random seed, drops any skeleton, re-bakes, and rebuilds.
func (a *App) Regen() string {
	a.mu.Lock()
	a.seed = randomSeed()
	a.cfg.Seed = fmt.Sprintf("%06x", a.seed)
	a.cfg.Frame.ModifiedSkeleton = nil
	a.cfg.Frame.GeneratedSkeleton = nil
	a.cfg.EnsureGeneratedSkeleton(a.seed)
	a.undoStack = nil
	a.skelVersion++
	hex := a.cfg.Seed
	a.mu.Unlock()
	a.signalBuild()
	return hex
}

// pushUndoLocked snapshots the current modified skeleton (nil if none yet) so a
// later /api/undo can restore it. Call under a.mu, BEFORE mutating. Bounded so
// a long session can't grow without limit.
func (a *App) pushUndoLocked() {
	const maxUndo = 200
	var snap *engine.Skeleton
	if a.cfg.Frame.ModifiedSkeleton != nil {
		snap = a.cfg.Frame.ModifiedSkeleton.Clone()
	}
	a.undoStack = append(a.undoStack, snap)
	if len(a.undoStack) > maxUndo {
		a.undoStack = a.undoStack[len(a.undoStack)-maxUndo:]
	}
}

func (a *App) signalBuild() {
	select {
	case a.buildSignal <- struct{}{}:
	default: // already pending — coalesce
	}
}

// Snapshot returns the current config + seed.
func (a *App) Snapshot() (Config, uint32) {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.cfg, a.seed
}

// encodeMesh packs the attribute arrays into a little-endian binary blob:
// uint32 vertexCount, then positions[3V], normals[3V], colors[3V] as float32.
func encodeMesh(pos, nrm, col []float32) []byte {
	v := uint32(len(pos) / 3)
	out := make([]byte, 4+(len(pos)+len(nrm)+len(col))*4)
	binary.LittleEndian.PutUint32(out, v)
	off := 4
	put := func(s []float32) {
		for _, f := range s {
			binary.LittleEndian.PutUint32(out[off:], math.Float32bits(f))
			off += 4
		}
	}
	put(pos)
	put(nrm)
	put(col)
	return out
}

// ---- HTTP ----

// Handler builds the panel's HTTP routes.
func (a *App) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/", a.handleIndex)
	mux.HandleFunc("/favicon.ico", func(w http.ResponseWriter, r *http.Request) { w.WriteHeader(http.StatusNoContent) })
	mux.HandleFunc("/api/schema", a.handleSchema)
	mux.HandleFunc("/api/config", a.handleConfig)
	mux.HandleFunc("/api/status", a.handleStatus)
	mux.HandleFunc("/api/mesh", a.handleMesh)
	mux.HandleFunc("/api/regen", a.handleRegen)
	mux.HandleFunc("/api/spin", a.handleSpin)
	mux.HandleFunc("/api/export", a.handleExport)
	mux.HandleFunc("/api/save", a.handleSave)
	mux.HandleFunc("/api/load", a.handleLoad)
	mux.HandleFunc("/api/skeleton", a.handleSkeleton)
	mux.HandleFunc("/api/edit", a.handleEdit)
	mux.HandleFunc("/api/undo", a.handleUndo)
	mux.HandleFunc("/api/edit-mode", a.handleEditMode)
	mux.HandleFunc("/api/preview-skin", a.handlePreviewSkin)
	return mux
}

// Serve installs the routes and blocks serving on addr.
func (a *App) Serve(addr string) error {
	return http.ListenAndServe(addr, a.Handler())
}

func writeJSON(w http.ResponseWriter, v any) {
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(v)
}

func (a *App) handleIndex(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	_, _ = w.Write(panelHTML)
}

func (a *App) handleSchema(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, Schema())
}

// clientConfig strips the (potentially large) skeletons from a config before
// sending it to the browser — the client manages params only; the skeleton is
// fetched/mutated through /api/skeleton + /api/edit.
func clientConfig(c Config) Config {
	c.Frame.GeneratedSkeleton = nil
	c.Frame.ModifiedSkeleton = nil
	return c
}

func (a *App) handleConfig(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		cfg, _ := a.Snapshot()
		writeJSON(w, clientConfig(cfg))
		return
	}
	raw, err := io.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	a.mu.Lock()
	oldCfg := a.cfg
	newCfg := a.cfg // preserves skeleton pointers (client never sends them)
	if err := json.Unmarshal(raw, &newCfg); err != nil {
		a.mu.Unlock()
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	var flags struct {
		Discard bool `json:"discardModified"`
	}
	_ = json.Unmarshal(raw, &flags)

	seedChanged := false
	if newCfg.Seed != oldCfg.Seed {
		if s, ok := parseSeed(newCfg.Seed); ok {
			a.seed = s
			newCfg.Seed = fmt.Sprintf("%06x", s)
			seedChanged = true
		} else {
			newCfg.Seed = oldCfg.Seed // reject unparsable edits
		}
	}
	structural := seedChanged || !frameScalarsEqual(oldCfg.Frame, newCfg.Frame)

	// Structural (skeleton-affecting) params are locked while the frame editor
	// is open — they must not clobber an in-progress edit session.
	if structural && a.editMode {
		a.mu.Unlock()
		writeJSON(w, map[string]any{"ok": false, "locked": true})
		return
	}

	// A structural change with unsaved edits present needs explicit confirmation.
	if structural && oldCfg.Frame.ModifiedSkeleton != nil && !flags.Discard {
		a.mu.Unlock()
		writeJSON(w, map[string]any{"ok": false, "needConfirm": true})
		return
	}
	a.cfg = newCfg
	if structural {
		if flags.Discard {
			a.cfg.Frame.ModifiedSkeleton = nil
		}
		a.cfg.Frame.GeneratedSkeleton = nil
		a.cfg.EnsureGeneratedSkeleton(a.seed)
		a.undoStack = nil
		a.skelVersion++
	}
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, map[string]any{"ok": true})
}

// frameScalarsEqual compares the structural (skeleton-affecting) frame params,
// ignoring the skeleton pointers.
func frameScalarsEqual(a, b FrameConfig) bool {
	a.GeneratedSkeleton, a.ModifiedSkeleton = nil, nil
	b.GeneratedSkeleton, b.ModifiedSkeleton = nil, nil
	return a == b
}

func (a *App) handleStatus(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	st := map[string]any{
		"building":       a.building,
		"seed":           a.cfg.Seed,
		"mode":           a.cfg.Mode,
		"lastExport":     a.lastExport,
		"version":        a.meshVersion,
		"tris":           a.triCount,
		"skelVersion":    a.skelVersion,
		"editMode":       a.editMode,
		"showHandshakes": a.showHandshakes,
		"modified":       a.cfg.Frame.ModifiedSkeleton != nil,
		"canUndo":        len(a.undoStack) > 0,
	}
	a.mu.Unlock()
	writeJSON(w, st)
}

// handleMesh returns the latest serialized mesh (binary; see encodeMesh).
func (a *App) handleMesh(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	data := a.meshData
	ver := a.meshVersion
	a.mu.Unlock()
	w.Header().Set("Content-Type", "application/octet-stream")
	w.Header().Set("Cache-Control", "no-store")
	w.Header().Set("X-Mesh-Version", strconv.Itoa(ver))
	_, _ = w.Write(data)
}

func (a *App) handleRegen(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]any{"seed": a.Regen()})
}

func (a *App) handleSpin(w http.ResponseWriter, r *http.Request) {
	var body struct {
		On bool `json:"on"`
	}
	_ = json.NewDecoder(r.Body).Decode(&body)
	a.mu.Lock()
	a.cfg.AutoSpin = body.On
	a.mu.Unlock()
	writeJSON(w, map[string]any{"autoSpin": body.On})
}

func (a *App) handleExport(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Path       string `json:"path"`
		Resolution int    `json:"resolution"`
	}
	_ = json.NewDecoder(r.Body).Decode(&body)
	cfg, seed := a.Snapshot()
	res := body.Resolution
	if res <= 0 {
		res = cfg.Skin.ExportResolution
	}
	path := body.Path
	if path == "" {
		path = fmt.Sprintf("lattice-%s.stl", cfg.Seed)
	}
	// Force skin mode for export regardless of the editor's current view.
	cfg.Mode = "skin"
	g := cfg.Build(seed, res)
	if err := engine.WriteBinarySTL(path, g); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	a.mu.Lock()
	a.lastExport = path
	a.mu.Unlock()
	writeJSON(w, map[string]any{"path": path, "resolution": res})
}

func (a *App) handleSave(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Path string `json:"path"`
	}
	_ = json.NewDecoder(r.Body).Decode(&body)
	cfg, _ := a.Snapshot()
	path := body.Path
	if path == "" {
		path = a.configPath
	}
	if path == "" {
		path = "config.json"
	}
	if err := cfg.Save(path); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	writeJSON(w, map[string]any{"path": path})
}

func (a *App) handleLoad(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Path string `json:"path"`
	}
	_ = json.NewDecoder(r.Body).Decode(&body)
	path := body.Path
	if path == "" {
		path = a.configPath
	}
	cfg, err := LoadConfig(path)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	a.mu.Lock()
	if s, ok := parseSeed(cfg.Seed); ok {
		a.seed = s
	}
	cfg.Seed = fmt.Sprintf("%06x", a.seed)
	a.cfg = cfg
	a.cfg.EnsureGeneratedSkeleton(a.seed)
	a.undoStack = nil
	a.skelVersion++
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, clientConfig(cfg))
}

// ---- editor endpoints ----

// skeletonResponse is the client-facing view of the effective skeleton plus a
// few frame scalars the client needs for pick radii / bounds.
type skeletonResponse struct {
	*engine.Skeleton
	StrutR    float64 `json:"strutR"`
	ButtressR float64 `json:"buttressR"`
	CubeSize  float64 `json:"cubeSize"`
	Modified  bool    `json:"modified"`
	Version   int     `json:"skelVersion"`
}

func (a *App) skeletonResponseLocked() skeletonResponse {
	return skeletonResponse{
		Skeleton:  a.cfg.EffectiveSkeleton(),
		StrutR:    a.cfg.Frame.StrutR,
		ButtressR: a.cfg.Frame.ButtressR,
		CubeSize:  a.cfg.Frame.CubeSize,
		Modified:  a.cfg.Frame.ModifiedSkeleton != nil,
		Version:   a.skelVersion,
	}
}

func (a *App) handleSkeleton(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	resp := a.skeletonResponseLocked()
	a.mu.Unlock()
	writeJSON(w, resp)
}

func (a *App) handleEdit(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Op        string    `json:"op"`
		VertexID  int       `json:"vertexID"`
		StrutID   int       `json:"strutID"`
		SupportID int       `json:"supportID"`
		Which     string    `json:"which"`
		To        []float64 `json:"to"`
		A         []float64 `json:"a"` // addStrut endpoints
		B         []float64 `json:"b"`
		Kind      string    `json:"kind"`
		Width     float64   `json:"width"`
		Show      bool      `json:"show"`
		Live      bool      `json:"live"` // throttled mid-drag update (no merge/reindex)
	}
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	to := engine.Vec3{}
	if len(body.To) == 3 {
		to = engine.V(body.To[0], body.To[1], body.To[2])
	}
	a.mu.Lock()
	if body.Op == "toggleHandshake" {
		a.showHandshakes = body.Show
		a.mu.Unlock()
		a.signalBuild()
		a.mu.Lock()
		resp := a.skeletonResponseLocked()
		a.mu.Unlock()
		writeJSON(w, resp)
		return
	}
	// Snapshot for undo once per gesture: at the start of a drag (its first
	// live update) or for a standalone edit — never on every throttled tick, so
	// one drag = one undo step. Take it before the fork so undo can reach the
	// unmodified baseline.
	if !a.midDrag {
		a.pushUndoLocked()
	}
	a.midDrag = body.Live
	if a.cfg.Frame.ModifiedSkeleton == nil {
		a.cfg.Frame.ModifiedSkeleton = a.cfg.EffectiveSkeleton().Clone()
	}
	sk := a.cfg.Frame.ModifiedSkeleton
	s := a.cfg.ToSettings()
	commit := !body.Live
	switch body.Op {
	case "moveVertex":
		sk.EditMoveVertex(s, body.VertexID, to, commit)
	case "moveEndpoint":
		sk.EditMoveEndpoint(s, body.StrutID, body.Which, to, commit)
	case "moveSupportEndpoint":
		sk.EditMoveSupportEndpoint(s, body.SupportID, body.Which, to, commit)
	case "addStrut":
		if len(body.A) == 3 && len(body.B) == 3 {
			sk.AddStrut(s, engine.V(body.A[0], body.A[1], body.A[2]),
				engine.V(body.B[0], body.B[1], body.B[2]), body.Kind)
		}
	case "setWidth":
		sk.SetStrutWidth(body.StrutID, body.Width)
	case "deleteStrut":
		sk.DeleteStrut(body.StrutID)
	case "deleteSupport":
		sk.DeleteSupport(body.SupportID)
	default:
		a.mu.Unlock()
		http.Error(w, "unknown op", http.StatusBadRequest)
		return
	}
	sk.RecomputeHandshakeFlags(s)
	a.skelVersion++
	resp := a.skeletonResponseLocked()
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, resp)
}

// handleUndo pops the most recent pre-edit snapshot and restores it (nil =
// back to the generated skeleton). No-op with an empty stack.
func (a *App) handleUndo(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	if len(a.undoStack) == 0 {
		resp := a.skeletonResponseLocked()
		a.mu.Unlock()
		writeJSON(w, resp)
		return
	}
	prev := a.undoStack[len(a.undoStack)-1]
	a.undoStack = a.undoStack[:len(a.undoStack)-1]
	a.cfg.Frame.ModifiedSkeleton = prev // may be nil → revert to generated
	a.skelVersion++
	resp := a.skeletonResponseLocked()
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, resp)
}

func (a *App) handleEditMode(w http.ResponseWriter, r *http.Request) {
	var body struct {
		On bool `json:"on"`
	}
	_ = json.NewDecoder(r.Body).Decode(&body)
	a.mu.Lock()
	a.editMode = body.On
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, map[string]any{"editMode": body.On})
}

func (a *App) handlePreviewSkin(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	a.previewSkin = true
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, map[string]any{"ok": true})
}

// ---- seed helpers ----

// ParseSeed parses a hex seed string; ok is false for empty/invalid input.
func ParseSeed(s string) (uint32, bool) { return parseSeed(s) }

// RandomSeed returns a fresh 24-bit random seed.
func RandomSeed() uint32 { return randomSeed() }

func parseSeed(s string) (uint32, bool) {
	if s == "" {
		return 0, false
	}
	var v uint64
	if _, err := fmt.Sscanf(s, "%x", &v); err != nil {
		return 0, false
	}
	return uint32(v) & 0xffffff, true
}

func randomSeed() uint32 { return rand.Uint32() & 0xffffff }
