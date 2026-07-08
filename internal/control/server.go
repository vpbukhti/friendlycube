package control

import (
	_ "embed"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
	"math/rand/v2"
	"net/http"
	"strconv"
	"sync"

	"friendlycube/internal/engine"
)

//go:embed panel.html
var panelHTML []byte

// App is the whole application: it owns the config, generates geometry on a
// background worker, keeps the latest mesh (serialized for the browser), and
// serves the control panel + WebGL view. There is no native window — the
// browser is the single surface.
type App struct {
	mu         sync.Mutex
	cfg        Config
	seed       uint32
	building   bool
	lastExport string
	configPath string

	meshData    []byte // encoded mesh for /api/mesh (see encodeMesh)
	meshVersion int    // bumped after each successful rebuild
	triCount    int

	buildSignal chan struct{} // buffered(1); coalesces rebuild requests
}

// NewApp seeds the app. If cfg.Seed is a valid hex it is used; otherwise a
// random seed is drawn and written back into cfg.Seed.
func NewApp(cfg Config, configPath string) *App {
	seed, ok := parseSeed(cfg.Seed)
	if !ok {
		seed = randomSeed()
	}
	cfg.Seed = fmt.Sprintf("%06x", seed)
	return &App{
		cfg:         cfg,
		seed:        seed,
		configPath:  configPath,
		buildSignal: make(chan struct{}, 1),
	}
}

// Start launches the background build worker and requests the first build.
func (a *App) Start() {
	go a.worker()
	a.signalBuild()
}

// worker turns coalesced build signals into a fresh serialized mesh. It runs
// forever on its own goroutine; the newest signal always wins.
func (a *App) worker() {
	for range a.buildSignal {
		a.mu.Lock()
		a.building = true
		cfg := a.cfg
		seed := a.seed
		a.mu.Unlock()

		g := cfg.Build(seed, cfg.Resolution)
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

// Regen draws a fresh random seed and triggers a rebuild. Returns the hex seed.
func (a *App) Regen() string {
	a.mu.Lock()
	a.seed = randomSeed()
	a.cfg.Seed = fmt.Sprintf("%06x", a.seed)
	hex := a.cfg.Seed
	a.mu.Unlock()
	a.signalBuild()
	return hex
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
	mux.HandleFunc("/api/schema", a.handleSchema)
	mux.HandleFunc("/api/config", a.handleConfig)
	mux.HandleFunc("/api/status", a.handleStatus)
	mux.HandleFunc("/api/mesh", a.handleMesh)
	mux.HandleFunc("/api/regen", a.handleRegen)
	mux.HandleFunc("/api/spin", a.handleSpin)
	mux.HandleFunc("/api/export", a.handleExport)
	mux.HandleFunc("/api/save", a.handleSave)
	mux.HandleFunc("/api/load", a.handleLoad)
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

func (a *App) handleConfig(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		cfg, _ := a.Snapshot()
		writeJSON(w, cfg)
		return
	}
	// POST: overlay the body onto the current config (partial updates ok).
	a.mu.Lock()
	oldSeed := a.cfg.Seed
	newCfg := a.cfg
	if err := json.NewDecoder(r.Body).Decode(&newCfg); err != nil {
		a.mu.Unlock()
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	// If the seed text changed to a valid value, adopt it.
	if newCfg.Seed != oldSeed {
		if s, ok := parseSeed(newCfg.Seed); ok {
			a.seed = s
			newCfg.Seed = fmt.Sprintf("%06x", s)
		} else {
			newCfg.Seed = oldSeed // reject unparsable edits
		}
	}
	a.cfg = newCfg
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, map[string]any{"ok": true})
}

func (a *App) handleStatus(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	st := map[string]any{
		"building":   a.building,
		"seed":       a.cfg.Seed,
		"mode":       a.cfg.Mode,
		"lastExport": a.lastExport,
		"version":    a.meshVersion,
		"tris":       a.triCount,
	}
	a.mu.Unlock()
	writeJSON(w, st)
}

// handleMesh returns the latest serialized mesh (binary; see encodeMesh). The
// X-Mesh-Version header lets the client skip re-fetching an unchanged mesh.
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
		res = cfg.ExportResolution
	}
	path := body.Path
	if path == "" {
		path = fmt.Sprintf("lattice-%s.stl", cfg.Seed)
	}
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
	a.mu.Unlock()
	a.signalBuild()
	writeJSON(w, cfg)
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
