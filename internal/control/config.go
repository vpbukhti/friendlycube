// Package control is the "control center": one Config holding every knob, its
// JSON persistence (null/absent → baked-in default), the mapping onto the
// engine's Settings/SkinParams, and the live browser control-panel server.
package control

import (
	"encoding/json"
	"os"

	"friendlycube/internal/engine"
)

// Config is the single source of truth for every parameter, split into two
// halves:
//
//   - Skin: the implicit-sleeve + meshing/view knobs (never affect the frame).
//   - Frame: the procedural generation params PLUS the baked skeleton. The
//     skeleton is the explicit, editable description of the struts/supports/
//     handshakes. GeneratedSkeleton is baked from (params, seed) if absent;
//     ModifiedSkeleton, when present, is the source of truth and suppresses
//     regeneration.
//
// A few sleeve knobs (VertexGuard, ClipBlend, ClipVBlend) accept a NEGATIVE
// sentinel meaning "auto" — the engine derives them from the tube radius / the
// corner mode.
type Config struct {
	Mode     string `json:"mode"`     // "skin" (implicit sleeve) or "debug" (colored features)
	Seed     string `json:"seed"`     // hex, e.g. "1a2b3c"; "" = random on startup
	AutoSpin bool   `json:"autoSpin"` // idle auto-rotation

	Skin  SkinConfig  `json:"skin"`
	Frame FrameConfig `json:"frame"`
}

// SkinConfig holds the implicit-skin + meshing/view knobs. Editing any of these
// never disturbs the skeleton (no discard-confirm).
type SkinConfig struct {
	BlendK      float64 `json:"blendK"`      // crowding blend reach k (length)
	Gamma       float64 `json:"gamma"`       // crowding exponent γ
	Cap         float64 `json:"cap"`         // joint push-out cap B (<=0 = off)
	SharpRatio  float64 `json:"sharpRatio"`  // smooth-anchor stiffness ratio k'/k
	Corner      float64 `json:"corner"`      // corner shape 0 sharp · 0.5 round · 1 flat
	Fillet      float64 `json:"fillet"`      // corner-solid radius (0 = match StrutR)
	VertexGuard float64 `json:"vertexGuard"` // corner clip-suppression radius (<=0 = auto)
	ClipBlend   float64 `json:"clipBlend"`   // outer edge clip fillet width (<0 = auto)
	ClipVBlend  float64 `json:"clipVBlend"`  // corner→edge clip neck width (<0 = auto)

	Resolution       int     `json:"resolution"`       // live marching-cubes grid per axis
	ExportResolution int     `json:"exportResolution"` // grid per axis for STL export
	Padding          float64 `json:"padding"`
	Relax            int     `json:"relax"`  // surface-tension passes (0 = off)
	Lambda           float64 `json:"lambda"` // surface-tension step per pass
}

// FrameConfig holds the procedural generation params and the baked skeleton.
// The scalar params are "structural": changing one while a ModifiedSkeleton
// exists prompts the client to discard the edits (see server handleConfig).
type FrameConfig struct {
	CubeSize         float64 `json:"cubeSize"`
	StrutR           float64 `json:"strutR"`    // strut / edge tube radius (edge width)
	ButtressR        float64 `json:"buttressR"` // thin secondary ("support") strut radius
	Target           int     `json:"target"`    // number of struts to place
	MinClearance     float64 `json:"minClearance"`
	Mix              float64 `json:"mix"` // fraction edge-to-edge vs free struts
	HandshakeProb    float64 `json:"handshakeProb"`
	NeighborWeight   float64 `json:"neighborWeight"`
	EdgeBias         float64 `json:"edgeBias"`
	Lf100            float64 `json:"lf100"` // length-factor weights (full → short)
	Lf95             float64 `json:"lf95"`
	Lf90             float64 `json:"lf90"`
	Lf85             float64 `json:"lf85"`
	Lf70             float64 `json:"lf70"`
	VertexPresnapPct float64 `json:"vertexPresnapPct"`
	EdgeCylinderPct  float64 `json:"edgeCylinderPct"`
	WireSnapMult     float64 `json:"wireSnapMult"`
	SupportNum       int     `json:"supportNum"`
	SupportJitter    float64 `json:"supportJitter"`
	ApexSkipMult     float64 `json:"apexSkipMult"`
	SupportMinLen    float64 `json:"supportMinLen"`

	// GeneratedSkeleton is baked from (params, seed) on first use and cached.
	// ModifiedSkeleton, when non-nil, is the hand-edited source of truth.
	GeneratedSkeleton *engine.Skeleton `json:"generatedSkeleton,omitempty"`
	ModifiedSkeleton  *engine.Skeleton `json:"modifiedSkeleton,omitempty"`
}

// DefaultConfig is the baked-in set of sane defaults (the tuned values the
// project converged on). A config file only needs to specify what it changes.
func DefaultConfig() Config {
	return Config{
		Mode:     "skin",
		Seed:     "005a08",
		AutoSpin: true,
		Skin: SkinConfig{
			BlendK:      0.05,  // macro "skin thickness" 0.5
			Gamma:       0.8,   // (advanced)
			Cap:         0.12,  // macro "skin thickness" 0.5
			SharpRatio:  0.2,   // (advanced)
			Corner:      0.62,  // macro "corner cutoff"
			Fillet:      0,     // (advanced) 0 = match strut
			VertexGuard: 0,     // (advanced) auto (mode-aware)
			ClipBlend:   0.034, // macro "edge roundness" 0.5  (= 0.4×r)
			ClipVBlend:  -1,    // (advanced) auto (~r)

			Resolution:       160,
			ExportResolution: 240,
			Padding:          0.3,
			Relax:            0,
			Lambda:           0.1,
		},
		Frame: FrameConfig{
			CubeSize:         2.0,
			StrutR:           0.085,
			ButtressR:        0.035,
			Target:           7,
			MinClearance:     2.5,
			Mix:              0.5,
			HandshakeProb:    0.5,
			NeighborWeight:   0.4,
			EdgeBias:         1.6,
			Lf100:            0.20,
			Lf95:             0.15,
			Lf90:             0.20,
			Lf85:             0.15,
			Lf70:             0.30,
			VertexPresnapPct: 0.10,
			EdgeCylinderPct:  0.10,
			WireSnapMult:     1.0,
			SupportNum:       3,
			SupportJitter:    0.55,
			ApexSkipMult:     1.3,
			SupportMinLen:    0.6,
		},
	}
}

// LoadConfig reads a JSON config, overlaying it on the baked-in defaults. Absent
// or null keys resolve to the default (encoding/json leaves the field untouched).
// A pre-nesting flat config (top-level "strutR" etc.) is transparently migrated.
func LoadConfig(path string) (Config, error) {
	cfg := DefaultConfig()
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, err
	}
	// Back-compat: migrate a flat (pre-skin/frame) config to the nested shape.
	var probe map[string]json.RawMessage
	if json.Unmarshal(data, &probe) == nil {
		_, hasFrame := probe["frame"]
		_, hasStrutR := probe["strutR"]
		if !hasFrame && hasStrutR {
			return migrateFlat(data)
		}
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		return cfg, err
	}
	return cfg, nil
}

// migrateFlat converts an old flat config (all keys at top level) into the
// nested shape, still honoring the null=default overlay.
func migrateFlat(data []byte) (Config, error) {
	def := DefaultConfig()
	// Mirror of the old flat layout; start from the resolved defaults so absent
	// keys keep their baked-in values.
	flat := struct {
		Mode     *string `json:"mode"`
		Seed     *string `json:"seed"`
		AutoSpin *bool   `json:"autoSpin"`
		SkinConfig
		FrameConfig
	}{}
	// Preload with defaults so partial files don't zero unspecified keys.
	flat.SkinConfig = def.Skin
	flat.FrameConfig = def.Frame
	if err := json.Unmarshal(data, &flat); err != nil {
		return def, err
	}
	cfg := def
	if flat.Mode != nil {
		cfg.Mode = *flat.Mode
	}
	if flat.Seed != nil {
		cfg.Seed = *flat.Seed
	}
	if flat.AutoSpin != nil {
		cfg.AutoSpin = *flat.AutoSpin
	}
	cfg.Skin = flat.SkinConfig
	cfg.Frame = flat.FrameConfig
	return cfg, nil
}

// Save writes the fully-resolved config (skin + frame incl. skeletons) as JSON.
func (c Config) Save(path string) error {
	data, err := json.MarshalIndent(c, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, append(data, '\n'), 0o644)
}

// ToSettings maps the generation knobs onto the engine's Settings.
func (c Config) ToSettings() engine.Settings {
	f := c.Frame
	return engine.Settings{
		CubeSize:         f.CubeSize,
		StrutR:           f.StrutR,
		ButtressR:        f.ButtressR,
		Target:           f.Target,
		MinClearance:     f.MinClearance,
		Mix:              f.Mix,
		HandshakeProb:    f.HandshakeProb,
		NeighborWeight:   f.NeighborWeight,
		EdgeBias:         f.EdgeBias,
		Lf100:            f.Lf100,
		Lf95:             f.Lf95,
		Lf90:             f.Lf90,
		Lf85:             f.Lf85,
		Lf70:             f.Lf70,
		VertexPresnapPct: f.VertexPresnapPct,
		EdgeCylinderPct:  f.EdgeCylinderPct,
		WireSnapMult:     f.WireSnapMult,
		SupportNum:       f.SupportNum,
		SupportJitter:    f.SupportJitter,
		ApexSkipMult:     f.ApexSkipMult,
		SupportMinLen:    f.SupportMinLen,
	}
}

// ToSkinParams maps the (real) skin + meshing knobs onto the engine's
// SkinParams. res substitutes a resolution (e.g. ExportResolution).
func (c Config) ToSkinParams(res int) engine.SkinParams {
	k := c.Skin
	return engine.SkinParams{
		Resolution:  res,
		BlendK:      k.BlendK,
		Gamma:       k.Gamma,
		Cap:         k.Cap,
		SharpRatio:  k.SharpRatio,
		VertexGuard: k.VertexGuard,
		ClipBlend:   k.ClipBlend,
		ClipVBlend:  k.ClipVBlend,
		Relax:       k.Relax,
		Lambda:      k.Lambda,
		Padding:     k.Padding,
		Fillet:      k.Fillet,
		Corner:      k.Corner,
	}
}

// EnsureGeneratedSkeleton bakes and caches the procedural skeleton if absent.
// Call on the App's stored config so the bake persists (and later Saves).
func (c *Config) EnsureGeneratedSkeleton(seed uint32) {
	if c.Frame.GeneratedSkeleton == nil {
		c.Frame.GeneratedSkeleton = engine.BakeSkeleton(c.ToSettings(), seed)
	}
}

// EffectiveSkeleton is the ModifiedSkeleton if present, else the Generated one.
func (c Config) EffectiveSkeleton() *engine.Skeleton {
	if c.Frame.ModifiedSkeleton != nil {
		return c.Frame.ModifiedSkeleton
	}
	return c.Frame.GeneratedSkeleton
}

// Build produces the geometry for the given seed at the given resolution,
// dispatching on Mode. It bakes the skeleton if neither is present (headless
// path). The handshake overlay is always included here (headless export shows
// handshakes); the live server composes the overlay conditionally instead.
func (c Config) Build(seed uint32, res int) *engine.Group {
	if c.Frame.GeneratedSkeleton == nil && c.Frame.ModifiedSkeleton == nil {
		c.Frame.GeneratedSkeleton = engine.BakeSkeleton(c.ToSettings(), seed)
	}
	s := c.ToSettings()
	skel := c.EffectiveSkeleton()
	var root *engine.Group
	if c.Mode == "debug" {
		root = engine.BuildSceneFromSkeleton(s, skel)
	} else {
		root = engine.BuildSkinFromSkeleton(s, skel, c.ToSkinParams(res))
	}
	root.Add(engine.BuildHandshakeOverlay(s, skel))
	return root
}
