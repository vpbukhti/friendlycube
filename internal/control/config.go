// Package control is the "control center": one Config holding every knob, its
// JSON persistence (null/absent → baked-in default), the mapping onto the
// engine's Settings/SkinParams, and the live browser control-panel server.
package control

import (
	"encoding/json"
	"os"

	"friendlycube/internal/engine"
)

// Config is the single source of truth for every parameter. Values are always
// concrete (resolved) here; the null-means-default semantics live in
// LoadConfig, which starts from DefaultConfig and lets JSON override only the
// keys that are present and non-null.
//
// A few sleeve knobs (VertexGuard, ClipBlend, ClipVBlend) accept a NEGATIVE
// sentinel meaning "auto" — the engine derives them from the tube radius / the
// corner mode. That is preserved here so the baked-in defaults behave exactly
// like the tuned command-line defaults did.
type Config struct {
	// ---- Session / view ----
	Mode     string `json:"mode"`     // "skin" (implicit sleeve) or "debug" (colored features)
	Seed     string `json:"seed"`     // hex, e.g. "1a2b3c"; "" = random on startup
	AutoSpin bool   `json:"autoSpin"` // idle auto-rotation

	// ---- Structure (generation) ----
	CubeSize         float64 `json:"cubeSize"`
	StrutR           float64 `json:"strutR"`    // strut / edge tube radius (edge width)
	ButtressR        float64 `json:"buttressR"` // support tube radius
	Target           int     `json:"target"`    // number of struts to place
	MinClearance     float64 `json:"minClearance"`
	Mix              float64 `json:"mix"`           // fraction edge-to-edge vs free struts
	HandshakeProb    float64 `json:"handshakeProb"` // amount of handshakes
	NeighborWeight   float64 `json:"neighborWeight"`
	EdgeBias         float64 `json:"edgeBias"`
	Lf100            float64 `json:"lf100"` // length-factor weights (full → short struts)
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

	// ---- Sleeve / skin field ----
	BlendK      float64 `json:"blendK"`      // crowding blend reach k (length)
	Gamma       float64 `json:"gamma"`       // crowding exponent γ
	Cap         float64 `json:"cap"`         // joint push-out cap B (<=0 = off)
	SharpRatio  float64 `json:"sharpRatio"`  // smooth-anchor stiffness ratio k'/k
	Corner      float64 `json:"corner"`      // corner shape 0 sharp · 0.5 round · 1 flat
	Fillet      float64 `json:"fillet"`      // corner-solid radius (0 = match StrutR)
	VertexGuard float64 `json:"vertexGuard"` // corner clip-suppression radius (<=0 = auto)
	ClipBlend   float64 `json:"clipBlend"`   // outer edge clip fillet width (<0 = auto)
	ClipVBlend  float64 `json:"clipVBlend"`  // corner→edge clip neck width (<0 = auto)

	// ---- Meshing ----
	Resolution       int     `json:"resolution"`       // live marching-cubes grid per axis
	ExportResolution int     `json:"exportResolution"` // grid per axis for STL export
	Padding          float64 `json:"padding"`
	Relax            int     `json:"relax"`  // surface-tension passes (0 = off)
	Lambda           float64 `json:"lambda"` // surface-tension step per pass
}

// DefaultConfig is the baked-in set of sane defaults (the tuned values the
// project converged on). A config file only needs to specify what it wants to
// change; anything absent or null falls back to these.
func DefaultConfig() Config {
	return Config{
		Mode:     "skin",
		Seed:     "005a08",
		AutoSpin: true,

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

		BlendK:      0.05,
		Gamma:       0.8,
		Cap:         0.12,
		SharpRatio:  0.2,
		Corner:      0.62,
		Fillet:      0,
		VertexGuard: 0,  // auto (mode-aware)
		ClipBlend:   -1, // auto (~0.4×r)
		ClipVBlend:  -1, // auto (~r)

		Resolution:       160,
		ExportResolution: 240,
		Padding:          0.3,
		Relax:            0,
		Lambda:           0.1,
	}
}

// LoadConfig reads a JSON config, overlaying it on the baked-in defaults.
// Because encoding/json leaves a Go field untouched when the JSON value is
// absent OR null, both cases resolve to the default — exactly the requested
// "null = overwrite with baked-in default" behavior. Only present, non-null
// keys override.
func LoadConfig(path string) (Config, error) {
	cfg := DefaultConfig()
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, err
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		return cfg, err
	}
	return cfg, nil
}

// Save writes the fully-resolved config (all keys present, no nulls) as JSON.
func (c Config) Save(path string) error {
	data, err := json.MarshalIndent(c, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, append(data, '\n'), 0o644)
}

// ToSettings maps the generation knobs onto the engine's Settings.
func (c Config) ToSettings() engine.Settings {
	return engine.Settings{
		CubeSize:         c.CubeSize,
		StrutR:           c.StrutR,
		ButtressR:        c.ButtressR,
		Target:           c.Target,
		MinClearance:     c.MinClearance,
		Mix:              c.Mix,
		HandshakeProb:    c.HandshakeProb,
		NeighborWeight:   c.NeighborWeight,
		EdgeBias:         c.EdgeBias,
		Lf100:            c.Lf100,
		Lf95:             c.Lf95,
		Lf90:             c.Lf90,
		Lf85:             c.Lf85,
		Lf70:             c.Lf70,
		VertexPresnapPct: c.VertexPresnapPct,
		EdgeCylinderPct:  c.EdgeCylinderPct,
		WireSnapMult:     c.WireSnapMult,
		SupportNum:       c.SupportNum,
		SupportJitter:    c.SupportJitter,
		ApexSkipMult:     c.ApexSkipMult,
		SupportMinLen:    c.SupportMinLen,
	}
}

// ToSkinParams maps the sleeve + meshing knobs onto the engine's SkinParams.
// res lets the caller substitute a resolution (e.g. ExportResolution).
func (c Config) ToSkinParams(res int) engine.SkinParams {
	return engine.SkinParams{
		Resolution:  res,
		BlendK:      c.BlendK,
		Gamma:       c.Gamma,
		Cap:         c.Cap,
		SharpRatio:  c.SharpRatio,
		VertexGuard: c.VertexGuard,
		ClipBlend:   c.ClipBlend,
		ClipVBlend:  c.ClipVBlend,
		Relax:       c.Relax,
		Lambda:      c.Lambda,
		Padding:     c.Padding,
		Fillet:      c.Fillet,
		Corner:      c.Corner,
	}
}

// Build produces the geometry for the given seed at the given resolution,
// dispatching on Mode. Pure CPU — safe to call off the GL thread.
func (c Config) Build(seed uint32, res int) *engine.Group {
	s := c.ToSettings()
	if c.Mode == "debug" {
		return engine.BuildScene(s, seed)
	}
	return engine.BuildSkin(s, seed, c.ToSkinParams(res))
}
