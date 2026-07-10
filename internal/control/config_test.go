package control

import (
	"os"
	"path/filepath"
	"testing"
)

// A flat (pre-skin/frame) config must migrate transparently to the nested shape,
// still honoring null=default for absent keys.
func TestLoadConfigMigratesFlat(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "flat.json")
	// Only a few keys present; everything else must fall back to defaults.
	flat := `{"mode":"debug","strutR":0.12,"target":9,"blendK":0.07,"resolution":128}`
	if err := os.WriteFile(path, []byte(flat), 0o644); err != nil {
		t.Fatal(err)
	}
	cfg, err := LoadConfig(path)
	if err != nil {
		t.Fatal(err)
	}
	if cfg.Mode != "debug" {
		t.Errorf("mode: got %q want debug", cfg.Mode)
	}
	if cfg.Frame.StrutR != 0.12 || cfg.Frame.Target != 9 {
		t.Errorf("frame overrides lost: strutR=%v target=%v", cfg.Frame.StrutR, cfg.Frame.Target)
	}
	if cfg.Skin.BlendK != 0.07 || cfg.Skin.Resolution != 128 {
		t.Errorf("skin overrides lost: blendK=%v res=%v", cfg.Skin.BlendK, cfg.Skin.Resolution)
	}
	// Untouched keys keep baked-in defaults.
	def := DefaultConfig()
	if cfg.Frame.CubeSize != def.Frame.CubeSize || cfg.Skin.Corner != def.Skin.Corner {
		t.Errorf("defaults not preserved: cubeSize=%v corner=%v", cfg.Frame.CubeSize, cfg.Skin.Corner)
	}
}

// A nested config round-trips through Save/Load, including a baked skeleton.
func TestConfigRoundTripNested(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "cfg.json")
	cfg := DefaultConfig()
	cfg.Frame.StrutR = 0.1
	cfg.Skin.BlendK = 0.06
	cfg.EnsureGeneratedSkeleton(0x005a08)
	if cfg.Frame.GeneratedSkeleton == nil || len(cfg.Frame.GeneratedSkeleton.Struts) == 0 {
		t.Fatal("expected a baked skeleton")
	}
	if err := cfg.Save(path); err != nil {
		t.Fatal(err)
	}
	got, err := LoadConfig(path)
	if err != nil {
		t.Fatal(err)
	}
	if got.Frame.StrutR != 0.1 || got.Skin.BlendK != 0.06 {
		t.Errorf("overrides lost on round-trip: %+v", got.Frame.StrutR)
	}
	if got.Frame.GeneratedSkeleton == nil ||
		len(got.Frame.GeneratedSkeleton.Struts) != len(cfg.Frame.GeneratedSkeleton.Struts) {
		t.Errorf("skeleton not persisted through save/load")
	}
}

// A partial nested config overrides only present keys.
func TestLoadConfigPartialNested(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "p.json")
	if err := os.WriteFile(path, []byte(`{"frame":{"target":3},"skin":{"gamma":1.1}}`), 0o644); err != nil {
		t.Fatal(err)
	}
	cfg, err := LoadConfig(path)
	if err != nil {
		t.Fatal(err)
	}
	def := DefaultConfig()
	if cfg.Frame.Target != 3 {
		t.Errorf("target override lost: %v", cfg.Frame.Target)
	}
	if cfg.Skin.Gamma != 1.1 {
		t.Errorf("gamma override lost: %v", cfg.Skin.Gamma)
	}
	if cfg.Frame.StrutR != def.Frame.StrutR || cfg.Skin.BlendK != def.Skin.BlendK {
		t.Errorf("unspecified keys should keep defaults")
	}
}
