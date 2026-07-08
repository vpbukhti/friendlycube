package control

// Param describes one control in the panel. Kind is one of:
// "float", "int", "bool", "enum", "text". For float/int, Min/Max/Step bound the
// slider; Note is an optional hint (e.g. "<0 = auto").
type Param struct {
	Key   string   `json:"key"`
	Label string   `json:"label"`
	Kind  string   `json:"kind"`
	Min   float64  `json:"min,omitempty"`
	Max   float64  `json:"max,omitempty"`
	Step  float64  `json:"step,omitempty"`
	Enum  []string `json:"enum,omitempty"`
	Note  string   `json:"note,omitempty"`
	// Rebuild is false for controls that change only the view (no re-mesh).
	Rebuild bool `json:"rebuild"`
}

// SchemaGroup is a titled section of the panel.
type SchemaGroup struct {
	Title  string  `json:"title"`
	Params []Param `json:"params"`
}

// Schema returns the panel layout. Keys match Config's json tags.
func Schema() []SchemaGroup {
	return []SchemaGroup{
		{Title: "Session", Params: []Param{
			{Key: "mode", Label: "Mode", Kind: "enum", Enum: []string{"skin", "debug"}, Rebuild: true},
			{Key: "seed", Label: "Seed (hex)", Kind: "text", Rebuild: true},
			{Key: "autoSpin", Label: "Auto-rotate", Kind: "bool", Rebuild: false},
		}},
		{Title: "Structure", Params: []Param{
			{Key: "cubeSize", Label: "Cube size", Kind: "float", Min: 1, Max: 4, Step: 0.05, Rebuild: true},
			{Key: "strutR", Label: "Strut / edge width", Kind: "float", Min: 0.02, Max: 0.25, Step: 0.005, Rebuild: true},
			{Key: "buttressR", Label: "Support width", Kind: "float", Min: 0.01, Max: 0.15, Step: 0.005, Rebuild: true},
			{Key: "target", Label: "Strut count", Kind: "int", Min: 1, Max: 20, Step: 1, Rebuild: true},
			{Key: "minClearance", Label: "Min clearance (×r)", Kind: "float", Min: 0, Max: 6, Step: 0.1, Rebuild: true},
			{Key: "mix", Label: "Edge-to-edge mix", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "handshakeProb", Label: "Handshake amount", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "neighborWeight", Label: "Neighbor weight", Kind: "float", Min: 0, Max: 2, Step: 0.05, Rebuild: true},
			{Key: "edgeBias", Label: "Edge bias", Kind: "float", Min: 0.2, Max: 4, Step: 0.1, Rebuild: true},
		}},
		{Title: "Strut length mix", Params: []Param{
			{Key: "lf100", Label: "Full (1.00)", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "lf95", Label: "0.95", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "lf90", Label: "0.90", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "lf85", Label: "0.85", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "lf70", Label: "Short (0.70)", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
		}},
		{Title: "Sleeve field", Params: []Param{
			{Key: "blendK", Label: "Blend reach k", Kind: "float", Min: 0.005, Max: 0.3, Step: 0.005, Rebuild: true},
			{Key: "gamma", Label: "Crowding γ", Kind: "float", Min: 0.3, Max: 1.6, Step: 0.05, Rebuild: true},
			{Key: "cap", Label: "Joint cap B", Kind: "float", Min: -0.01, Max: 0.5, Step: 0.01, Note: "≤0 = off", Rebuild: true},
			{Key: "sharpRatio", Label: "Smooth anchor k'/k", Kind: "float", Min: 0.02, Max: 0.5, Step: 0.02, Rebuild: true},
			{Key: "corner", Label: "Corner (sharp·round·flat)", Kind: "float", Min: 0, Max: 1, Step: 0.02, Rebuild: true},
			{Key: "fillet", Label: "Corner solid radius", Kind: "float", Min: 0, Max: 0.25, Step: 0.005, Note: "0 = match strut", Rebuild: true},
			{Key: "vertexGuard", Label: "Corner clip guard", Kind: "float", Min: -0.01, Max: 0.4, Step: 0.005, Note: "<0 = auto", Rebuild: true},
			{Key: "clipBlend", Label: "Edge clip fillet", Kind: "float", Min: -0.01, Max: 0.1, Step: 0.002, Note: "<0 = auto", Rebuild: true},
			{Key: "clipVBlend", Label: "Corner→edge neck", Kind: "float", Min: -0.01, Max: 0.2, Step: 0.005, Note: "<0 = auto", Rebuild: true},
		}},
		{Title: "Meshing", Params: []Param{
			{Key: "resolution", Label: "Live resolution", Kind: "int", Min: 48, Max: 320, Step: 8, Rebuild: true},
			{Key: "exportResolution", Label: "Export resolution", Kind: "int", Min: 64, Max: 512, Step: 8, Rebuild: false},
			{Key: "padding", Label: "Box padding", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
			{Key: "relax", Label: "Tension passes", Kind: "int", Min: 0, Max: 40, Step: 1, Rebuild: true},
			{Key: "lambda", Label: "Tension step λ", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true},
		}},
	}
}
