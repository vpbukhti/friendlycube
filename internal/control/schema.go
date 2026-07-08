package control

// Param describes one control in the panel. Kind is one of:
// "float", "int", "bool", "enum", "text", or "master" (a 0..1 macro slider that
// isn't stored in Config — the panel derives it from, and writes it back into,
// the underlying params listed in Advanced).
type Param struct {
	Key   string   `json:"key"`
	Label string   `json:"label"`
	Kind  string   `json:"kind"`
	Min   float64  `json:"min,omitempty"`
	Max   float64  `json:"max,omitempty"`
	Step  float64  `json:"step,omitempty"`
	Enum  []string `json:"enum,omitempty"`
	Note  string   `json:"note,omitempty"`
	// Help is a plain-language explanation shown on hover.
	Help string `json:"help,omitempty"`
	// Advanced holds the underlying controls revealed by this control's
	// "Advanced" disclosure (used by the macro sliders and the corner slider).
	Advanced []Param `json:"advanced,omitempty"`
	// Rebuild is false for controls that change only the view (no re-mesh).
	Rebuild bool `json:"rebuild"`
}

// SchemaGroup is a titled section of the panel.
type SchemaGroup struct {
	Title  string  `json:"title"`
	Params []Param `json:"params"`
}

// Schema returns the panel layout. Keys match Config's json tags (except the
// "master" sliders, which are panel-only and drive the underlying keys).
func Schema() []SchemaGroup {
	return []SchemaGroup{
		{Title: "Session", Params: []Param{
			{Key: "mode", Label: "Mode", Kind: "enum", Enum: []string{"skin", "debug"}, Rebuild: true,
				Help: "Switch between the finished stone look and a colored diagram of the parts inside."},
			{Key: "seed", Label: "Seed (hex)", Kind: "text", Rebuild: true,
				Help: "The recipe number. The same number always grows the exact same sculpture; change it for a different one."},
			{Key: "autoSpin", Label: "Auto-rotate", Kind: "bool", Rebuild: false,
				Help: "Slowly turn the sculpture on its own when you're not touching it."},
		}},
		{Title: "Structure", Params: []Param{
			{Key: "cubeSize", Label: "Cube size", Kind: "float", Min: 1, Max: 4, Step: 0.05, Rebuild: true,
				Help: "How big the whole cube frame is."},
			{Key: "strutR", Label: "Strut / edge width", Kind: "float", Min: 0.02, Max: 0.25, Step: 0.005, Rebuild: true,
				Help: "How thick the struts and cube edges are — thin and wiry versus chunky."},
			{Key: "buttressR", Label: "Support-strut width", Kind: "float", Min: 0.01, Max: 0.15, Step: 0.005, Rebuild: true,
				Help: "How thick the thin secondary (\"support\") struts are — a finer web of struts that's part of the installation itself, not a printing aid."},
			{Key: "target", Label: "Strut count", Kind: "int", Min: 1, Max: 20, Step: 1, Rebuild: true,
				Help: "How many struts to weave across the cube. More struts make a busier, denser sculpture."},
			{Key: "minClearance", Label: "Min clearance", Kind: "float", Min: 0, Max: 6, Step: 0.1, Rebuild: true,
				Help: "How far apart struts must stay. Higher keeps them from crowding or touching."},
			{Key: "mix", Label: "Edge-to-edge mix", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Balance between struts that run along the frame's edges and struts that cross freely through the middle."},
			{Key: "handshakeProb", Label: "Handshake amount", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "How often a strut becomes a pair of clasped hands instead of a plain bar."},
			{Key: "neighborWeight", Label: "Neighbor pull", Kind: "float", Min: 0, Max: 2, Step: 0.05, Rebuild: true,
				Help: "How much struts prefer connecting to nearby edges versus reaching across to the far side."},
			{Key: "edgeBias", Label: "Edge bias", Kind: "float", Min: 0.2, Max: 4, Step: 0.1, Rebuild: true,
				Help: "Whether struts tend to attach near the ends of an edge or spread toward its middle."},
		}},
		{Title: "Strut length mix", Params: []Param{
			{Key: "lf100", Label: "Full length", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Share of struts left at full length, tip to tip."},
			{Key: "lf95", Label: "Slightly short", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Share of struts trimmed just a little."},
			{Key: "lf90", Label: "Short", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Share of struts trimmed a bit more."},
			{Key: "lf85", Label: "Shorter", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Share of struts trimmed noticeably shorter."},
			{Key: "lf70", Label: "Stub", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Share of struts trimmed short, leaving free-floating tips inside the cube."},
		}},
		{Title: "Skin", Params: []Param{
			{Key: "skinThickness", Label: "Skin thickness", Kind: "master", Min: 0, Max: 1, Step: 0.02, Rebuild: true,
				Help: "How much soft flesh builds up over the struts — from a lean, wiry skeleton to a plump, webbed body. The middle is the tuned look.",
				Advanced: []Param{
					{Key: "blendK", Label: "Blend reach", Kind: "float", Min: 0.005, Max: 0.3, Step: 0.005, Rebuild: true,
						Help: "How far the skin reaches out to merge nearby struts into smooth webbing."},
					{Key: "cap", Label: "Joint swell limit", Kind: "float", Min: -0.01, Max: 0.5, Step: 0.01, Note: "≤0 = off", Rebuild: true,
						Help: "A limit on how much the skin can swell where struts meet, so joints don't balloon."},
					{Key: "gamma", Label: "Crowd fill", Kind: "float", Min: 0.3, Max: 1.6, Step: 0.05, Rebuild: true,
						Help: "How much the skin fills in where many struts crowd together. Lower keeps busy meeting points flatter."},
				}},
			{Key: "edgeRoundness", Label: "Edge roundness", Kind: "master", Min: 0, Max: 1, Step: 0.02, Rebuild: true,
				Help: "How soft the cube's outer edges look where the skin meets them — from a crisp, defined edge to a gently rounded one.",
				Advanced: []Param{
					{Key: "clipBlend", Label: "Edge softening", Kind: "float", Min: -0.01, Max: 0.1, Step: 0.002, Note: "<0 = auto", Rebuild: true,
						Help: "How gently the skin rounds into the cube's outer edges."},
					{Key: "sharpRatio", Label: "Blend smoothness", Kind: "float", Min: 0.02, Max: 0.5, Step: 0.02, Rebuild: true,
						Help: "How smoothly the skin's blends flow, avoiding faint crease lines."},
				}},
			{Key: "corner", Label: "Corner cutoff", Kind: "float", Min: 0, Max: 1, Step: 0.02, Note: "sharp · round · flat", Rebuild: true,
				Help: "The shape of the eight cube corners — from a sharp point, through a smooth round, to a flat clipped facet.",
				Advanced: []Param{
					{Key: "fillet", Label: "Corner ball size", Kind: "float", Min: 0, Max: 0.25, Step: 0.005, Note: "0 = match strut", Rebuild: true,
						Help: "Size of the rounded corner ball. Leave at zero to match the strut thickness."},
					{Key: "vertexGuard", Label: "Corner protection", Kind: "float", Min: -0.01, Max: 0.4, Step: 0.005, Note: "<0 = auto", Rebuild: true,
						Help: "How much of the corner is shielded from the edge trimming. Auto is usually right."},
					{Key: "clipVBlend", Label: "Corner blend", Kind: "float", Min: -0.01, Max: 0.2, Step: 0.005, Note: "<0 = auto", Rebuild: true,
						Help: "How smoothly the corner flows into the edges around it."},
				}},
		}},
		{Title: "Meshing", Params: []Param{
			{Key: "resolution", Label: "Preview detail", Kind: "int", Min: 48, Max: 320, Step: 8, Rebuild: true,
				Help: "Detail of the live preview. Higher looks smoother but takes longer to redraw after each change."},
			{Key: "exportResolution", Label: "Export detail", Kind: "int", Min: 64, Max: 512, Step: 8, Rebuild: false,
				Help: "Detail of the saved model file. Higher gives a finer print but a bigger file and slower export."},
			{Key: "padding", Label: "Box padding", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "Extra empty space kept around the sculpture so it never touches the edge of the build box."},
			{Key: "relax", Label: "Surface tension", Kind: "int", Min: 0, Max: 40, Step: 1, Rebuild: true,
				Help: "Passes of gentle smoothing that pull the surface taut, like soap film. Zero turns it off."},
			{Key: "lambda", Label: "Tension strength", Kind: "float", Min: 0, Max: 1, Step: 0.05, Rebuild: true,
				Help: "How strongly each smoothing pass pulls — gentle versus aggressive."},
		}},
	}
}
