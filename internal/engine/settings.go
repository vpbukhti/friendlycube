package engine

// Settings mirror the DEFAULTS object in the JS source. Same names and
// defaults to keep the behaviour identical given the same seed.
type Settings struct {
	CubeSize                      float64
	StrutR                        float64
	ButtressR                     float64
	Target                        int
	MinClearance                  float64
	Mix                           float64
	HandshakeProb                 float64
	NeighborWeight                float64
	EdgeBias                      float64
	Lf100, Lf95, Lf90, Lf85, Lf70 float64
	VertexPresnapPct              float64
	EdgeCylinderPct               float64
	WireSnapMult                  float64
	SupportNum                    int
	SupportJitter                 float64
	ApexSkipMult                  float64
	SupportMinLen                 float64
}

func DefaultSettings() Settings {
	return Settings{
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
	}
}

// Debug palette — same values as the JS COLORS map.
var Colors = struct {
	Wireframe       int
	StrutFull       int
	StrutEdgeToEdge int
	StrutShortened  int
	Arm             int
	SkinAnchor      int
	FloatingAnchor  int
	SnappedToWire   int
	SkinWebbing     int
	PyramidSupport  int
	CollisionSnap   int
	SnapToInner     int
	LandingBead     int
}{
	Wireframe:       0x9b9b9b,
	StrutFull:       0xf2ede2,
	StrutEdgeToEdge: 0x5dcaa5,
	StrutShortened:  0xfac775,
	Arm:             0xf0997b,
	SkinAnchor:      0x85b7eb,
	FloatingAnchor:  0xed93b1,
	SnappedToWire:   0x97c459,
	SkinWebbing:     0x5dcaa5,
	PyramidSupport:  0x7f77dd,
	CollisionSnap:   0xe24b4a,
	SnapToInner:     0xfac775,
	LandingBead:     0x378add,
}

// LCG state matching the JS generator:
//
//	s = (s * 1664525 + 1013904223) >>> 0
//	rand() returns s / 2^32
type Rand struct {
	s uint32
}

func NewRand(seed uint32) *Rand { return &Rand{s: seed} }

func (r *Rand) F() float64 {
	r.s = r.s*1664525 + 1013904223
	return float64(r.s) / 4294967296.0
}
