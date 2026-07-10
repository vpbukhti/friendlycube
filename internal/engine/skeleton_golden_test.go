package engine

import (
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"math"
	"testing"
)

// Golden determinism guard for the bake path.
//
//   - At HandshakeProb=0 there are no handshakes, so the debug + skin meshes
//     must be BYTE-IDENTICAL to the values captured from the pre-refactor code.
//   - The baked SUPPORT segments must match the pre-refactor supports at every
//     HandshakeProb — this proves the handshake RNG draws are counted correctly
//     (supports are planned from the same `dec` stream, after those draws).
//     These golden hashes were captured from the original code in a git worktree
//     (hash of P1/P2 float64 + Kind + BeadKind per support).
func hashMeshG(g *Group) string {
	pos, nrm, col := MeshBuffers(g)
	h := sha256.New()
	buf := make([]byte, 4)
	for _, s := range [][]float32{pos, nrm, col} {
		for _, f := range s {
			binary.LittleEndian.PutUint32(buf, math.Float32bits(f))
			h.Write(buf)
		}
	}
	return fmt.Sprintf("%x tris=%d", h.Sum(nil)[:8], len(pos)/9)
}

func hashSkelSupports(sk *Skeleton) string {
	h := sha256.New()
	buf := make([]byte, 8)
	for _, ss := range sk.Supports {
		for _, v := range []Vec3{ss.P1, ss.P2} {
			for _, f := range v {
				binary.LittleEndian.PutUint64(buf, math.Float64bits(f))
				h.Write(buf)
			}
		}
		h.Write([]byte(ss.Kind + ss.BeadKind))
	}
	return fmt.Sprintf("%x n=%d", h.Sum(nil)[:8], len(sk.Supports))
}

var goldenSeeds = []uint32{0x005a08, 0x1a2b3c, 0x000001, 0xabcdef}

// hp=0.0 → full debug + skin meshes identical to the old code.
var goldenNoHandshake = map[uint32]struct{ debug, skin string }{
	0x005a08: {"caa72df2e918c544 tris=3360", "372364ab35c5fe49 tris=44464"},
	0x1a2b3c: {"37c3aeb4714f9aee tris=2976", "5d42aeb83f34e086 tris=39460"},
	0x000001: {"2969c77fe42325cd tris=3296", "a98c77d2663372a3 tris=42408"},
	0xabcdef: {"42c5daaea458086f tris=2528", "84110a8c2e4ba98b tris=36132"},
}

// True support segments from the old pipeline (worktree capture), per hp.
var goldenSupports = map[float64]map[uint32]string{
	0.0: {
		0x005a08: "7ca7aa02d096c33a n=30", 0x1a2b3c: "6a260763b3062d92 n=24",
		0x000001: "cea8bde33b7cc679 n=29", 0xabcdef: "335a6b9712249657 n=17",
	},
	0.5: {
		0x005a08: "f740573cdf8e98c7 n=30", 0x1a2b3c: "2dfe96b7937b579a n=24",
		0x000001: "adae418dd76b4c7f n=30", 0xabcdef: "188b30ec6b4eed57 n=18",
	},
	1.0: {
		0x005a08: "1497a01b85efc43d n=30", 0x1a2b3c: "697245280e1cf3ac n=24",
		0x000001: "7c8c7f1a35dacf4c n=30", 0xabcdef: "d57d5a2803c31ffb n=18",
	},
}

func TestBakeReproducesNoHandshake(t *testing.T) {
	sp := DefaultSkinParams()
	sp.Resolution = 64
	for _, seed := range goldenSeeds {
		s := DefaultSettings()
		s.HandshakeProb = 0.0
		sk := BakeSkeleton(s, seed)
		g := goldenNoHandshake[seed]
		if got := hashMeshG(BuildSceneFromSkeleton(s, sk)); got != g.debug {
			t.Errorf("seed %06x debug mesh: got %s want %s", seed, got, g.debug)
		}
		if got := hashMeshG(BuildSkinFromSkeleton(s, sk, sp)); got != g.skin {
			t.Errorf("seed %06x skin mesh: got %s want %s", seed, got, g.skin)
		}
	}
}

func TestBakeSupportsMatchOldPipeline(t *testing.T) {
	for hp, golden := range goldenSupports {
		for _, seed := range goldenSeeds {
			s := DefaultSettings()
			s.HandshakeProb = hp
			sk := BakeSkeleton(s, seed)
			if got := hashSkelSupports(sk); got != golden[seed] {
				t.Errorf("hp=%.1f seed %06x supports: got %s want %s", hp, seed, got, golden[seed])
			}
		}
	}
}
