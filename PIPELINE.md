# friendlycube — pipeline, math, and parameters

How a seed becomes a watertight sculpture, and what every knob does. This is the
reference behind the [README](README.md); the implicit-sleeve technique is
written up from first principles in
[organic_sleeve_method.md](organic_sleeve_method.md).

Everything is **deterministic**: the same seed + config always produces
bit-identical geometry. Randomness comes from a seeded LCG, and the geometry is a
pure function of `(config, seed)`.

---

## Overview

```
seed + config
   │
   ▼
[1] cube topology ......... 8 vertices, 12 edges, 6 faces
   │
   ▼
[2] struts ................ span the cube (random + edge-to-edge), snapped,
   │                         trimmed by length factors, spaced by clearance
   ▼
[3] handshakes ............ some struts become clasped-hand meshes
   │
   ▼
[4] support struts ........ a thinner secondary web (part of the piece)
   │
   ▼   ── skeleton = a set of tube segments (edges, struts, supports) ──
   │
   ▼
[5] implicit sleeve ....... one signed-distance field wraps the whole skeleton
   │                         (the "organic sleeve"): blend, cap, clip, corners
   ▼
[6] marching cubes ........ extract the zero level set → triangles + normals
   │
   ▼
[7] surface tension ....... optional mean-curvature relaxation (soap-film)
   │
   ▼
[8] output ................ binary mesh → WebGL viewer  ·  or  → binary STL
   │
   ▼  (optional)
[9] print split ........... printable pieces, guide rods, numbered cuts
```

Stages 1–4 live in `internal/engine` (`topology.go`, `struts.go`, `joints.go`,
`supports.go`, `rebuild.go`). Stage 5 is `sdf.go` + `skin.go`, stage 6 is
`mc.go`, stage 7 is `relax.go`, stage 8 is `stl.go` / `mesh.go`.

---

## [1] Cube topology

`BuildCubeTopology(half)` places the 8 corners at `(±half, ±half, ±half)`,
derives the 12 edges (corner pairs differing on one axis) and 6 faces. `half =
cubeSize / 2`. This frame is the scaffold everything else snaps to.

## [2] Struts

`GenerateStruts` places `target` struts, retrying up to a cap. Each candidate is
one of two kinds, chosen with probability `mix`:

- **Random line** — a random point + direction, clipped to the cube. Endpoints
  landing near a corner or edge **snap** to it (tolerances `vertexPresnapPct`,
  `edgeCylinderPct`, `wireSnapMult`), so struts tend to key into the frame.
- **Edge-to-edge** — pick a source edge, a point on it (`edgeBias` biases toward
  ends vs middle), then a *non-adjacent* destination edge (`neighborWeight`
  weights near vs far edges), avoiding pairs that share a corner.

Accepted struts must be longer than half the cube and must clear all existing
struts by `minClearance × strutR`. A **length factor** then optionally trims a
strut toward its midpoint — drawn from the weighted set `{1.00, .95, .90, .85,
.70}` (weights `lf100…lf70`); trimming leaves free-floating tips inside the cube.

## [3] Handshakes

For each strut, with probability `handshakeProb`, the middle is replaced by two
forearms + clasped hands (a placeholder mesh, meant to be re-sculpted later). The
handshake RNG is seeded separately (`seed ^ 0x9e3779b9`) so toggling handshakes
doesn't disturb the strut skeleton.

## [4] Support struts

A thinner secondary web at radius `buttressR` (**part of the installation, not a
printing aid**). Two planners: *webbing* between skin anchors on the same face,
and *pyramid* props under floating strut tips. A global pass removes
support-vs-support collisions.

The result of 1–4 is a **skeleton**: a bag of tube segments — the 12 edge tubes
(radius `strutR`), the strut tubes (`strutR`), and the support tubes
(`buttressR`).

---

## [5] The implicit sleeve (the interesting part)

Rather than mesh each tube and boolean them, the whole skeleton is wrapped in one
**signed-distance field** `D(p)`; the surface is its zero level set `D = 0`
(negative inside). This gives a single watertight skin with controllable organic
blending. Evaluated per grid point in `Field.Eval` (`sdf.go`).

### 5.1 Per-tube distance

For a point `p` and tube *i* (segment with radius `rᵢ`):

```
eᵢ(p) = distance(p, segmentᵢ) − rᵢ      // <0 inside the bare tube
```

A lone tube's surface is exactly `eᵢ = 0`, i.e. a clean cylinder of radius `rᵢ`.

### 5.2 Crowding field — blending tubes where they meet

A hard union `min eᵢ` would give creased joints. Instead we measure how many
tubes are "nearly tied" at `p` and let the surface bulge out there — a fillet
whose size grows with how crowded the point is. With `e_min = minᵢ eᵢ` and blend
reach `k`:

```
S     = Σᵢ exp( −(eᵢ − e_min) / k )        // effective # of contributing tubes (≥1)
x     = ln S                                // crowding, ≥ 0
dip   = k · x^γ                             // how far to push the surface out
D     = e_min − B · tanh( dip / B )         // cap the push-out at B
```

- `γ` (**gamma**) shapes how the bulge grows with crowd size: `γ=1` is the
  classic log-sum-exp smooth-min; `γ<1` keeps pairwise fillets full while
  flattening busy meeting points; `γ>1` exaggerates them.
- `B` (**cap**) is a hard ceiling (via `tanh`) on how far any joint may swell
  past the bare tube. `B ≤ 0` disables it.

### 5.3 Smooth anchor — keeping it C¹ for γ ≠ 1

`x` is anchored to the hard `e_min`, whose gradient kinks along the surface where
the nearest-tube identity switches (the fillet valleys). For `γ ≠ 1` that kink
survives into `D` as a faint crease. Fix: replace the hard `e_min` with a *sharp
but smooth* soft-min at a stiffer reach `k' = sharpRatio · k` (`k' ≪ k`). Two
log-sum-exp sums are accumulated in one pass:

```
Sk  = Σ exp(−(eᵢ−e_min)/k)      Sk' = Σ exp(−(eᵢ−e_min)/k')
sm_k' = e_min − k'·ln Sk'                    // smooth stand-in for the hard min
x     = ln Sk − (k'/k)·ln Sk'   (≥ 0)        // crowding, now smooth
D     = sm_k' − B·tanh(k·x^γ / B)
```

Every term is now a composition of smooth functions, so `D` is C∞ even for
`γ ≠ 1`. As `k' → 0` this reduces exactly to §5.2. (The hard `e_min` is still
computed, but only as the numerically-stable exponent offset — it cancels
analytically inside both soft-mins.) A BVH culls tubes farther than `≈18·k` from
`e_min`, since their exponentials are negligible.

### 5.4 The wireframe and the corners

The 12 cube edge tubes are combined with a **hard** min (so the three caps at a
corner coincide into one exact sphere, no smooth-min puff), then reshaped by the
**corner morph**, and only then fed into the crowding field as a single extra
contributor. The morph (`corner` ∈ [0,1], `cornerMorphFromSlider`) sculpts each
of the 8 corners along one axis:

- **`< 0.5` sharp** — union a power-norm "miter" solid (intersection of the three
  edge cylinders). Exponent runs from ~128 (crisp point) down to ~5.4 (where the
  solid vanishes inside the round). It protrudes along the body diagonal to
  `r·√1.5 / 3^(1/power)` (≈ 1.22·r at full sharp).
- **`= 0.5` round** — the natural radius-`r` sphere. No modification.
- **`> 0.5` flat** — a plane cut perpendicular to the diagonal (depth `Delta`),
  filleted at the rim (`Fillet`); slides from tangent (no cut) to a deep facet.

### 5.5 Outer edge clip

Left alone, webbing near the frame bulges past the cube's edges. The field is
therefore intersected with the cube's **outer skin** — a rounded box whose edges
*are* the 12 edge tubes (quarter-cylinders of radius `r`) and whose faces sit at
`half + r`:

```
D ← smooth-max( D, clip )            // smaxPoly, width = clipBlend
```

`smaxPoly ≥ hard-max`, so the edges are always fully clipped (never bulge past),
but the join is a **concave fillet** of width `clipBlend` instead of a crease.
`clipBlend = 0` gives a hard crease.

**Corner protection.** A ball of radius `vertexGuard` around the nearest corner
is *unioned into the clip solid* (smoothly, over `clipVBlend`) so the corner morph
governs the corners untouched, necking smoothly back into the clipped edge. The
guard is only needed where the morph pokes past the skin — i.e. the sharp miter —
so `vertexGuard` auto-resolves to `0` for round/flat corners and to `1.05 × the
miter tip` for sharp ones.

---

## [6] Marching cubes

`MarchingCubes` samples `D` on a `resolution³` grid over the padded bounding box
and extracts the zero isosurface with the standard Lorensen–Cline tables. Both
the sampling and the cell pass are parallelized across Z-slabs; per-vertex
normals come from the field gradient (central differences). Cost is ~cubic in
`resolution`.

## [7] Surface tension (optional)

`relax > 0` runs constrained mean-curvature flow on the extracted mesh (soap-film
tautening): weld coincident vertices, then per pass move each vertex a fraction
`lambda` toward its neighbours' average, reproject any vertex that fell inside a
tube back onto it (the tubes are the film's wire frame), and offset along normals
to conserve enclosed volume. `relax = 0` is off.

## [8] Output

The mesh is emitted either as a compact binary buffer for the WebGL viewer
(`uint32` vertex count + float32 positions/normals/colors, served at
`/api/mesh`) or as a binary **STL** (`stl.go`), with winding made consistently
outward via the signed volume.

---

## Print split

`engine/split_*.go` turns the skin into printable pieces. The skin is meshed
once and then cut like Blender's bisect: crossed triangles are split at the
plane, the section loop gets one copy of its vertices per side, and each side
is closed with a cap triangulated in the plane. Pieces are the connected
components left. Planning runs on a mesh at `planVoxelMM`; the export replays
the same cut program on a mesh at `voxelMM`.

1. **Graph** (`split_graph.go`). The 12 edges, the struts and the supports are
   tubes; tubes passing close enough for their skins to fuse share a junction.
   - A junction is a *vertex* where thick tubes meet (three arms, or two from
     different tubes) or where two or more supports land.
   - Thick tubes are split only at vertices. A support landing on a tube
     mid-span doesn't split the tube; the support is cut near it instead.
2. **Cut distances.** Each branch's skin radius is probed around its axis (12
   directions, every 0.12·r), walking out from the junction past the other
   tubes' cylinders. The branch piece stands on one end:
   - **Top** end: the first spot where, for the next 1.5·r, the skin widens by
     at most `tan(strutTilt)` per unit height toward the joint (`jointTilt` for
     supports: their fillet is joint geometry).
   - **Bottom** end: the section may sit deeper in the flare (walls leaning in
     going up print fine) while it stays a clean blob of at most 1.6·r (3·r for
     supports).
   - Branches too short to stand alone are absorbed into their joint; supports
     too short for two cuts get one across their middle.
3. **Fitting on the mesh** (`split_plan.go`, `split_mesh.go`). Each cut's
   section is the loop around its axis point. It must stay narrow, contain no
   other tube's axis, and not touch any other cut's section (checked where the
   two planes meet). Each branch piece must come off on its own. A cut failing
   any of these moves away from its joint.
4. **Atoms and orientation.** After all branch cuts, each connected component
   is an atom. A piece standing on one of its cut faces is scored by the area of
   downward-facing surface steeper than its limit: 15° on struts, 35° on joints
   and around support stubs, ignoring the first 1.5 cells above the bed.
   - A joint prints whole on its best face if that's clean.
   - Otherwise one split plane is searched: 500 directions over the hemisphere ×
     5 offsets, then local refinement, scoring both halves standing on the cut.
     Slicing through a cut face is penalised. Every loop the plane leaves must be
     a real face (≥ 8 mm²).
5. **Merge.** Neighbouring atoms are greedily united (their cut undone) while
   the union still prints on one of its outer faces. Thin cuts go first, then the
   smallest faces.
6. **Rods and numbers** (`split_cuts.go`, `split_glyph.go`). Each remaining cut
   is laid out on its exact 2D outline, with a clearance grid from a distance
   transform.
   - The main rod goes where `footprint + wall` fits, otherwise the thin rod;
     a second rod is added as far from the first as still fits. Each socket is
     checked in 3D on both sides (inside the skin, crossing no other cut except
     a split seam on its own face).
   - The rod is a trapezoid prism tapering to `taper` at both ends about its
     flat base. The rod is its socket's section inset by `clearanceMM` on every
     side (base, top, both slopes); the socket has an extra
     0.3 mm of depth for glue.
   - Numbers are drawn as round-capped strokes, outlined into exact contours
     (marching squares on the stroke distance, then simplified), 1–2 mm tall (strokes at least 0.25 mm)
     with an orientation bar, upright or turned 90°. The second face's copy is
     mirrored so each reads correctly.
   - A face with no room for a rod and its number together gets a dash key
     instead: the bar under the number widens into a fixed 1 × 3 mm groove,
     1.2 mm deep in both faces, and a loose key `clearanceMM` smaller per side sits half in
     each. Its grooves may not cross any other cut, split seams included.
   - Sockets and numbers become pockets: holes in the cap triangulation, with
     walls and a floor. Split faces are laid out last, on a mesh that already
     has the pockets of the faces they cross.
7. **Export** (`split_export.go`). The cut program replays on the export mesh:
   branch cuts, then each split plane on the component holding its joint (its
   loops are found again by position), then merged cuts are undone by welding
   their loop vertices back together. Shells are matched to planned pieces by
   their cut faces. The combined STL keeps every piece and rod in place; the tar
   holds each piece turned onto its print face.

### Print split parameters (`print` in the config)

| key | meaning |
|-----|---------|
| `cubeMM` | physical cube edge; sets the mm-per-unit scale |
| `strutTilt` / `jointTilt` | max overhang (° from vertical) on strut / joint pieces |
| `clearanceMM` | gap on every side of a rod or dash key in its socket (mm) |
| `planVoxelMM` / `voxelMM` | skin mesh cell for planning + preview / for export (cut faces are exact either way) |
| `minPieceMM` | shorter strut sections stay part of their joint |
| `merge`, `engrave`, `engraveDepthMM` | re-merge printable neighbours; number engraving |
| `mainRod`, `thinRod` | rod `baseMM`, `topMM`, `heightMM`, `lengthMM`, `taper`, `wallMM` |
| `explode` | view only: pull pieces apart |

## Parameters

Defaults live in `config.default.json`; `0.5` on a skin macro reproduces the
tuned reference look. Lengths are in world units (cube spans `cubeSize`).

### Structure — *which* struts, *how many* (you keep full control)

| key | meaning |
|-----|---------|
| `cubeSize` | size of the cube frame |
| `strutR` | strut / edge tube radius `r` — the skeleton width and the base scale for the skin |
| `buttressR` | thin secondary ("support") strut radius — part of the piece |
| `target` | number of struts to place |
| `minClearance` | min gap between struts, in units of `r` |
| `mix` | fraction of edge-to-edge struts vs free struts |
| `handshakeProb` | chance a strut becomes clasped hands |
| `neighborWeight` | preference for near vs far destination edges |
| `edgeBias` | attach near an edge's ends vs its middle |
| `lf100…lf70` | weights of the length-trim buckets (full → stub) |
| `vertexPresnapPct`, `edgeCylinderPct`, `wireSnapMult` | endpoint snap tolerances |
| `supportNum`, `supportJitter`, `apexSkipMult`, `supportMinLen` | support-web placement |

### Skin — three macros (with Advanced decoupling)

The macros are panel-side conveniences; the engine's real inputs are the
underlying values, which the macros rewrite together. Editing an Advanced value
decouples it. Formulas below scale with `r = strutR`.

**Skin thickness** `t` ∈ [0,1] → drives:

```
blendK = 0.05 · (r/0.085) · 2.5^(2t−1)      (reach k)
cap    = 0.12 · (r/0.085) · 2.8^(2t−1)      (swell limit B)
```

- Advanced: `blendK` (blend reach k), `cap` (joint swell limit B, ≤0 = off),
  `gamma` (crowd fill γ). `t=0.5` ⇒ `blendK=0.05`, `cap=0.12` at `r=0.085`.

**Edge roundness** `e` ∈ [0,1] → drives:

```
clipBlend = 0.8·r·e                 (e ≤ 0.5)     crisp → 0.4·r
          = r·(0.4 + 1.6·(e−0.5))   (e > 0.5)     0.4·r → 1.2·r soft
```

- Advanced: `clipBlend` (edge softening fillet), `sharpRatio` (blend smoothness
  `k'/k`). `e=0` is a crisp crease; `e=0.5` ⇒ `clipBlend = 0.4·r`.

**Corner cutoff** `corner` ∈ [0,1] — sharp (0) · round (0.5) · flat (1); see §5.4.

- Advanced: `fillet` (corner-ball radius; 0 = match `r`), `vertexGuard` (corner
  clip protection; `<0` = auto), `clipVBlend` (corner→edge neck width; `<0` =
  auto).

### Meshing / view

| key | meaning |
|-----|---------|
| `resolution` | live marching-cubes grid per axis (detail vs speed) |
| `exportResolution` | grid per axis for STL export |
| `padding` | empty margin around the sculpture in the sample box |
| `relax`, `lambda` | surface-tension passes and per-pass step (§7) |
| `mode` | `skin` (implicit sleeve) or `debug` (colored per-feature meshes) |
| `seed` | hex recipe number; empty ⇒ random on start |
| `autoSpin` | idle auto-rotation |

### How the skin knobs interact

- `strutR` is the master scale: the skin fillets/caps are ratios of it, so
  thicker struts get proportionally larger blends (the macros re-derive against
  the current `r`).
- **Thickness** vs **cap**: a large `blendK` reaches far to web struts together;
  `cap` bounds how fat the resulting joints get. Push thickness up and joints
  fill in until `cap` holds them.
- `gamma < 1` is what lets long edge fillets coexist with modest high-crowd
  joints — raise thickness for webbing without ballooning the busy centers.
- **Edge roundness** only affects where the sleeve meets the outer edges; it can
  never let material escape the cube's outer skin (the clip is ≥ a hard cut).
- **Corner cutoff** is independent of the edges; its auto guards keep the
  near-corner edge at the same width as the rest of the edge.
