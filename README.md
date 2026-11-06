# friendlycube

Procedurally grows an organic "marble lattice" inside a cube frame — a tangle of
struts (some clasped into handshakes) wrapped in one smooth, watertight skin —
and lets you shape it live in the browser or export it as an STL.

It's a single self-contained Go program: the geometry engine, an implicit-surface
mesher, and a small web server that hosts a WebGL viewer + control panel. No cgo,
no external services, no runtime dependencies.

For how it actually works — the generation pipeline, the implicit-sleeve math, and
every parameter — see **[PIPELINE.md](PIPELINE.md)**.

## Requirements

- Go 1.26+ (`go.mod` requires zero external modules; pure Go, no C toolchain).

## Run

```sh
go run .
```

It builds the default sculpture and opens `http://127.0.0.1:8730` in your browser:
the 3D view and the control panel live on the same page. (Pass `-open=false` to
skip auto-opening; the URL is always printed.)

Build a reusable binary instead:

```sh
go build -o friendlycube .
./friendlycube
```

## The live view

- **Drag** to tumble the sculpture (free rotation — no limits), **scroll** to zoom.
- Rotation has inertia: it tracks your pointer while dragging and keeps the
  momentum you fling it with on release, coasting to a stop. When idle it spins
  slowly on its own (toggle with **Auto-rotate**).
- Every slider has a **hover tooltip** (after a short delay) explaining, in plain
  language, what it does to the result.

## Controls

The panel is grouped into **Session**, **Structure**, **Strut length mix**,
**Skin**, and **Meshing**. Changes re-mesh live (debounced, so dragging a slider
doesn't thrash the renderer).

The three **Skin** controls are *macros* — each drives several underlying field
values together so you don't have to think about them individually:

- **Skin thickness** — lean/wiry ↔ plump/webbed
- **Edge roundness** — crisp outer edges ↔ softly rounded
- **Corner cutoff** — sharp point ↔ round ↔ flat facet at the 8 cube corners

Each has an **▸ Advanced** disclosure that decouples the underlying knobs for fine
control. The macro reproduces the tuned reference look at its midpoint. Full
details of every parameter are in [PIPELINE.md](PIPELINE.md#parameters).

Buttons: **Regenerate** (new random seed), **Save** / **Load** config, **Export
STL**.

## Frame editor

**Edit frame** switches the panel into an editor and the view into the colored
debug diagram, where you shape the strut skeleton by hand:

- **Click a strut** → edit its **width**, or **delete** it.
- **Click a joint** (an interior meeting point of struts) → move it. Dragging the
  joint moves *every* strut that shares it; **arrow keys** nudge it in small
  steps; dragging one of the colored **gizmo arrows** locks motion to that axis.
  The gizmo axes follow either the **cube** or the **camera** frame (toggle).
- **Click a strut, then one of its ends** → move just that one end, detaching it
  from any shared joint.
- Moving snaps to nearby joints, cube corners and cube edges (joints win over
  edges), can't leave the cube, and won't drop two joints on top of each other.
- **handshakes** are drawn as a separate overlay on top of the (watertight) body,
  and turn **red** when a strut is too short for one or another strut crowds it.
  Toggle their visibility; **Preview skin** re-meshes the finished look at any
  time. **Exit** leaves the editor.

The cube's 12 edges and 8 corners are the fixed frame — they're never editable.
Once you hand-edit, the explicit skeleton becomes the source of truth; changing a
structural parameter (or Regenerate) asks before discarding your edits.

## Print split

Mode **split** cuts the sculpture into pieces that each print without supports,
glued back together on keyed joints. How it works is in
[PIPELINE.md](PIPELINE.md#print-split).

- **Cuts are exact.** The skin is cut like Blender's bisect: each cut face is
  triangulated in the cut plane itself, so faces are flat with sharp rims.
  Every piece is one closed shell.
- **Struts and edges stay whole between vertices.** They are only cut near the
  places where thick tubes meet. A single support landing on a strut doesn't cut
  it: the support is cut off close to the strut and leaves a short stub.
- **Strut pieces** stand on one of their cuts. No surface leans out more than
  **15°** past vertical, so each cut sits just where a joint's flare begins.
- **Joint pieces** print whole on one of their faces when they can, with
  overhangs up to **35°**. If a joint can't print whole, it is cut in half,
  once, by the best plane found. The fillet where a support meets a tube counts
  as joint too (35°), which keeps support stubs short.
- **Merging:** neighbouring pieces are joined back together whenever the result
  still prints, so you glue fewer joints.
- **Guide rods:** every cut gets one or two rods. The main rod is a 4 mm
  trapezoid 8 mm long; a 2.9 mm rod 6 mm long is used where the main one
  doesn't fit.
  Rods leave 0.1 mm on every side in their sockets (adjustable), print lying on their flat side, and
  are interchangeable within each size.
- **Dash keys:** where a cut face has no room for a rod and its number
  together, the bar under the number becomes a groove (1 × 3 mm, 1.2 mm deep)
  in both faces, and a loose key (0.8 × 2.8 × 2.0 mm, `key-dash.stl`)
  sits half in each. All dash keys are the same size.
- **Numbers:** each cut's number is engraved into both of its faces, as vector
  outlines cut into the face. The bar under a number marks its bottom edge.
  Where a joint's split plane crosses a cut face, the rod and number may
  straddle the seam (glue the two halves first). Every joint gets a number;
  `parts.txt` lists them all too.

In the view, pieces are see-through and the rods inside them are solid. Hover a
piece to bring it forward and list its cuts; **Explode** pulls the pieces apart.
Pieces that can't be made printable within the limits are drawn red.

**Export pieces** downloads two files:
- `lattice-<seed>-split.stl`: every piece where it sits in the cube, with the
  rods in their sockets, as separate bodies in one file. Arranging them on the
  bed is left to the slicer.
- `lattice-<seed>-split.tar`: one STL per numbered piece, each standing on its
  print face at z = 0, the rod and dash key STLs and `parts.txt`, which lists every piece
  with its cuts, neighbours and rods.

At the default 150 mm cube expect about 80 pieces and 110 glued joints, and an
export of about a minute. Raising **Strut overhang °** or lowering **Strut
count** / support settings reduces both.

## Configuration

All parameters live in one JSON config, split into two halves: **`skin`** (the
implicit-sleeve + meshing/view knobs) and **`frame`** (the procedural generation
params *and* the baked skeleton). Top-level `mode` / `seed` / `autoSpin` sit
outside both. On start the program loads `config.default.json` (override with
`-config path.json`); any key that is **absent or `null`** falls back to the
baked-in default, so a config file only needs to list what it changes. Older
flat configs (all keys at the top level) are migrated automatically on load.

The `frame` holds a `generatedSkeleton` — the explicit struts/supports/handshakes
baked from `(params, seed)`, cached on first build — and, once you use the
**Frame editor**, a `modifiedSkeleton` that takes over as the source of truth.
**Save** writes the whole thing (both halves plus the skeleton) so hand-edits
survive a reload.

- `config.default.json` at the repo root holds every default value.
- **Save** in the panel writes the current settings to a file; **Load** reads one
  back and re-meshes.
- The `-seed` flag overrides the config's seed (same seed ⇒ identical sculpture).

## Headless export

Skip the window entirely and write an STL:

```sh
./friendlycube -stl out.stl                 # default config, at export resolution
./friendlycube -stl out.stl -seed 1a2b3c    # a specific sculpture
./friendlycube -stl out.stl -config mine.json
```

## Flags

| flag | default | meaning |
|------|---------|---------|
| `-config` | `config.default.json` | JSON config path (absent/`null` keys → baked-in defaults) |
| `-seed` | *(empty)* | hex seed override, e.g. `1a2b3c`; empty = use config, else random |
| `-stl` | *(empty)* | if set, write an STL to this path (headless) and exit |
| `-split` | *(empty)* | if set, split into printable pieces, write `<path>.stl` + `<path>.tar`, and exit |
| `-addr` | `127.0.0.1:8730` | address for the panel + WebGL server |
| `-open` | `true` | open the panel in the default browser on start |

## Layout

```
main.go                  flags, config load, headless export, server start
internal/engine/         the geometry + generation engine (physics)
  topology · struts · joints · supports · scene · skin   generation
  bake · skeleton                                        editable frame (bake + edit ops)
  sdf · mesh · mc · relax · stl · math                   implicit field + meshing
internal/control/        the "control center": Config, web panel + frame editor, mesh server
config.default.json      all baked-in defaults
PIPELINE.md              pipeline, math, and parameter reference
organic_sleeve_method.md the original write-up of the implicit-sleeve method
```
