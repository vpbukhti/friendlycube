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

## Configuration

All parameters live in one JSON config. On start the program loads
`config.default.json` (override with `-config path.json`); any key that is
**absent or `null`** falls back to the baked-in default, so a config file only
needs to list what it changes.

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
| `-addr` | `127.0.0.1:8730` | address for the panel + WebGL server |
| `-open` | `true` | open the panel in the default browser on start |

## Layout

```
main.go                  flags, config load, headless export, server start
internal/engine/         the geometry + generation engine (physics)
  topology · struts · joints · supports · scene · skin   generation
  sdf · mesh · mc · relax · stl · math                   implicit field + meshing
internal/control/        the "control center": Config, web panel, mesh server
config.default.json      all baked-in defaults
PIPELINE.md              pipeline, math, and parameter reference
organic_sleeve_method.md the original write-up of the implicit-sleeve method
```
