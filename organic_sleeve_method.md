# Organic Sleeve Around a Tube Skeleton — Construction Guide

How to wrap a skeleton of straight tube segments in a smooth, watertight,
organic sleeve with controllable blending at the joints, and triangulate it
into an STL. Method: a shaped implicit scalar field, sampled on a grid, with
the surface extracted as its zero level set.

## Input

- A skeleton: a list of edges, each a segment from point **a** to point **b**.
- A tube radius `r` (may vary per edge).

## Step 1 — Per-edge signed distance

For a sample point **p** and an edge (**a**, **b**), compute the distance to
the segment:

```
t     = clamp( (p − a)·(b − a) / |b − a|² ,  0, 1 )
c     = a + t·(b − a)                 (closest point on the segment)
d(p)  = | p − c |
e(p)  = d(p) − r                      (signed distance to the tube surface)
```

`e < 0` inside the tube, `e = 0` on it, `e > 0` outside.

## Step 2 — Combine all edges into one field

For each sample point, with `e₁ … e_N` the values from Step 1 over all edges:

```
e_min = minᵢ eᵢ                                  (hard union of the tubes)

S     = Σᵢ exp( −(eᵢ − e_min) / k )              (effective contributor count)

x     = ln S                                     (crowding measure, ≥ 0)

dip   = k · x^γ

D(p)  = e_min − B · tanh( dip / B )
```

The sleeve surface is the level set **D = 0** (negative inside).

Interpretation of each piece:

- **S** counts how many tubes are "nearly tied" in distance at this point:
  exactly 1 next to a lone tube, ≈ n at a clean n-edge vertex, between 1 and 2
  along the wedge between two edges at a small angle. Distant tubes contribute
  essentially nothing (exponential falloff), so blending is local.
- **dip** is how far the surface is pushed beyond the plain union. On a lone
  tube `S = 1`, `dip = 0`, and the surface sits at exactly `d = r` — tube
  radius is exact and independent of all other parameters.
- **tanh cap**: `tanh` has slope 1 at zero and asymptote 1, so weak blends are
  untouched while the total push-out can never exceed `B`.

At a vertex where n edges meet, the surface sits at approximately

```
J ≈ r + B · tanh( k · (ln n)^γ / B )
```

which is the formula to use for predicting or pinning joint sizes.

## Step 3 — Parameters

| knob | role | guidance |
|------|------|----------|
| `r`  | tube radius, exact on lone spans | from your geometry |
| `k`  | blend strength and reach: how strongly a *pairwise* meeting of edges blends, and how far apart / how far along the tubes surfaces still merge | start at 0.5–2 × r; raise for long fillets and webbing between edges at small angles |
| `γ`  | crowding exponent: how blend depth grows with the number of contributing edges | 1 = classic logarithmic growth (`dip = k ln n`); 0.4–0.6 = each extra edge adds much less, so high-valence joints flatten into broad deltas while pairwise fillets keep full strength; > 1 exaggerates busy joints. Useful range ≈ 0.4–1.5 |
| `B`  | absolute cap on extra thickness anywhere | very large = effectively off; ≈ 0.5–1 × r to clamp joints while keeping long reach from a large `k` |

Recipe for "sleeve extends along the edges, joints stay modest": set
γ ≈ 0.4–0.5 first, then raise `k` — with low γ, `k` feeds the edge fillets
without the joints keeping pace.

To change `r` while keeping a chosen joint size `J` fixed, re-solve `k` from
the joint formula: `k = B · atanh( (J − r)/B ) / (ln n)^γ`, with n the
dominant vertex valence.

## Step 4 — Sample the field on a grid

1. Bounding box: the skeleton's axis-aligned bounds, padded on every side by
   at least ~1.5 × the largest expected sleeve radius (`r + B` is a safe
   estimate), plus a couple of grid cells.
2. Choose a uniform grid spacing `h`. Resolution of 60 samples along the
   longest axis is fine for previewing; 90+ for a final export. Cost is cubic
   in resolution and linear in edge count.
3. Evaluate `D` at every grid node (Steps 1–2). If memory is a concern,
   evaluate one z-slab at a time.

Numerical care: compute `S` with `e_min` already subtracted (as written
above) — every exponent is then ≤ 0, so nothing overflows, and terms with
`(eᵢ − e_min)/k > ~18` can be skipped as negligible.

## Step 5 — Extract the isosurface

Run any level-set mesher on the grid at level 0. Two standard choices:

- **Marching cubes** — per-cell lookup-table triangulation; available in most
  geometry libraries.
- **Naive surface nets** — simpler to implement from scratch: for every grid
  cell whose 8 corners are not all the same sign, place one vertex at the
  average of the sign-change crossings on its 12 edges; then for every grid
  edge whose two endpoint samples differ in sign, connect the vertices of the
  4 cells sharing that edge into a quad (two triangles). Produces slightly
  smoother meshes than marching cubes at the same resolution.

Both produce a closed, manifold mesh because the field is well defined
everywhere. Ensure consistent outward orientation: compute the mesh's signed
volume ( Σ over triangles of `a · (b × c) / 6` ); if negative, flip the
winding of every triangle.

Sanity checks worth automating: the mesh is watertight, and its Euler
characteristic matches the skeleton topology — for a connected frame with V
vertices and E edges, genus `g = E − V + 1` and `χ = 2 − 2g`.

## Step 6 — Optional: surface-tension pass

If tauter, concave fillets are wanted, relax the mesh with constrained mean
curvature flow (this is simulated surface tension — gradient descent on
surface area):

1. Per pass, move every vertex toward the average of its neighbors by a
   factor λ ≈ 0.5 (or use a cotangent-Laplacian implicit step for higher
   fidelity).
2. After every pass, project any vertex that fell inside a tube back out:
   find its closest point **c** on the skeleton; if `|p − c| < r`, set
   `p = c + r · (p − c)/|p − c|`. The tubes act as the film's wire frame and
   never shrink.
3. Optionally conserve enclosed volume ("water weight"): after each pass,
   measure the mesh volume, and offset all vertices along their normals by
   `(V₀ − V) / A_total` so material squeezed out of the bulges re-inflates
   the low-curvature spans instead of vanishing.
4. Stop after 10–30 passes for a subtle-to-strong effect. Many more passes
   will thin narrow membranes toward pinch-off; re-check watertightness
   afterwards if printing.

## Step 7 — Export STL

Binary STL layout: 80-byte header, uint32 triangle count, then per triangle
a float32 normal (3 values), three float32 vertices (9 values), and 2 padding
bytes — 50 bytes per triangle. Compute each normal as the normalized cross
product of the triangle's edge vectors, consistent with the outward winding
from Step 5.

## Pitfalls

- **Fused joints:** edges much shorter than `r + dip` let neighboring joints
  melt into one blob. Reduce `k` or `γ`, or thin those tubes.
- **Mixed valences:** the joint-size formula is per-valence; with both 3-way
  and 6-way vertices, one parameter set cannot pin both exactly — tune for
  the valence that matters most.
- **Extreme cap settings:** very small `B` with very large `k` flattens the
  field gradient at joints, which can read as a plateau. Substitute a gentler
  cap such as `dip / (1 + dip/B)`, or add a small residual slope
  `+ ε·dip` (ε ≈ 0.05).
- **Grid too coarse:** thin webbing needs a few cells across its thickness to
  survive extraction; if membranes come out perforated, increase resolution.

## Extension point

The crowding shape is one explicit function slot: the field is really
`D = e_min − k·φ(ln S)` with `φ(x) = x^γ` (capped). Any increasing φ with
`φ(0) = 0` is a valid drop-in — including non-monotonic profiles (e.g. peak
at two contributors, decay beyond) for fillets between edge pairs with nearly
bare high-valence vertices.
