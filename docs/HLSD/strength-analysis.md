# Strength check — High Level Design

## Purpose and scope

The strength check answers "will this printed part hold this load?" before the
part is printed. The user paints where the part is held (fixture) and where the
load is applied, aims the load with an arrow, enters it in kg, lb or N, and gets
a safety factor, the weakest cut, its stress and the deflection at the load.

It is an estimate, not FEA. It is meant to catch the common failures of printed
parts: layers splitting under bending, too few walls, sparse infill and holes.
The slicer already knows the part's orientation on the plate and its print
settings, so the check uses both instead of asking the user for them.

The load case is a what-if check, not a print setting. It changes neither the
model, G-code nor the project file, and it never schedules background
processing.

## Components

- `libslic3r/Feature/StrengthAnalysis` holds the analysis: `analyse()` takes the
  object mesh and painted facets in world coordinates, a force vector, material
  properties and the print settings that matter. It has no GUI dependencies, and
  it is covered by `tests/libslic3r/test_strength_analysis.cpp`.
- `GLGizmoStrength` is a painter gizmo. The left button paints the fixture
  (`EnforcerBlockerType::ENFORCER`), the right button paints the load (`BLOCKER`),
  and Shift erases. The Face tool (smart fill) selects whole faces, which is what
  a fixture or a load usually covers.

## Load case state

Paint is held in the gizmo, keyed by `ModelObject` id, one
`TriangleSplittingData` per model part, together with the part's facet count.
Paint whose facet count no longer matches the mesh is dropped rather than
applied to different triangles. Paint, direction, magnitude and unit go through
the gizmo's `on_save`/`on_load`, so undo and redo work while the gizmo is open.
Nothing is written to `ModelVolume`, so `.3mf` files, slicing and the
`FacetsAnnotation` users are untouched.

The arrow's tip sits on the centroid of the painted load area, and it points
along the force. Dragging the arrow swings its tail over a sphere around the
load point, like a trackball, with rotate-gizmo rings showing that sphere. The
direction snaps to an axis within 4°. Preset buttons and X/Y/Z fields set the
direction exactly. Any edit after a run marks the results out of date until the
next Analyze.

## Analysis

The object is cut by planes normal to X, Y and Z at layer-height spacing, using
`slice_mesh_ex` on a cyclically permuted copy of the mesh for X and Y. A cyclic
permutation keeps handedness, so contour orientation stays valid. Only cuts that
separate the fixture from the load are made. A cut separates them when the
fixture's extent along that axis lies entirely on one side of it and the load
point on the other. A fixture straddling a cut means the load reaches ground
without crossing it.

For each such cut the part beyond it is a free body, so the internal force is the
applied force and the internal moment is `r × F` about the cut's centroid. This
is exact for a statically determinate single load, whatever the shape of the
load path. The cut's area, centroid, second moments and extreme fibre distances
come from Green's theorem over the ExPolygon loops.

A mesh is solid but a print is not. Each cut is filled as printed: wall loops of
the inner wall line width around its boundary, and sparse infill density inside.
Area and second moments are scaled by that fraction. Torsion uses a
Saint-Venant solid above 85% fill and a Bredt closed shell below.

Stress at a cut is the axial stress, plus bending about both in-plane axes,
combined by von Mises with transverse shear (1.5 V/A) and torsional shear. The
two kinds of shear are added, which is conservative. Allowable stress is the
filament's tensile strength:

- times its layer-adhesion factor for a Z cut, which runs between layers
- times that factor plus 0.15 (at most 1) for an X or Y cut, which runs across
  beads
- divided by the fatigue notch factor `Kf = 1 + q(Kt − 1)` where the cut passes
  a hole

`Kt` is Peterson's fit for a hole in a finite strip, measured from the ligament
to the outer contour. `q` falls from 1 for brittle filaments (elongation ≤ 5%)
to 0.4 for ductile ones. The cut with the lowest safety factor is reported.

Deflection at the load point uses the unit-load method, integrating
`m·M / EI` along the axis on which the load sits furthest from the fixture.

## Materials

`find_material()` maps a profile's `filament_type` to strength, modulus,
layer-adhesion factor and elongation. The values are typical of reasonably tuned
prints. Unknown types fall back to PLA, and the results panel says so. The
object's own settings override the presets, as they do when slicing.

## Limits

Beam theory assumes slender load paths and one load. Stocky blocks, contact
stresses, buckling, creep and print defects are out of scope. An L-shaped path
is handled for stress, but its deflection counts only the longest run.
