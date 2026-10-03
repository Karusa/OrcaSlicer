# Strength check — High Level Design

## Purpose and scope

The strength check answers "will this printed part hold?" before the part is
printed. The user paints where the part is held (fixture) and where it is loaded,
aims the load with an arrow, and gets one of two answers:

- **Maximum load** (the default): the largest load the part takes at a chosen
  safety factor, 1.5 unless changed. Newcomers rarely know what load to expect.
- **Check a load**: the safety factor at a load entered in kg, lb or N. A safety
  factor below 1 flashes red.

Both come with the weakest cut, its stress, and the deflection at the load. The
part can be shown in stress colours and in its deflected shape, optionally
animated and exaggerated.

It is an estimate, not FEA. It is meant to catch the common failures of printed
parts: layers splitting under bending, too few walls or skins, sparse infill and
holes. The slicer already knows the part's orientation on the plate and how it
will be printed, so the check uses both.

The load case is a what-if check, not a print setting. It changes neither the
model, G-code nor the project file, and it never schedules background
processing.

## Components

- `libslic3r/Feature/StrengthAnalysis` holds the analysis. It has no GUI
  dependencies. Tests are in `tests/libslic3r/test_strength_analysis.cpp`, for the
  analysis itself, and `tests/fff_print/test_strength_analysis.cpp`, for sliced
  layers.
- `GLGizmoStrength` is a painter gizmo. The left button paints the fixture
  (`EnforcerBlockerType::ENFORCER`), the right button paints the load (`BLOCKER`),
  and Shift erases. The Face tool (smart fill) selects whole faces, which is what
  a fixture or a load usually covers.

## The printed part

The analysis works on a stack of `PrintedLayer`s in world coordinates. Each holds
the layer's outline and its fills, each fill an area with a density: 1 for walls
and solid infill, the sparse infill density elsewhere. There are two sources.

- **Sliced object.** `layers_from_print_object()` reads the `PrintObject` once
  `posPrepareInfill` is done. Walls are each region's slices minus its
  `fill_expolygons`. `stInternal` surfaces take the region's
  `sparse_infill_density`, `stInternalVoid` none, and every other fill surface
  counts as solid. Modifiers, height range modifiers, per-part settings, variable
  layer height and Orca's own solid infill decisions all come through this way.
  The gizmo uses it whenever the plate is sliced and not being re-sliced. Editing
  settings invalidates the step, so stale slices are never used.
- **Settings only.** `layers_from_mesh()` lays the mesh out as Orca would. It
  uses walls of `wall_loops` × inner wall line width. Skins go wherever the
  layers within `top_shell_layers` above or `bottom_shell_layers` below do not
  cover a point; each of those counts is raised to cover `top_shell_thickness`
  and `bottom_shell_thickness`. Sparse infill fills the rest. The object's own
  settings override the presets. The panel says which source was used, and
  suggests slicing to include modifiers.

## Load case state

Paint is held in the gizmo, keyed by `ModelObject` id, one
`TriangleSplittingData` per model part, together with the part's facet count.
Paint whose facet count no longer matches the mesh is dropped rather than
applied to different triangles. Paint, direction, load mode, load, unit and target
safety factor go through the gizmo's `on_save`/`on_load`, so undo and redo work
while the gizmo is open. Nothing is written to `ModelVolume`, so `.3mf` files,
slicing and the `FacetsAnnotation` users are untouched.

The arrow's tip sits on the centroid of the painted load area, and it points
along the force. Dragging the arrow swings its tail over a sphere around the
load point, like a trackball, with rotate-gizmo rings showing that sphere. The
direction snaps to an axis within 4°. Preset buttons and X/Y/Z fields set the
direction exactly.

Every result is linear in the load, so the analysis runs once at 1 N.
`scale_load()` turns it into either answer, so changing the load, the unit, the
mode or the target safety factor needs no new run. Changing the paint or the
direction marks the results out of date until the next Analyze.

## Analysis

Cuts are made between every pair of layers and on a regular grid normal to X and
Y. Only cuts that separate the fixture from the load are made. A cut separates
them when the fixture's extent along that axis lies entirely on one side of it
and the load point on the other. A fixture straddling a cut means the load
reaches ground without crossing it.

For each such cut the part beyond it is a free body, so the internal force is the
applied force and the internal moment is `r × F` about the cut's centroid. This
is exact for a statically determinate single load, whatever the shape of the
load path.

- **Cuts between layers** take the layer above's fills. Area, centroid and second
  moments come from Green's theorem over the fill loops, weighted by density.
- **Vertical cuts** are built from rectangles: where the cut line crosses each
  fill, times that layer's height. Edges are bucketed by the grid lines they
  cross, so the cost follows the edge count rather than edges × cuts.

So walls and skins sit where they are printed, at the outer fibres of the
section, instead of being smeared over it. Torsion uses a Saint-Venant solid
above 85% fill, and otherwise a Bredt closed shell from the solid area and the
section perimeter.

Stress at a cut is the axial stress, plus bending about both in-plane axes,
combined by von Mises with transverse shear (1.5 V/A) and torsional shear. The
two kinds of shear are added, which is conservative. Allowable stress is the
filament's tensile strength:

- times its layer-adhesion factor for a cut between layers
- times that factor plus 0.15 (at most 1) for a vertical cut, which runs across
  beads
- divided by the fatigue notch factor `Kf = 1 + q(Kt − 1)` where the cut passes
  a hole

`Kt` is Peterson's fit for a hole in a finite strip, measured from the ligament
to the outer contour. `q` falls from 1 for brittle filaments (elongation ≤ 5%)
to 0.4 for ductile ones. The cut with the lowest safety factor is reported.

## Deflected shape and stress colours

The deflected shape follows the axis on which the load sits furthest from the
fixture. Curvature `M / EI` at each cut along that axis is integrated from the
fixture outwards into a rotation and a displacement of the section centroids.
Past the last cut the part moves rigidly. A point of the part is displaced with
its section: `w + θ × (p − c)`. The deflection reported at the load is the same
integral, so the number and the drawn shape agree.

Stress colours give each point the utilization (stress over allowable) of the
most loaded cut through it. Blue is unloaded and red is at the breaking point.
The gizmo draws the result mesh as eight colour bands with the existing
`gouraud_light` shader. When animated, it rebuilds the displaced mesh only while
the animation phase changes, and requests frames only while animating or
flashing.

## Materials

`find_material()` maps a profile's `filament_type` to strength, modulus,
layer-adhesion factor and elongation. The values are typical of reasonably tuned
prints. Unknown types fall back to PLA, and the results panel says so.

## Limits

Beam theory assumes slender load paths and one load. Stocky blocks, contact
stresses, buckling, creep and print defects are out of scope. Bead orientation
within a layer and the infill pattern are not modelled; sparse infill counts by
density alone. An L-shaped path is handled for stress, but its deflected shape
follows only the longest run, and twist is not drawn.
