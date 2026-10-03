#pragma once

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"

#include <admesh/stl.h>

#include <optional>
#include <string>
#include <vector>

// Quick strength estimate of a printed part under one painted load case.
//
// This is beam theory on the part's printed cross-sections, not FEA: every cut that
// separates the painted fixture from the painted load carries the load's force and
// moment, and its stress is checked against the strength of the material printed
// there. The part is described as a stack of printed layers, taken from the sliced
// object when there is one, so modifiers and solid infill count.
// See docs/HLSD/strength-analysis.md.
namespace Slic3r {
class PrintObject;
struct PrintInstance;
} // namespace Slic3r

namespace Slic3r::Feature::StrengthAnalysis {

struct Material
{
    const char *name;
    double      uts;        // MPa, tensile strength along the beads
    double      modulus;    // MPa
    double      z_factor;   // strength across layers as a fraction of uts
    double      elongation; // % at break; low means brittle and notch sensitive
};

// Properties for a slicer filament_type such as "PETG-CF", or nullptr if unknown.
const Material *find_material(const std::string &filament_type);
const Material &default_material();

// Area of a layer printed at one density: 1 for walls and solid infill, the sparse
// infill density elsewhere.
struct LayerFill
{
    ExPolygons area;
    double     density;
};

// One printed layer, in world coordinates.
struct PrintedLayer
{
    double            z_bottom = 0.;
    double            z_top    = 0.;
    ExPolygons        outline;
    std::vector<LayerFill> fills;
};
using PrintedLayers = std::vector<PrintedLayer>;

// Settings used to lay out a mesh as Orca would print it, when there are no sliced layers.
struct PrintSettings
{
    double layer_height  = 0.2;  // mm
    double wall_width    = 0.45; // mm per wall loop
    int    wall_loops    = 2;
    int    top_layers    = 3;
    int    bottom_layers = 3;
    double infill        = 0.15; // sparse infill density, 0..1
};

// Walls inset from each layer's outline, solid skins within top_layers of an upward
// facing surface and bottom_layers of a downward facing one, sparse infill elsewhere.
PrintedLayers layers_from_mesh(const indexed_triangle_set &mesh, const PrintSettings &print);

// Layers of a sliced object, placed at one of its instances with the part's bottom at
// z_min. Needs posPrepareInfill done; walls and solid infill count as solid, sparse
// infill at its region's density, so modifiers and per-part settings are honoured.
PrintedLayers layers_from_print_object(const PrintObject &object, const PrintInstance &instance, double z_min);

struct LoadCase
{
    indexed_triangle_set fixture;                // painted fixture facets, world mm
    indexed_triangle_set load;                   // painted load facets, world mm
    Vec3d                force = Vec3d::Zero(); // N, world
};

enum class Mode { Bending, Tension, Shear, Torsion };

// One analysed cut: stress as a share of what the material there can take.
struct Cut
{
    int    axis;
    double pos;
    double utilization;
};

// The deflected beam axis, sampled at the cuts along the run it is integrated over.
struct BeamStation
{
    double pos;
    Vec3d  centroid;
    Vec3d  displacement; // mm
    Vec3d  rotation;     // rad, as a rotation vector
};

struct Result
{
    std::string error; // set when there is no answer, in user-facing wording

    double safety_factor  = 0.;
    double stress         = 0.; // von Mises at the critical cut, MPa
    double allowable      = 0.; // printed strength at that cut, MPa
    double kt             = 1.; // stress concentration at that cut
    Mode   mode           = Mode::Bending;
    int    axis           = 2;  // normal of the critical cut: 0 = X, 1 = Y, 2 = Z
    Vec3d  critical_point = Vec3d::Zero(); // centroid of the critical cut
    Vec3d  load_point     = Vec3d::Zero();
    double deflection     = 0.; // mm at the load point, along the load

    std::vector<Cut>         cuts;          // sorted by axis, then position
    int                      run_axis = -1; // axis the deflected shape follows
    std::vector<BeamStation> run;           // ordered from the fixture outwards

    bool ok() const { return error.empty(); }
};

Result analyse(const PrintedLayers &layers, const LoadCase &load_case, const Material &material);

// Everything in a Result is linear in the load: rescale it to `factor` times the load.
void scale_load(Result &result, double factor);

// Displacement of a world point of the part in the deflected shape, mm.
Vec3d displacement_at(const Result &result, const Vec3d &point);
// Utilization of the most loaded cut through a world point, 0 where nothing is stressed.
double utilization_at(const Result &result, const Vec3d &point);

// Area-weighted centroid of a set of facets, such as a painted load area.
std::optional<Vec3d> area_centroid(const indexed_triangle_set &its);

// Stress concentration of a circular hole of diameter d in a strip of width w (Peterson).
double kt_hole(double d, double w);
// Share of the geometric stress concentration a material actually feels.
double notch_sensitivity(double elongation);

} // namespace Slic3r::Feature::StrengthAnalysis
