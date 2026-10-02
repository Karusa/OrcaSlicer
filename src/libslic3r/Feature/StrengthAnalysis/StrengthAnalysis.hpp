#pragma once

#include "libslic3r/Point.hpp"

#include <admesh/stl.h>

#include <optional>
#include <string>

// Quick strength estimate of a printed part under one painted load case.
//
// This is beam theory on the part's real cross-sections, not FEA: every cut that
// separates the painted fixture from the painted load carries the load's moment
// about the cut's centroid, and its stress is checked against the strength of a
// printed section (walls + infill, weaker across layers, knocked down at holes).
// See docs/HLSD/strength-analysis.md.
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

struct PrintSettings
{
    double layer_height = 0.2;  // mm, also the spacing of the analysed cuts
    double wall_width   = 0.45; // mm per wall loop
    int    wall_loops   = 2;
    double infill       = 0.15; // sparse infill density, 0..1
};

struct LoadCase
{
    indexed_triangle_set fixture;                // painted fixture facets, world mm
    indexed_triangle_set load;                   // painted load facets, world mm
    Vec3d                force = Vec3d::Zero(); // N, world
};

enum class Mode { Bending, Tension, Shear, Torsion };

struct Result
{
    std::string error; // set when there is no answer, in user-facing wording

    double safety_factor = 0.;
    double stress        = 0.; // von Mises at the critical cut, MPa
    double allowable     = 0.; // printed strength at that cut, MPa
    double kt            = 1.; // stress concentration at that cut
    Mode   mode          = Mode::Bending;
    int    axis          = 2;  // normal of the critical cut: 0 = X, 1 = Y, 2 = Z
    Vec3d  critical_point = Vec3d::Zero(); // centroid of the critical cut
    Vec3d  load_point     = Vec3d::Zero();
    double deflection     = 0.; // mm at the load point, along the load

    bool ok() const { return error.empty(); }
};

// `mesh` is the whole object in world coordinates.
Result analyse(const indexed_triangle_set &mesh, const LoadCase &load_case, const Material &material, const PrintSettings &print);

// Area-weighted centroid of a set of facets, such as a painted load area.
std::optional<Vec3d> area_centroid(const indexed_triangle_set &its);

// Stress concentration of a circular hole of diameter d in a strip of width w (Peterson).
double kt_hole(double d, double w);
// Share of the geometric stress concentration a material actually feels.
double notch_sensitivity(double elongation);

} // namespace Slic3r::Feature::StrengthAnalysis
