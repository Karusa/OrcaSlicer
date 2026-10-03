#include <catch2/catch_all.hpp>

#include "libslic3r/Feature/StrengthAnalysis/StrengthAnalysis.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using namespace Slic3r::Feature::StrengthAnalysis;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// Facets of `its` lying in the plane coordinate[axis] == value, as if painted.
indexed_triangle_set face(const indexed_triangle_set &its, int axis, float value)
{
    indexed_triangle_set out;
    out.vertices = its.vertices;
    for (const Vec3i32 &f : its.indices)
        if (std::all_of(f.begin(), f.end(), [&](int v) { return std::abs(its.vertices[v][axis] - value) < 1e-4f; }))
            out.indices.push_back(f);
    return out;
}

constexpr double LENGTH = 100., WIDTH = 10., HEIGHT = 10., LOAD = 10.;

// A WIDTH x HEIGHT bar along X, held at x = 0 and pushed down at x = LENGTH.
LoadCase cantilever(const indexed_triangle_set &bar, double load = LOAD)
{
    return {face(bar, 0, 0.f), face(bar, 0, float(LENGTH)), Vec3d(0., 0., -load)};
}

PrintSettings solid()
{
    PrintSettings p;
    p.infill = 1.;
    return p;
}

Result analyse_bar(const PrintSettings &print, double load = LOAD)
{
    const indexed_triangle_set bar = its_make_cube(LENGTH, WIDTH, HEIGHT);
    return analyse(layers_from_mesh(bar, print), cantilever(bar, load), *find_material("PLA"));
}

} // namespace

TEST_CASE("Cantilever stress and deflection follow beam theory", "[StrengthAnalysis]")
{
    const Material &pla = *find_material("PLA");
    const Result    res = analyse_bar(solid());
    REQUIRE(res.ok());

    // Critical cut next to the fixture, where the moment arm is longest.
    CHECK(res.axis == 0);
    CHECK(res.mode == Mode::Bending);
    CHECK_THAT(res.critical_point.x(), WithinAbs(0., 0.02 * LENGTH));
    const double sigma = LOAD * (LENGTH - res.critical_point.x()) / (WIDTH * HEIGHT * HEIGHT / 6.);
    CHECK_THAT(res.stress, WithinRel(sigma, 0.02));
    // A vertical cut runs along the beads, so it keeps more than layer adhesion alone.
    CHECK_THAT(res.allowable, WithinRel(pla.uts * (pla.z_factor + 0.15), 1e-6));
    CHECK_THAT(res.safety_factor, WithinRel(res.allowable / res.stress, 1e-9));

    const double I = WIDTH * HEIGHT * HEIGHT * HEIGHT / 12.;
    CHECK_THAT(res.deflection, WithinRel(LOAD * LENGTH * LENGTH * LENGTH / (3. * pla.modulus * I), 0.05));
}

TEST_CASE("Sparse infill weakens a part", "[StrengthAnalysis]")
{
    PrintSettings sparse;
    sparse.wall_loops = 2;
    sparse.infill     = 0.15;
    const Result full  = analyse_bar(solid());
    const Result light = analyse_bar(sparse);
    REQUIRE(full.ok());
    REQUIRE(light.ok());
    CHECK(light.safety_factor < full.safety_factor);
    CHECK(light.deflection > full.deflection);
}

TEST_CASE("Top and bottom skins carry bending", "[StrengthAnalysis]")
{
    // Bending a flat bar puts its top and bottom skins furthest from the neutral axis.
    PrintSettings bare;
    bare.wall_loops    = 1;
    bare.infill        = 0.;
    bare.top_layers    = 0;
    bare.bottom_layers = 0;
    PrintSettings skinned  = bare;
    skinned.top_layers    = 5;
    skinned.bottom_layers = 5;
    const Result without = analyse_bar(bare);
    const Result with    = analyse_bar(skinned);
    REQUIRE(without.ok());
    REQUIRE(with.ok());
    CHECK(with.safety_factor > 1.5 * without.safety_factor);
}

TEST_CASE("Fill densities of the printed layers drive the result", "[StrengthAnalysis]")
{
    // Sliced layers carry whatever modifiers decided; here every sparse fill is made solid.
    const indexed_triangle_set bar = its_make_cube(LENGTH, WIDTH, HEIGHT);
    PrintSettings              sparse;
    sparse.infill        = 0.1;
    PrintedLayers layers = layers_from_mesh(bar, sparse);
    const Result  before = analyse(layers, cantilever(bar), default_material());
    for (PrintedLayer &layer : layers)
        for (LayerFill &fill : layer.fills)
            fill.density = 1.;
    const Result after = analyse(layers, cantilever(bar), default_material());
    const Result solid_bar = analyse_bar(solid());
    REQUIRE(before.ok());
    REQUIRE(after.ok());
    CHECK(after.safety_factor > before.safety_factor);
    CHECK_THAT(after.safety_factor, WithinRel(solid_bar.safety_factor, 1e-6));
}

TEST_CASE("Load across layers is limited by layer adhesion", "[StrengthAnalysis]")
{
    // The same bar standing up: held at the bottom, pushed sideways at the top.
    const indexed_triangle_set post = its_make_cube(WIDTH, HEIGHT, LENGTH);
    const LoadCase             lc{face(post, 2, 0.f), face(post, 2, float(LENGTH)), Vec3d(LOAD, 0., 0.)};
    const Material            &pla = *find_material("PLA");
    const Result               res = analyse(layers_from_mesh(post, solid()), lc, pla);
    REQUIRE(res.ok());
    CHECK(res.axis == 2);
    CHECK_THAT(res.allowable, WithinRel(pla.uts * pla.z_factor, 1e-6));
}

TEST_CASE("Deflected shape follows the load from a still fixture", "[StrengthAnalysis]")
{
    const Result res = analyse_bar(solid());
    REQUIRE(res.ok());
    REQUIRE(res.run_axis == 0);
    const Vec3d tip = displacement_at(res, res.load_point);
    CHECK(tip.z() < 0.);
    CHECK_THAT(-tip.z(), WithinRel(res.deflection, 1e-6));
    CHECK_THAT(displacement_at(res, Vec3d(0., WIDTH / 2., HEIGHT / 2.)).norm(), WithinAbs(0., 1e-9));
    // Deflection grows towards the load.
    CHECK(displacement_at(res, Vec3d(LENGTH / 2., WIDTH / 2., HEIGHT / 2.)).z() > tip.z());
}

TEST_CASE("Results scale linearly with the load", "[StrengthAnalysis]")
{
    Result scaled = analyse_bar(solid());
    scale_load(scaled, 2.);
    const Result doubled = analyse_bar(solid(), 2. * LOAD);
    REQUIRE(doubled.ok());
    CHECK_THAT(scaled.safety_factor, WithinRel(doubled.safety_factor, 1e-9));
    CHECK_THAT(scaled.deflection, WithinRel(doubled.deflection, 1e-9));
    CHECK_THAT(displacement_at(scaled, scaled.load_point).z(), WithinRel(displacement_at(doubled, doubled.load_point).z(), 1e-9));
}

TEST_CASE("Stress colouring peaks at the critical cut", "[StrengthAnalysis]")
{
    const Result res = analyse_bar(solid());
    REQUIRE(res.ok());
    CHECK_THAT(utilization_at(res, res.critical_point), WithinRel(1. / res.safety_factor, 1e-6));
    CHECK(utilization_at(res, Vec3d(0.9 * LENGTH, WIDTH / 2., HEIGHT / 2.)) < utilization_at(res, res.critical_point));
}

TEST_CASE("A load applied on the fixture stresses nothing", "[StrengthAnalysis]")
{
    // Pushed down on the very face it is held by: no cut separates the load from the fixture.
    const indexed_triangle_set bar    = its_make_cube(LENGTH, WIDTH, HEIGHT);
    const indexed_triangle_set bottom = face(bar, 2, 0.f);
    const LoadCase             lc{bottom, bottom, Vec3d(0., 0., -LOAD)};
    CHECK_FALSE(analyse(layers_from_mesh(bar, solid()), lc, default_material()).ok());
}

TEST_CASE("Unpainted fixture or load is reported", "[StrengthAnalysis]")
{
    const indexed_triangle_set bar    = its_make_cube(LENGTH, WIDTH, HEIGHT);
    const PrintedLayers        layers = layers_from_mesh(bar, solid());
    LoadCase                   lc     = cantilever(bar);
    lc.fixture.indices.clear();
    CHECK_FALSE(analyse(layers, lc, default_material()).ok());
    lc = cantilever(bar);
    lc.load.indices.clear();
    CHECK_FALSE(analyse(layers, lc, default_material()).ok());
}

TEST_CASE("Filament types resolve to material properties", "[StrengthAnalysis]")
{
    REQUIRE(find_material("petg-cf") != nullptr);
    CHECK(std::string(find_material("petg-cf")->name) == "PETG-CF");
    CHECK(std::string(find_material("PA6-CF")->name) == "PA-CF");
    CHECK(find_material("unobtainium") == nullptr);
}

TEST_CASE("Hole stress concentration follows Peterson", "[StrengthAnalysis]")
{
    CHECK_THAT(kt_hole(0.01, 100.), WithinAbs(3., 0.01));
    CHECK_THAT(kt_hole(5., 10.), WithinRel(3. - 3.13 * 0.5 + 3.66 * 0.25 - 1.53 * 0.125, 1e-9));
    CHECK(kt_hole(0., 10.) == 1.);
    CHECK(notch_sensitivity(4.) == 1.);
    CHECK(notch_sensitivity(400.) == 0.4);
}
