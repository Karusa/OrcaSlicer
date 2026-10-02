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

// A WIDTH x HEIGHT solid bar along X, held at x = 0 and pushed down at x = LENGTH.
LoadCase cantilever(const indexed_triangle_set &bar)
{
    return {face(bar, 0, 0.f), face(bar, 0, float(LENGTH)), Vec3d(0., 0., -LOAD)};
}

PrintSettings solid()
{
    PrintSettings p;
    p.infill = 1.;
    return p;
}

} // namespace

TEST_CASE("Cantilever stress and deflection follow beam theory", "[StrengthAnalysis]")
{
    const indexed_triangle_set bar = its_make_cube(LENGTH, WIDTH, HEIGHT);
    const Material            &pla = *find_material("PLA");
    const Result               res = analyse(bar, cantilever(bar), pla, solid());
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
    const indexed_triangle_set bar = its_make_cube(LENGTH, WIDTH, HEIGHT);
    PrintSettings              sparse;
    sparse.wall_loops = 2;
    sparse.infill     = 0.15;
    const Result full  = analyse(bar, cantilever(bar), default_material(), solid());
    const Result light = analyse(bar, cantilever(bar), default_material(), sparse);
    REQUIRE(full.ok());
    REQUIRE(light.ok());
    CHECK(light.safety_factor < full.safety_factor);
    CHECK(light.deflection > full.deflection);
}

TEST_CASE("Load across layers is limited by layer adhesion", "[StrengthAnalysis]")
{
    // The same bar standing up: held at the bottom, pushed sideways at the top.
    const indexed_triangle_set post = its_make_cube(WIDTH, HEIGHT, LENGTH);
    const LoadCase             lc{face(post, 2, 0.f), face(post, 2, float(LENGTH)), Vec3d(LOAD, 0., 0.)};
    const Material            &pla = *find_material("PLA");
    const Result               res = analyse(post, lc, pla, solid());
    REQUIRE(res.ok());
    CHECK(res.axis == 2);
    CHECK_THAT(res.allowable, WithinRel(pla.uts * pla.z_factor, 1e-6));
}

TEST_CASE("A load applied on the fixture stresses nothing", "[StrengthAnalysis]")
{
    // Pushed down on the very face it is held by: no cut separates the load from the fixture.
    const indexed_triangle_set bar    = its_make_cube(LENGTH, WIDTH, HEIGHT);
    const indexed_triangle_set bottom = face(bar, 2, 0.f);
    const LoadCase             lc{bottom, bottom, Vec3d(0., 0., -LOAD)};
    CHECK_FALSE(analyse(bar, lc, default_material(), solid()).ok());
}

TEST_CASE("Unpainted fixture or load is reported", "[StrengthAnalysis]")
{
    const indexed_triangle_set bar = its_make_cube(LENGTH, WIDTH, HEIGHT);
    LoadCase                   lc  = cantilever(bar);
    lc.fixture.indices.clear();
    CHECK_FALSE(analyse(bar, lc, default_material(), solid()).ok());
    lc = cantilever(bar);
    lc.load.indices.clear();
    CHECK_FALSE(analyse(bar, lc, default_material(), solid()).ok());
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
