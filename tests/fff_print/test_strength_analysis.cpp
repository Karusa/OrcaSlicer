#include <catch2/catch_all.hpp>

#include "libslic3r/Feature/StrengthAnalysis/StrengthAnalysis.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;
using namespace Slic3r::Feature::StrengthAnalysis;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double LENGTH = 100., WIDTH = 10., HEIGHT = 10., LOAD = 10.;

// A LENGTH x WIDTH x HEIGHT bar, 0.2 mm layers, 2 walls, 3 top and bottom layers.
void slice_bar(Print &print, Model &model, const std::string &infill)
{
    init_print({make_cube(LENGTH, WIDTH, HEIGHT)}, print, model,
               {{"layer_height", 0.2},
                {"initial_layer_print_height", 0.2},
                {"wall_loops", 2},
                {"top_shell_layers", 3},
                {"bottom_shell_layers", 3},
                {"top_shell_thickness", 0},
                {"bottom_shell_thickness", 0},
                {"sparse_infill_density", infill}});
    print.process();
}

indexed_triangle_set world_mesh(const PrintObject &object)
{
    const ModelObject   *mo  = object.model_object();
    indexed_triangle_set its = mo->volumes.front()->mesh().its;
    its_transform(its, mo->instances.front()->get_matrix() * mo->volumes.front()->get_matrix());
    return its;
}

// Held on its -X end face, pushed down on its +X end face.
LoadCase cantilever(const indexed_triangle_set &bar)
{
    const BoundingBoxf3 bbox = bounding_box(bar);
    auto end_face = [&bar](float x) {
        indexed_triangle_set out;
        out.vertices = bar.vertices;
        for (const Vec3i32 &f : bar.indices)
            if (std::all_of(f.begin(), f.end(), [&](int v) { return std::abs(bar.vertices[v].x() - x) < 1e-3f; }))
                out.indices.push_back(f);
        return out;
    };
    return {end_face(float(bbox.min.x())), end_face(float(bbox.max.x())), Vec3d(0., 0., -LOAD)};
}

PrintedLayers sliced_layers(const Print &print)
{
    const PrintObject &object = *print.objects().front();
    return layers_from_print_object(object, object.instances().front(), bounding_box(world_mesh(object)).min.z());
}

double area_at_density(const PrintedLayer &layer, double density)
{
    double area = 0.;
    for (const LayerFill &fill : layer.fills)
        if (std::abs(fill.density - density) < 1e-6)
            for (const ExPolygon &ex : fill.area)
                area += unscaled<double>(unscaled<double>(ex.area()));
    return area;
}

} // namespace

TEST_CASE("Sliced layers count walls and skins as solid and sparse infill at its density", "[StrengthAnalysis]")
{
    Print print;
    Model model;
    slice_bar(print, model, "15%");
    const PrintedLayers layers = sliced_layers(print);

    // 10 mm at 0.2 mm layers.
    REQUIRE(layers.size() == 50);
    CHECK_THAT(layers.back().z_top - layers.front().z_bottom, WithinRel(HEIGHT, 1e-6));
    // The first layer is bottom skin; the middle one is mostly sparse infill inside the walls.
    CHECK_THAT(area_at_density(layers.front(), 0.15), WithinAbs(0., 1e-9));
    CHECK(area_at_density(layers[25], 0.15) > 0.5 * LENGTH * WIDTH);
    CHECK(area_at_density(layers[25], 1.) > 0.);
}

TEST_CASE("Denser infill from the slicer makes the part stronger", "[StrengthAnalysis]")
{
    Print sparse_print, dense_print;
    Model sparse_model, dense_model;
    slice_bar(sparse_print, sparse_model, "15%");
    slice_bar(dense_print, dense_model, "60%");
    const LoadCase lc     = cantilever(world_mesh(*sparse_print.objects().front()));
    const Result   sparse = analyse(sliced_layers(sparse_print), lc, default_material());
    const Result   dense  = analyse(sliced_layers(dense_print), lc, default_material());
    REQUIRE(sparse.ok());
    REQUIRE(dense.ok());
    CHECK(dense.safety_factor > sparse.safety_factor);
    CHECK(dense.deflection < sparse.deflection);
}

TEST_CASE("Estimate from settings agrees with the sliced layers", "[StrengthAnalysis]")
{
    Print print;
    Model model;
    slice_bar(print, model, "15%");
    const indexed_triangle_set bar = world_mesh(*print.objects().front());
    PrintSettings              settings;
    settings.layer_height  = 0.2;
    settings.wall_loops    = 2;
    settings.top_layers    = 3;
    settings.bottom_layers = 3;
    settings.infill        = 0.15;
    const Result sliced    = analyse(sliced_layers(print), cantilever(bar), default_material());
    const Result estimated = analyse(layers_from_mesh(bar, settings), cantilever(bar), default_material());
    REQUIRE(sliced.ok());
    REQUIRE(estimated.ok());
    CHECK_THAT(estimated.safety_factor, WithinRel(sliced.safety_factor, 0.25));
    CHECK_THAT(estimated.deflection, WithinRel(sliced.deflection, 0.25));
}
