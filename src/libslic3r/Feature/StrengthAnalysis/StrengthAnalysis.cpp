#include "StrengthAnalysis.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

namespace Slic3r::Feature::StrengthAnalysis {

namespace {

// Typical values for reasonably tuned prints. z_factor is what layer adhesion keeps.
constexpr std::array<Material, 22> MATERIALS{{
    {"PLA", 50, 3500, 0.55, 4},      {"PLA+", 45, 3000, 0.62, 8},    {"PLA-CF", 48, 5500, 0.42, 2},
    {"PETG", 45, 2100, 0.72, 12},    {"PETG-CF", 60, 5000, 0.50, 4}, {"ABS", 38, 2100, 0.45, 15},
    {"ASA", 42, 2200, 0.45, 14},     {"ABS-CF", 55, 5500, 0.35, 3},  {"PC", 62, 2300, 0.42, 8},
    {"PC-CF", 85, 6000, 0.35, 3},    {"PA6", 55, 1600, 0.55, 25},    {"PA12", 48, 1400, 0.60, 30},
    {"PA-CF", 95, 6500, 0.35, 4},    {"PA-GF", 80, 5000, 0.40, 5},   {"PPA-CF", 130, 9000, 0.35, 3},
    {"PET-CF", 90, 6000, 0.38, 3},   {"PP", 25, 1300, 0.60, 40},     {"TPU", 28, 60, 0.80, 400},
    {"HIPS", 25, 1900, 0.50, 20},    {"PEEK", 95, 4000, 0.45, 20},   {"PPS-CF", 90, 8000, 0.40, 3},
    {"PCTG", 45, 1900, 0.75, 15},
}};

// filament_type spellings found in the bundled profiles that share a row above.
constexpr std::array<std::pair<const char *, const char *>, 12> ALIASES{{
    {"PA", "PA6"},          {"PAHT", "PA6"},        {"PA-HT", "PA6"},   {"PA6-CF", "PA-CF"},
    {"PA12-CF", "PA-CF"},   {"PAHT-CF", "PA-CF"},   {"PA6-GF", "PA-GF"}, {"PPS", "PPS-CF"},
    {"ASA-CF", "ABS-CF"},   {"ABS-GF", "ABS-CF"},   {"TPU-AMS", "TPU"}, {"PLA-AERO", "PLA"},
}};

constexpr double SHEAR_PEAK        = 1.5;  // peak / mean transverse shear, rectangular section
constexpr double SOLID_THRESHOLD   = 0.85; // printed / enclosed area above which torsion sees a solid
constexpr double TRIM              = 0.01; // ends of the part left uncut, as a fraction of its span
constexpr int    MAX_CUTS_PER_AXIS = 600;
constexpr double MIN_CUT_STEP      = 0.2;  // mm between vertical cuts
constexpr int    HOLE_PROBE_LAYERS = 12;
constexpr double MIN_HOLE_RADIUS   = 0.3;
constexpr double SOLID_DENSITY     = 0.999;

struct Section
{
    int    axis = 2;
    double pos  = 0.;
    Vec3d  centroid;
    double area = 0.;          // printed material, density weighted, mm^2
    double I_u = 0., I_v = 0.; // second moments resisting moments about the in-plane axes u, v
    double S_u = 0., S_v = 0.; // section moduli for the same
    double Q = 0.;             // torsional section modulus, mm^3
    double strength = 0.;      // MPa after anisotropy and notch factor
    double kt = 1.;
};

struct Hole
{
    Vec2d  center; // world X, Y
    double radius;
    double kt;
};

int u_of(int axis) { return (axis + 1) % 3; }
int v_of(int axis) { return (axis + 2) % 3; }

// Area integrals in a cut's own plane (u, v): Iuu = integral of v^2, Ivv = integral of u^2.
struct Moments
{
    double A = 0., Su = 0., Sv = 0., Iuu = 0., Ivv = 0.;

    void add(const Moments &o, double w)
    {
        A += w * o.A, Su += w * o.Su, Sv += w * o.Sv, Iuu += w * o.Iuu, Ivv += w * o.Ivv;
    }
    void add_rect(double u0, double u1, double v0, double v1, double w)
    {
        const double du = u1 - u0, dv = v1 - v0;
        A += w * du * dv;
        Su += w * 0.5 * (u1 * u1 - u0 * u0) * dv;
        Sv += w * 0.5 * (v1 * v1 - v0 * v0) * du;
        Iuu += w * du * (v1 * v1 * v1 - v0 * v0 * v0) / 3.;
        Ivv += w * dv * (u1 * u1 * u1 - u0 * u0 * u0) / 3.;
    }
};

// Green's theorem over every loop; holes run clockwise and subtract themselves.
Moments moments_of(const ExPolygons &area)
{
    Moments m;
    auto    add_loop = [&m](const Polygon &poly) {
        for (size_t i = 0; i < poly.points.size(); ++i) {
            const Vec2d  a = unscale(poly.points[i]), b = unscale(poly.points[(i + 1) % poly.points.size()]);
            const double cr = a.x() * b.y() - b.x() * a.y();
            m.A += 0.5 * cr;
            m.Su += (a.x() + b.x()) * cr / 6.;
            m.Sv += (a.y() + b.y()) * cr / 6.;
            m.Iuu += (a.y() * a.y() + a.y() * b.y() + b.y() * b.y()) * cr / 12.;
            m.Ivv += (a.x() * a.x() + a.x() * b.x() + b.x() * b.x()) * cr / 12.;
        }
    };
    for (const ExPolygon &ex : area) {
        add_loop(ex.contour);
        for (const Polygon &h : ex.holes)
            add_loop(h);
    }
    return m;
}

double triangle_area(const indexed_triangle_set &its, const Vec3i32 &f)
{
    const Vec3d a = its.vertices[f[0]].cast<double>(), b = its.vertices[f[1]].cast<double>(), c = its.vertices[f[2]].cast<double>();
    return 0.5 * (b - a).cross(c - a).norm();
}

// Extent of the facets only: a facet set may share the vertex list of the whole mesh.
BoundingBoxf3 facets_bounding_box(const indexed_triangle_set &its)
{
    BoundingBoxf3 bbox;
    for (const Vec3i32 &f : its.indices)
        for (int v : f)
            bbox.merge(its.vertices[v].cast<double>());
    return bbox;
}

// Is a cut normal to `axis` at `pos` between the fixture and the load? If the
// fixture straddles the cut, the load reaches ground without crossing it.
bool separates(const BoundingBoxf3 &fixture, int axis, double pos, const Vec3d &load_point)
{
    if (fixture.min[axis] > pos)
        return load_point[axis] < pos;
    if (fixture.max[axis] < pos)
        return load_point[axis] > pos;
    return false;
}

double distance_to_polygon(const Vec2d &p, const Polygon &poly)
{
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poly.points.size(); ++i) {
        const Vec2d  a = unscale(poly.points[i]), b = unscale(poly.points[(i + 1) % poly.points.size()]);
        const Vec2d  ab = b - a;
        const double t  = std::clamp(ab.squaredNorm() > 0. ? (p - a).dot(ab) / ab.squaredNorm() : 0., 0., 1.);
        best            = std::min(best, (a + t * ab - p).norm());
    }
    return best;
}

// Holes in a layer, with the net-section stress concentration each causes.
std::vector<Hole> holes_of(const ExPolygons &outline)
{
    std::vector<Hole> out;
    for (const ExPolygon &ex : outline)
        for (const Polygon &hole : ex.holes) {
            const double r = std::sqrt(std::abs(unscaled<double>(unscaled<double>(hole.area()))) / PI);
            if (r < MIN_HOLE_RADIUS)
                continue;
            const Vec2d  c        = unscale(hole.centroid());
            const double ligament = distance_to_polygon(c, ex.contour) - r;
            if (ligament > 0.)
                out.push_back({c, r, kt_hole(2. * r, 2. * r + 2. * ligament)});
        }
    return out;
}

// Vertical holes, for the vertical cuts that pass through them.
std::vector<Hole> probe_holes(const PrintedLayers &layers)
{
    std::vector<Hole> out;
    for (int k = 0; k < HOLE_PROBE_LAYERS; ++k)
        for (const Hole &h : holes_of(layers[(2 * k + 1) * layers.size() / (2 * HOLE_PROBE_LAYERS)].outline))
            if (std::none_of(out.begin(), out.end(), [&h](const Hole &o) { return (o.center - h.center).norm() < 1.; }))
                out.push_back(h);
    return out;
}

// Accumulated cut through printed material, before it becomes a Section.
struct CutData
{
    Moments      printed;        // density weighted
    double       gross     = 0.; // everything inside the outline
    double       solid     = 0.; // walls and solid infill
    double       perimeter = 0.;
    BoundingBoxf extent;         // in (u, v)
};

std::optional<Section> make_section(int axis, double pos, const CutData &cut, double kt, const Material &mat)
{
    const Moments &m = cut.printed;
    if (m.A <= 1e-6 || !cut.extent.defined)
        return std::nullopt;
    Section s;
    s.axis = axis;
    s.pos  = pos;
    s.area = m.A;
    const Vec2d c(m.Su / m.A, m.Sv / m.A);
    s.centroid[axis]       = pos;
    s.centroid[u_of(axis)] = c.x();
    s.centroid[v_of(axis)] = c.y();
    s.I_u                  = std::max(0., m.Iuu - m.A * c.y() * c.y());
    s.I_v                  = std::max(0., m.Ivv - m.A * c.x() * c.x());
    const double c_u       = std::max(cut.extent.max.x() - c.x(), c.x() - cut.extent.min.x());
    const double c_v       = std::max(cut.extent.max.y() - c.y(), c.y() - cut.extent.min.y());
    s.S_u                  = c_v > 0. ? s.I_u / c_v : 0.;
    s.S_v                  = c_u > 0. ? s.I_v / c_u : 0.;
    if (m.A >= SOLID_THRESHOLD * cut.gross)
        s.Q = 0.2 * std::pow(m.A, 1.5); // compact solid, Saint-Venant
    else if (cut.perimeter > 0.)
        s.Q = 2. * cut.gross * cut.solid / cut.perimeter; // closed shell of walls and skins, Bredt
    // Across layers only adhesion holds; along a vertical cut the beads carry most of it.
    const double strength = mat.uts * (axis == 2 ? mat.z_factor : std::min(1., mat.z_factor + 0.15));
    s.kt                  = kt;
    s.strength            = strength / (1. + notch_sensitivity(mat.elongation) * (kt - 1.));
    return s;
}

// Cuts between layers, each through the layer above the interface.
std::vector<Section> cut_between_layers(const PrintedLayers &layers, const BoundingBoxf3 &fixture, const Vec3d &load_point,
                                        const Material &mat)
{
    std::vector<Section> out;
    for (size_t i = 1; i < layers.size(); ++i) {
        const PrintedLayer &layer = layers[i];
        if (!separates(fixture, 2, layer.z_bottom, load_point))
            continue;
        CutData cut;
        for (const LayerFill &fill : layer.fills) {
            const Moments m = moments_of(fill.area);
            cut.printed.add(m, fill.density);
            if (fill.density >= SOLID_DENSITY)
                cut.solid += m.A;
        }
        cut.gross = moments_of(layer.outline).A;
        for (const ExPolygon &ex : layer.outline)
            for (size_t k = 0; k < ex.contour.points.size(); ++k) {
                const Vec2d a = unscale(ex.contour.points[k]);
                cut.perimeter += (unscale(ex.contour.points[(k + 1) % ex.contour.points.size()]) - a).norm();
                cut.extent.merge(a);
            }
        double kt = 1.;
        for (const Hole &h : holes_of(layer.outline))
            kt = std::max(kt, h.kt);
        if (std::optional<Section> s = make_section(2, layer.z_bottom, cut, kt, mat))
            out.push_back(*s);
    }
    return out;
}

// Vertical cuts normal to X or Y on a regular grid. Each layer adds rectangles: the
// spans where the cut line crosses each fill, times the layer height. Edges are
// bucketed by the grid lines they cross, so the cost follows the edge count rather
// than edges times cuts.
std::vector<Section> cut_across_layers(const PrintedLayers &layers, int axis, const BoundingBoxf3 &bbox, const BoundingBoxf3 &fixture,
                                       const Vec3d &load_point, const std::vector<Hole> &holes, const Material &mat)
{
    const double span = bbox.size()[axis] * (1. - 2. * TRIM);
    if (span <= 1e-6)
        return {};
    const double lo   = bbox.min[axis] + bbox.size()[axis] * TRIM;
    const int    n    = std::clamp(int(span / MIN_CUT_STEP) + 1, 4, MAX_CUTS_PER_AXIS);
    const double step = span / (n - 1);
    const int    h    = 1 - axis; // the horizontal axis lying in the cut

    std::vector<char> active(n);
    for (int k = 0; k < n; ++k)
        active[k] = separates(fixture, axis, lo + k * step, load_point);
    if (std::none_of(active.begin(), active.end(), [](char a) { return a; }))
        return {};

    std::vector<CutData>             cuts(n);
    std::vector<std::vector<double>> crossings(n);
    std::vector<int>                 touched;
    for (const PrintedLayer &layer : layers) {
        const double z0 = layer.z_bottom, z1 = layer.z_top;
        for (const LayerFill &fill : layer.fills) {
            auto add_loop = [&](const Polygon &poly) {
                for (size_t i = 0; i < poly.points.size(); ++i) {
                    const Vec2d p  = unscale(poly.points[i]), q = unscale(poly.points[(i + 1) % poly.points.size()]);
                    const int   k0 = std::max(0, int(std::ceil((std::min(p[axis], q[axis]) - lo) / step)));
                    const int   k1 = std::min(n - 1, int(std::floor((std::max(p[axis], q[axis]) - lo) / step)));
                    for (int k = k0; k <= k1; ++k) {
                        const double c = lo + k * step;
                        if (!active[k] || (p[axis] > c) == (q[axis] > c))
                            continue;
                        if (crossings[k].empty())
                            touched.push_back(k);
                        crossings[k].push_back(p[h] + (c - p[axis]) / (q[axis] - p[axis]) * (q[h] - p[h]));
                    }
                }
            };
            for (const ExPolygon &ex : fill.area) {
                add_loop(ex.contour);
                for (const Polygon &hole : ex.holes)
                    add_loop(hole);
            }
            for (int k : touched) {
                std::vector<double> &x = crossings[k];
                std::sort(x.begin(), x.end());
                CutData &cut = cuts[k];
                for (size_t i = 0; i + 1 < x.size(); i += 2) {
                    // (u, v) is (h, z) for a cut normal to X and (z, h) for one normal to Y.
                    const double u0 = axis == 0 ? x[i] : z0, u1 = axis == 0 ? x[i + 1] : z1;
                    const double v0 = axis == 0 ? z0 : x[i], v1 = axis == 0 ? z1 : x[i + 1];
                    cut.printed.add_rect(u0, u1, v0, v1, fill.density);
                    const double a = (x[i + 1] - x[i]) * (z1 - z0);
                    cut.gross += a;
                    if (fill.density >= SOLID_DENSITY)
                        cut.solid += a;
                    cut.extent.merge(Vec2d(u0, v0));
                    cut.extent.merge(Vec2d(u1, v1));
                }
                x.clear();
            }
            touched.clear();
        }
    }

    std::vector<Section> out;
    for (int k = 0; k < n; ++k) {
        if (!active[k])
            continue;
        const double pos = lo + k * step;
        CutData     &cut = cuts[k];
        cut.perimeter    = cut.extent.defined ? 2. * (cut.extent.size().x() + cut.extent.size().y()) : 0.;
        double kt        = 1.;
        for (const Hole &hole : holes)
            if (std::abs(hole.center[axis] - pos) <= hole.radius)
                kt = std::max(kt, hole.kt);
        if (std::optional<Section> s = make_section(axis, pos, cut, kt, mat))
            out.push_back(*s);
    }
    return out;
}

// Curvature of the beam axis at a cut, as a rotation per unit length.
Vec3d curvature(const Section &s, const Vec3d &load_point, const Vec3d &force, double modulus)
{
    const Vec3d M = (load_point - s.centroid).cross(force);
    Vec3d       k = Vec3d::Zero();
    if (s.I_u > 0.)
        k[u_of(s.axis)] = M[u_of(s.axis)] / (modulus * s.I_u);
    if (s.I_v > 0.)
        k[v_of(s.axis)] = M[v_of(s.axis)] / (modulus * s.I_v);
    return k;
}

// Deflected shape along the axis on which the load sits furthest from the fixture,
// integrating curvature from the fixture outwards. Torsional twist is not included.
void deflected_shape(Result &res, const std::vector<Section> &sections, const Vec3d &fixture_center, const BoundingBoxf3 &fixture,
                     const Vec3d &force, double modulus)
{
    int dom = 0;
    for (int a = 1; a < 3; ++a)
        if (std::abs(res.load_point[a] - fixture_center[a]) > std::abs(res.load_point[dom] - fixture_center[dom]))
            dom = a;
    std::vector<const Section *> run;
    for (const Section &s : sections)
        if (s.axis == dom)
            run.push_back(&s);
    if (run.size() < 2 || modulus <= 0.)
        return;
    std::sort(run.begin(), run.end(), [](const Section *a, const Section *b) { return a->pos < b->pos; });
    if (fixture.min[dom] > run.front()->pos)
        std::reverse(run.begin(), run.end());

    res.run_axis = dom;
    Vec3d theta = Vec3d::Zero(), w = Vec3d::Zero();
    Vec3d kappa = curvature(*run.front(), res.load_point, force, modulus);
    res.run.push_back({run.front()->pos, run.front()->centroid, w, theta});
    for (size_t k = 1; k < run.size(); ++k) {
        const Vec3d next_kappa = curvature(*run[k], res.load_point, force, modulus);
        const Vec3d next_theta = theta + 0.5 * (kappa + next_kappa) * std::abs(run[k]->pos - run[k - 1]->pos);
        w += (0.5 * (theta + next_theta)).cross(run[k]->centroid - run[k - 1]->centroid);
        theta = next_theta;
        kappa = next_kappa;
        res.run.push_back({run[k]->pos, run[k]->centroid, w, theta});
    }
    // Beyond the last cut the part is unloaded and moves rigidly with the end of the run.
    res.deflection = displacement_at(res, res.load_point).dot(force.normalized());
}

} // namespace

std::optional<Vec3d> area_centroid(const indexed_triangle_set &its)
{
    Vec3d  sum   = Vec3d::Zero();
    double total = 0.;
    for (const Vec3i32 &f : its.indices) {
        const double a = triangle_area(its, f);
        sum += a * (its.vertices[f[0]] + its.vertices[f[1]] + its.vertices[f[2]]).cast<double>() / 3.;
        total += a;
    }
    if (total <= 0.)
        return std::nullopt;
    return sum / total;
}

const Material &default_material() { return MATERIALS.front(); }

const Material *find_material(const std::string &filament_type)
{
    std::string key = boost::algorithm::to_upper_copy(boost::algorithm::trim_copy(filament_type));
    std::replace(key.begin(), key.end(), ' ', '-');
    for (const auto &[alias, name] : ALIASES)
        if (key == alias)
            key = name;
    for (const Material &m : MATERIALS)
        if (key == m.name)
            return &m;
    return nullptr;
}

double kt_hole(double d, double w)
{
    if (d <= 0. || w <= d)
        return 1.;
    const double x = std::min(d / w, 0.5);
    return std::max(1., 3. - 3.13 * x + 3.66 * x * x - 1.53 * x * x * x);
}

double notch_sensitivity(double elongation) { return elongation <= 5. ? 1. : std::max(0.4, 1. - (elongation - 5.) / 60.); }

PrintedLayers layers_from_mesh(const indexed_triangle_set &mesh, const PrintSettings &print)
{
    const BoundingBoxf3 bbox = facets_bounding_box(mesh);
    if (!bbox.defined || bbox.size().z() <= 0.)
        return {};
    const int          n = std::max(1, int(std::round(bbox.size().z() / std::max(print.layer_height, 0.01))));
    const double       h = bbox.size().z() / n;
    std::vector<float> zs(n);
    for (int i = 0; i < n; ++i)
        zs[i] = float(bbox.min.z() + (i + 0.5) * h);
    const std::vector<ExPolygons> outlines = slice_mesh_ex(mesh, zs);

    PrintedLayers layers(n);
    tbb::parallel_for(0, n, [&](int i) {
        PrintedLayer &layer = layers[i];
        layer.z_bottom      = bbox.min.z() + i * h;
        layer.z_top         = layer.z_bottom + h;
        layer.outline       = outlines[i];
        // Sparse infill only where the layers below and above cover it, as Orca does for skins.
        ExPolygons sparse = offset_ex(outlines[i], -float(scaled(std::max(0, print.wall_loops) * print.wall_width)));
        for (int k = 1; k <= print.bottom_layers && !sparse.empty(); ++k)
            sparse = i - k >= 0 ? intersection_ex(sparse, outlines[i - k]) : ExPolygons();
        for (int k = 1; k <= print.top_layers && !sparse.empty(); ++k)
            sparse = i + k < n ? intersection_ex(sparse, outlines[i + k]) : ExPolygons();
        layer.fills = {{diff_ex(outlines[i], sparse), 1.}, {std::move(sparse), std::clamp(print.infill, 0., 1.)}};
    });
    return layers;
}

PrintedLayers layers_from_print_object(const PrintObject &object, const PrintInstance &instance, double z_min)
{
    auto placed = [&instance](ExPolygons area) {
        for (ExPolygon &ex : area)
            ex.translate(instance.shift);
        return area;
    };
    PrintedLayers out;
    if (object.layers().empty())
        return out;
    // Layers start above any raft; the part itself sits at z_min.
    const double z0 = object.layers().front()->print_z - object.layers().front()->height;
    for (const Layer *layer : object.layers()) {
        PrintedLayer &pl = out.emplace_back();
        pl.z_bottom      = layer->print_z - layer->height - z0 + z_min;
        pl.z_top         = layer->print_z - z0 + z_min;
        pl.outline       = placed(layer->lslices);
        for (const LayerRegion *region : layer->regions()) {
            const double density = region->region().config().sparse_infill_density.value / 100.;
            // Walls fill whatever the infill does not.
            pl.fills.push_back({placed(diff_ex(to_expolygons(region->slices.surfaces), region->fill_expolygons)), 1.});
            for (const Surface &s : region->fill_surfaces.surfaces)
                pl.fills.push_back({placed({s.expolygon}), s.surface_type == stInternal ? density : s.surface_type == stInternalVoid ? 0. : 1.});
        }
    }
    return out;
}

Result analyse(const PrintedLayers &layers, const LoadCase &lc, const Material &mat)
{
    Result                     res;
    const std::optional<Vec3d> load_point     = area_centroid(lc.load);
    const std::optional<Vec3d> fixture_center = area_centroid(lc.fixture);
    if (!fixture_center) {
        res.error = _u8L("Paint where the part is held.");
        return res;
    }
    if (!load_point) {
        res.error = _u8L("Paint where the load is applied.");
        return res;
    }
    if (lc.force.norm() <= 0.) {
        res.error = _u8L("Enter a load greater than zero.");
        return res;
    }
    if (layers.empty()) {
        res.error = _u8L("The part has no printed layers.");
        return res;
    }
    res.load_point = *load_point;

    BoundingBoxf3 bbox;
    for (const PrintedLayer &layer : layers)
        for (const ExPolygon &ex : layer.outline)
            for (const Point &p : ex.contour.points) {
                const Vec2d q = unscale(p);
                bbox.merge(Vec3d(q.x(), q.y(), layer.z_bottom));
                bbox.merge(Vec3d(q.x(), q.y(), layer.z_top));
            }
    const BoundingBoxf3     fixture = facets_bounding_box(lc.fixture);
    const std::vector<Hole> holes   = probe_holes(layers);

    std::vector<Section> sections = cut_between_layers(layers, fixture, *load_point, mat);
    for (int axis : {0, 1}) {
        std::vector<Section> cut = cut_across_layers(layers, axis, bbox, fixture, *load_point, holes, mat);
        sections.insert(sections.end(), cut.begin(), cut.end());
    }

    const Vec3d &F        = lc.force;
    double       worst_sf = std::numeric_limits<double>::max();
    for (const Section &s : sections) {
        const int    a = s.axis, u = u_of(a), v = v_of(a);
        const Vec3d  M       = (*load_point - s.centroid).cross(F); // N.mm
        const double bending = (s.S_u > 0. ? std::abs(M[u]) / s.S_u : 0.) + (s.S_v > 0. ? std::abs(M[v]) / s.S_v : 0.);
        const double axial   = std::abs(F[a]) / s.area;
        const double tau_v   = SHEAR_PEAK * std::hypot(F[u], F[v]) / s.area;
        // A section with no closed loop of wall has next to no torsional strength.
        const double tau_t = s.Q > 0. ? std::abs(M[a]) / s.Q : (std::abs(M[a]) > 1e-9 ? std::numeric_limits<double>::infinity() : 0.);
        const double sigma = bending + axial;
        // Shear peaks of the two kinds sit at different points, so adding them is conservative.
        const double tau = tau_v + tau_t;
        const double vm  = std::sqrt(sigma * sigma + 3. * tau * tau);
        if (vm <= 1e-12)
            continue;
        res.cuts.push_back({a, s.pos, vm / s.strength});
        const double sf = s.strength / vm;
        if (sf < worst_sf) {
            worst_sf           = sf;
            res.safety_factor  = sf;
            res.stress         = vm;
            res.allowable      = s.strength;
            res.kt             = s.kt;
            res.axis           = a;
            res.critical_point = s.centroid;
            res.mode           = 3. * tau * tau > sigma * sigma ? (tau_t > tau_v ? Mode::Torsion : Mode::Shear) :
                                                                  (axial > bending ? Mode::Tension : Mode::Bending);
        }
    }
    if (worst_sf == std::numeric_limits<double>::max()) {
        res.error = _u8L("No cut through the part separates the fixture from the load, so the load reaches the fixture "
                         "without stressing the part. Paint a smaller fixture or load another area.");
        return res;
    }
    std::sort(res.cuts.begin(), res.cuts.end(), [](const Cut &a, const Cut &b) { return std::tie(a.axis, a.pos) < std::tie(b.axis, b.pos); });
    deflected_shape(res, sections, *fixture_center, fixture, F, mat.modulus);
    return res;
}

void scale_load(Result &res, double factor)
{
    res.stress *= factor;
    res.safety_factor /= factor;
    res.deflection *= factor;
    for (Cut &cut : res.cuts)
        cut.utilization *= factor;
    for (BeamStation &st : res.run) {
        st.displacement *= factor;
        st.rotation *= factor;
    }
}

Vec3d displacement_at(const Result &res, const Vec3d &p)
{
    if (res.run.empty())
        return Vec3d::Zero();
    const std::vector<BeamStation> &run = res.run;
    const double                    sgn = run.size() > 1 && run[1].pos < run[0].pos ? -1. : 1.;
    auto along = [&](double pos) { return (pos - run.front().pos) * sgn; };
    const double s = along(p[res.run_axis]);
    if (s <= 0.)
        return Vec3d::Zero(); // the held side of the fixture edge
    // First station beyond p along the run.
    const auto it = std::upper_bound(run.begin() + 1, run.end(), s, [&](double s, const BeamStation &st) { return s < along(st.pos); });
    const BeamStation &a = *(it - 1);
    if (it == run.end())
        return a.displacement + a.rotation.cross(p - a.centroid);
    const BeamStation &b = *it;
    const double       t = std::clamp((s - along(a.pos)) / (along(b.pos) - along(a.pos)), 0., 1.);
    const Vec3d        w = (1. - t) * a.displacement + t * b.displacement;
    const Vec3d        r = (1. - t) * a.rotation + t * b.rotation;
    const Vec3d        c = (1. - t) * a.centroid + t * b.centroid;
    return w + r.cross(p - c);
}

double utilization_at(const Result &res, const Vec3d &p)
{
    double worst = 0.;
    for (int axis = 0; axis < 3; ++axis) {
        const auto first = std::lower_bound(res.cuts.begin(), res.cuts.end(), axis, [](const Cut &c, int a) { return c.axis < a; });
        const auto last  = std::upper_bound(first, res.cuts.end(), axis, [](int a, const Cut &c) { return a < c.axis; });
        if (last - first < 2)
            continue;
        // Cuts are evenly spaced or one per layer: a point belongs to the nearest one
        // within a cut spacing, else it lies outside the stressed part.
        const auto   it      = std::lower_bound(first, last, p[axis], [](const Cut &c, double v) { return c.pos < v; });
        const double spacing = ((last - 1)->pos - first->pos) / double(last - first - 1);
        if (it != last && std::abs(it->pos - p[axis]) <= spacing)
            worst = std::max(worst, it->utilization);
        if (it != first && std::abs((it - 1)->pos - p[axis]) <= spacing)
            worst = std::max(worst, (it - 1)->utilization);
    }
    return worst;
}

} // namespace Slic3r::Feature::StrengthAnalysis
