#include "StrengthAnalysis.hpp"

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

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

constexpr double KT_CAP            = 6.;
constexpr double SHEAR_PEAK        = 1.5;  // peak / mean transverse shear, rectangular section
constexpr double SOLID_THRESHOLD   = 0.85; // printed / enclosed area above which torsion sees a solid
constexpr double TRIM              = 0.01; // ends of the part left uncut, as a fraction of its span
constexpr int    MAX_CUTS_PER_AXIS = 600;
constexpr int    HOLE_PROBE_CUTS   = 12;
constexpr double MIN_HOLE_RADIUS   = 0.3;

struct Section
{
    int    axis = 2;
    double pos  = 0.;
    Vec3d  centroid;
    double area = 0.;      // printed material, mm^2
    double S_u = 0., S_v = 0.; // section moduli resisting moments about the in-plane axes u, v
    double I_u = 0., I_v = 0.;
    double Q = 0.;         // torsional section modulus, mm^3
    double strength = 0.;  // MPa after anisotropy and notch factor
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

// Cyclic permutation so that `axis` becomes Z. Being cyclic it keeps handedness,
// hence contour orientation.
indexed_triangle_set axis_to_z(const indexed_triangle_set &its, int axis)
{
    indexed_triangle_set out = its;
    if (axis != 2)
        for (stl_vertex &p : out.vertices)
            p = stl_vertex(p[u_of(axis)], p[v_of(axis)], p[axis]);
    return out;
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
        const Vec2d a = unscale(poly.points[i]), b = unscale(poly.points[(i + 1) % poly.points.size()]);
        const Vec2d ab = b - a;
        const double t = std::clamp(ab.squaredNorm() > 0. ? (p - a).dot(ab) / ab.squaredNorm() : 0., 0., 1.);
        best = std::min(best, (a + t * ab - p).norm());
    }
    return best;
}

// Holes in one cut, with the net-section stress concentration each causes.
std::vector<Hole> holes_of(const ExPolygons &cut)
{
    std::vector<Hole> out;
    for (const ExPolygon &ex : cut)
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

// Gross geometry of a cut in its own plane (u, v), by Green's theorem over every loop.
struct CutGeometry
{
    double area = 0., enclosed = 0., perimeter = 0., outer_perimeter = 0.;
    Vec2d  centroid = Vec2d::Zero();
    double I_u = 0., I_v = 0., c_u = 0., c_v = 0.;
};

std::optional<CutGeometry> geometry_of(const ExPolygons &cut)
{
    CutGeometry g;
    double   su = 0., sv = 0., iu = 0., iv = 0.;
    auto     add_loop = [&](const Polygon &poly, bool outer) {
        for (size_t i = 0; i < poly.points.size(); ++i) {
            const Vec2d  a = unscale(poly.points[i]), b = unscale(poly.points[(i + 1) % poly.points.size()]);
            const double cr = a.x() * b.y() - b.x() * a.y();
            g.area += 0.5 * cr;
            su += (a.x() + b.x()) * cr;
            sv += (a.y() + b.y()) * cr;
            iu += (a.y() * a.y() + a.y() * b.y() + b.y() * b.y()) * cr;
            iv += (a.x() * a.x() + a.x() * b.x() + b.x() * b.x()) * cr;
            const double len = (b - a).norm();
            g.perimeter += len;
            if (outer)
                g.outer_perimeter += len;
        }
    };
    for (const ExPolygon &ex : cut) {
        add_loop(ex.contour, true);
        g.enclosed += unscaled<double>(unscaled<double>(ex.contour.area()));
        for (const Polygon &h : ex.holes)
            add_loop(h, false);
    }
    if (g.area <= 1e-6)
        return std::nullopt;
    g.centroid = Vec2d(su, sv) / (6. * g.area);
    g.I_u      = std::max(0., iu / 12. - g.area * sqr(g.centroid.y()));
    g.I_v      = std::max(0., iv / 12. - g.area * sqr(g.centroid.x()));
    for (const ExPolygon &ex : cut)
        for (const Point &p : ex.contour.points) {
            const Vec2d d = unscale(p) - g.centroid;
            g.c_u         = std::max(g.c_u, std::abs(d.x()));
            g.c_v         = std::max(g.c_v, std::abs(d.y()));
        }
    return g;
}

std::vector<Section> cut_axis(const indexed_triangle_set &mesh, int axis, const BoundingBoxf3 &bbox, const BoundingBoxf3 &fixture,
                              const Vec3d &load_point, const std::vector<Hole> &holes, const Material &mat, const PrintSettings &print)
{
    const double span = bbox.size()[axis] * (1. - 2. * TRIM);
    if (span <= 1e-6)
        return {};
    const double lo = bbox.min[axis] + bbox.size()[axis] * TRIM;
    const int    n  = std::clamp(int(std::round(span / std::max(print.layer_height, 0.01))), 4, MAX_CUTS_PER_AXIS);

    std::vector<float> zs;
    for (int i = 0; i < n; ++i)
        if (const double pos = lo + span * i / (n - 1); separates(fixture, axis, pos, load_point))
            zs.push_back(float(pos));
    if (zs.empty())
        return {};

    const std::vector<ExPolygons> cuts     = slice_mesh_ex(axis_to_z(mesh, axis), zs);
    const double                  q        = notch_sensitivity(mat.elongation);
    // Across layers only adhesion holds; along a vertical cut the beads carry most of it.
    const double                  strength = mat.uts * (axis == 2 ? mat.z_factor : std::min(1., mat.z_factor + 0.15));
    const double                  shell    = std::max(0, print.wall_loops) * print.wall_width;

    std::vector<Section> out;
    for (size_t i = 0; i < cuts.size(); ++i) {
        const std::optional<CutGeometry> g = geometry_of(cuts[i]);
        if (!g)
            continue;
        // A mesh is solid; the print is walls around the boundary plus sparse infill inside.
        const double wall_area = std::min(g->area, g->perimeter * shell);
        const double printed   = wall_area + std::clamp(print.infill, 0., 1.) * (g->area - wall_area);
        const double frac      = printed / g->area;

        Section s;
        s.axis              = axis;
        s.pos               = zs[i];
        s.centroid[axis]    = zs[i];
        s.centroid[u_of(axis)] = g->centroid.x();
        s.centroid[v_of(axis)] = g->centroid.y();
        s.area              = printed;
        s.I_u               = g->I_u * frac;
        s.I_v               = g->I_v * frac;
        s.S_u               = g->c_v > 0. ? s.I_u / g->c_v : 0.;
        s.S_v               = g->c_u > 0. ? s.I_v / g->c_u : 0.;
        if (frac >= SOLID_THRESHOLD)
            s.Q = 0.2 * std::pow(printed, 1.5);              // compact solid, Saint-Venant
        else if (g->outer_perimeter > 0.)
            s.Q = 2. * g->enclosed * std::min(shell, g->area / g->outer_perimeter); // closed thin-walled shell, Bredt

        if (axis == 2) {
            for (const Hole &h : holes_of(cuts[i]))
                s.kt = std::max(s.kt, h.kt);
        } else {
            for (const Hole &h : holes)
                if (std::abs(h.center[axis] - s.pos) <= h.radius)
                    s.kt = std::max(s.kt, h.kt);
        }
        s.strength = strength / (1. + q * (s.kt - 1.));
        out.push_back(s);
    }
    return out;
}

// Holes through the part seen from above, for the cuts that pass through them sideways.
std::vector<Hole> probe_holes(const indexed_triangle_set &mesh, const BoundingBoxf3 &bbox)
{
    std::vector<float> zs;
    for (int k = 0; k < HOLE_PROBE_CUTS; ++k)
        zs.push_back(float(bbox.min.z() + bbox.size().z() * (k + 0.5) / HOLE_PROBE_CUTS));
    std::vector<Hole> out;
    for (const ExPolygons &cut : slice_mesh_ex(mesh, zs))
        for (const Hole &h : holes_of(cut))
            if (std::none_of(out.begin(), out.end(), [&h](const Hole &o) { return (o.center - h.center).norm() < 1.; }))
                out.push_back(h);
    return out;
}

// Deflection at the load point by the unit-load method, integrated along the axis
// on which the load sits furthest from the fixture.
double deflection(const std::vector<Section> &sections, const Vec3d &load_point, const Vec3d &fixture_center, const Vec3d &force, double modulus)
{
    int dom = 0;
    for (int a = 1; a < 3; ++a)
        if (std::abs(load_point[a] - fixture_center[a]) > std::abs(load_point[dom] - fixture_center[dom]))
            dom = a;
    std::vector<const Section *> run;
    for (const Section &s : sections)
        if (s.axis == dom)
            run.push_back(&s);
    if (run.size() < 2 || modulus <= 0.)
        return 0.;
    const Vec3d dir   = force.normalized();
    double      total = 0.;
    for (size_t k = 0; k < run.size(); ++k) {
        const double ds = std::abs(run[std::min(k + 1, run.size() - 1)]->pos - run[k > 0 ? k - 1 : 0]->pos) /
                          (k == 0 || k + 1 == run.size() ? 1. : 2.);
        const Vec3d m = (load_point - run[k]->centroid).cross(dir);
        if (run[k]->I_u > 0.)
            total += sqr(m[u_of(dom)]) / run[k]->I_u * ds;
        if (run[k]->I_v > 0.)
            total += sqr(m[v_of(dom)]) / run[k]->I_v * ds;
    }
    return force.norm() * total / modulus;
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

double notch_sensitivity(double elongation)
{
    return elongation <= 5. ? 1. : std::max(0.4, 1. - (elongation - 5.) / 60.);
}

Result analyse(const indexed_triangle_set &mesh, const LoadCase &lc, const Material &mat, const PrintSettings &print)
{
    Result res;
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
    res.load_point = *load_point;

    const BoundingBoxf3 bbox    = bounding_box(mesh);
    const BoundingBoxf3 fixture = facets_bounding_box(lc.fixture);
    const std::vector<Hole> holes = probe_holes(mesh, bbox);

    std::vector<Section> sections;
    for (int axis : {2, 0, 1}) {
        std::vector<Section> cut = cut_axis(mesh, axis, bbox, fixture, *load_point, holes, mat, print);
        sections.insert(sections.end(), cut.begin(), cut.end());
    }

    const Vec3d &F = lc.force;
    double       worst_sf = std::numeric_limits<double>::max();
    for (const Section &s : sections) {
        const int    a = s.axis, u = u_of(a), v = v_of(a);
        const Vec3d  M = (*load_point - s.centroid).cross(F); // N.mm
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
    res.deflection = deflection(sections, *load_point, *fixture_center, F, mat.modulus);
    return res;
}

} // namespace Slic3r::Feature::StrengthAnalysis
