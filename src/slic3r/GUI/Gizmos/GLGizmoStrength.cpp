#include "GLGizmoStrength.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/CameraUtils.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/Utils/UndoRedo.hpp"
#include "GLGizmoUtils.hpp"

#include <boost/format.hpp>
#include <glad/gl.h>

namespace Slic3r::GUI {

namespace SA = Feature::StrengthAnalysis;

const ColorRGBA GLGizmoStrength::FixtureColor = {0.13f, 0.69f, 0.33f, 1.f};
const ColorRGBA GLGizmoStrength::LoadColor    = {1.f, 0.45f, 0.1f, 1.f};

namespace {

constexpr double KG_TO_N = 9.80665;
constexpr double LB_TO_N = 4.4482216152605;
// Snap the aimed arrow onto an axis when it comes this close to one.
constexpr double AXIS_SNAP_DEG = 4.;
constexpr double ARROW_GRAB_PX = 10.;
// An exaggerated deflection is drawn this large relative to the part.
constexpr double EXAGGERATED_DEFLECTION = 0.1;
constexpr double ANIMATION_PERIOD_S     = 2.;
constexpr double FLASH_HZ               = 1.5;

const ColorRGBA AxisColors[3] = {{0.9f, 0.2f, 0.2f, 0.6f}, {0.2f, 0.75f, 0.2f, 0.6f}, {0.25f, 0.45f, 1.f, 0.6f}};
const ColorRGBA CriticalColor = {0.9f, 0.1f, 0.1f, 1.f};
// Utilization from 0 (blue) to the breaking point and beyond (red).
const std::array<ColorRGBA, 8> StressColors{{{0.15f, 0.3f, 0.9f, 1.f},
                                             {0.1f, 0.55f, 0.95f, 1.f},
                                             {0.1f, 0.75f, 0.8f, 1.f},
                                             {0.2f, 0.75f, 0.35f, 1.f},
                                             {0.65f, 0.8f, 0.2f, 1.f},
                                             {0.95f, 0.8f, 0.15f, 1.f},
                                             {0.95f, 0.5f, 0.1f, 1.f},
                                             {0.9f, 0.12f, 0.1f, 1.f}}};

double distance_to_segment(const Vec2d &p, const Vec2d &a, const Vec2d &b)
{
    const Vec2d  ab = b - a;
    const double t  = ab.squaredNorm() > 0. ? std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0., 1.) : 0.;
    return (a + t * ab - p).norm();
}

} // namespace

GLGizmoStrength::GLGizmoStrength(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id)
    : GLGizmoPainterBase(parent, icon_filename, sprite_id)
{
    m_cursor_type = TriangleSelector::CursorType::CIRCLE;
}

bool GLGizmoStrength::on_init()
{
    const wxString ctrl  = GUI::shortkey_ctrl_prefix();
    const wxString shift = GUI::shortkey_shift_prefix();

    m_desc["fixture"]     = _L("Paint fixture");
    m_desc["load"]        = _L("Paint load");
    m_desc["remove"]      = _L("Erase");
    m_desc["remove_all"]  = _L("Erase all");
    m_desc["cursor_size"] = _L("Brush size");
    m_desc["aim"]         = _L("Aim load");

    m_shortcuts = {{_L("Left mouse button"), m_desc["fixture"]},
                   {_L("Right mouse button"), m_desc["load"]},
                   {shift + _L("Left mouse button"), m_desc["remove"]},
                   {_L("Drag the arrow"), m_desc["aim"]},
                   {ctrl + _L("Mouse wheel"), m_desc["cursor_size"]}};

    // Unit length, scaled to the part when drawn.
    m_arrow.init_from(stilized_arrow(24, 0.09f, 0.3f, 0.035f, 0.7f));
    m_ring.init_from(smooth_torus(64, 12, 1.f, 0.01f));
    m_marker.init_from(smooth_sphere(24, 1.f));
    return true;
}

std::string GLGizmoStrength::on_get_name() const { return _u8L("Strength check"); }

PainterGizmoType GLGizmoStrength::get_painter_type() const { return PainterGizmoType::STRENGTH; }

void GLGizmoStrength::on_shutdown()
{
    m_aiming = m_arrow_hovered = false;
    m_parent.toggle_model_objects_visibility(true);
}

wxString GLGizmoStrength::handle_snapshot_action_name(bool shift_down, Button button_down) const
{
    if (shift_down)
        return _L("Remove selection");
    return button_down == Button::Left ? m_desc.at("fixture") : m_desc.at("load");
}

void GLGizmoStrength::on_save(cereal::BinaryOutputArchive &ar) const
{
    ar(m_paint, m_direction.x(), m_direction.y(), m_direction.z(), m_magnitude, m_unit, m_load_mode, m_target_fos);
}

void GLGizmoStrength::on_load(cereal::BinaryInputArchive &ar)
{
    ar(m_paint, m_direction.x(), m_direction.y(), m_direction.z(), m_magnitude, m_unit, m_load_mode, m_target_fos);
    invalidate_result();
    update_result();
    GLGizmoPainterBase::on_load(ar);
}

void GLGizmoStrength::update_model_object()
{
    const ModelObject *mo    = m_c->selection_info()->model_object();
    Paint             &paint = m_paint[mo->id().id];
    paint.clear();
    size_t idx = 0;
    for (const ModelVolume *mv : mo->volumes)
        if (mv->is_model_part())
            paint.push_back({mv->mesh().its.indices.size(), m_triangle_selectors[idx++]->serialize()});
    update_load_point();
    invalidate_result();
    m_parent.set_as_dirty();
}

void GLGizmoStrength::update_from_model_object(bool first_update)
{
    wxBusyCursor wait;

    const ModelObject *mo = m_c->selection_info()->model_object();
    m_triangle_selectors.clear();

    const std::vector<ColorRGBA> ebt_colors{GLVolume::NEUTRAL_COLOR, FixtureColor, LoadColor};
    const auto                   paint = m_paint.find(mo->id().id);
    size_t                       idx   = 0;
    for (const ModelVolume *mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;
        m_triangle_selectors.emplace_back(std::make_unique<TriangleSelectorPatch>(mv->mesh(), ebt_colors));
        if (paint != m_paint.end() && idx < paint->second.size() && paint->second[idx].facets == mv->mesh().its.indices.size())
            m_triangle_selectors.back()->deserialize(paint->second[idx].data, false);
        m_triangle_selectors.back()->request_update_render_data();
        ++idx;
    }

    if (first_update) {
        m_base_result.reset();
        update_result();
    }
    update_load_point();
}

void GLGizmoStrength::update_load_point()
{
    const std::vector<Transform3d> trafos = mesh_trafo_matrices();
    indexed_triangle_set           load;
    BoundingBoxf3                  bbox;
    const ModelObject             *mo  = m_c->selection_info()->model_object();
    size_t                         idx = 0;
    for (const ModelVolume *mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;
        indexed_triangle_set its = m_triangle_selectors[idx]->get_facets(EnforcerBlockerType::BLOCKER);
        its_transform(its, trafos[idx]);
        its_merge(load, its);
        bbox.merge(mv->mesh().transformed_bounding_box(trafos[idx]));
        ++idx;
    }
    m_load_point   = SA::area_centroid(load);
    m_arrow_length = std::clamp(0.35 * bbox.max_size(), 10., 60.);
}

double GLGizmoStrength::unit_to_newton() const { return m_unit == Unit::Kilogram ? KG_TO_N : m_unit == Unit::Pound ? LB_TO_N : 1.; }

std::string GLGizmoStrength::format_load(double newtons) const
{
    const char *units[3] = {"kg", "lb", "N"};
    return (boost::format("%1$.2f %2%") % (newtons / unit_to_newton()) % units[int(m_unit)]).str();
}

void GLGizmoStrength::set_direction(const Vec3d &direction)
{
    if (direction.squaredNorm() < 1e-12)
        return;
    Vec3d d = direction.normalized();
    for (int axis = 0; axis < 3; ++axis)
        if (std::abs(d[axis]) > std::cos(Geometry::deg2rad(AXIS_SNAP_DEG)))
            d = Vec3d::Unit(axis) * (d[axis] > 0. ? 1. : -1.);
    if (d != m_direction) {
        m_direction = d;
        invalidate_result();
    }
}

// The arrow pushes on the load point: its tip sits on the part, its tail out in space.
Transform3d GLGizmoStrength::arrow_matrix() const
{
    return Geometry::translation_transform(*m_load_point - m_direction * m_arrow_length) *
           Transform3d(Eigen::Quaterniond::FromTwoVectors(Vec3d::UnitZ(), m_direction)) * Geometry::scale_transform(m_arrow_length);
}

bool GLGizmoStrength::arrow_hit(const Vec2d &mouse) const
{
    const Camera &camera = wxGetApp().plater()->get_camera();
    const Points  pts    = CameraUtils::project(camera, std::vector<Vec3d>{*m_load_point - m_direction * m_arrow_length, *m_load_point});
    return distance_to_segment(mouse, pts[0].cast<double>(), pts[1].cast<double>()) < ARROW_GRAB_PX * m_parent.get_scale();
}

// Swing the arrow tail over a sphere around the load point, like a trackball.
void GLGizmoStrength::aim_arrow(const Vec2d &mouse)
{
    const Camera &camera = wxGetApp().plater()->get_camera();
    Vec3d         origin, ray;
    CameraUtils::ray_from_screen_pos(camera, mouse, origin, ray);
    ray.normalize();
    const Vec3d  center = *m_load_point;
    const double b      = ray.dot(origin - center);
    const double disc   = b * b - ((origin - center).squaredNorm() - m_arrow_length * m_arrow_length);
    // Nearest hit on the sphere, or the point of the ray closest to it when the ray misses.
    const Vec3d tail = origin + ray * (disc >= 0. ? -b - std::sqrt(disc) : -b);
    set_direction(center - tail);
    m_parent.set_as_dirty();
}

bool GLGizmoStrength::on_mouse(const wxMouseEvent &mouse_event)
{
    const Vec2d mouse(mouse_event.GetX(), mouse_event.GetY());
    if (m_aiming) {
        if (mouse_event.Dragging())
            aim_arrow(mouse);
        else if (mouse_event.LeftUp())
            m_aiming = false;
        return true;
    }
    if (mouse_event.Moving() && m_load_point) {
        const bool hovered = arrow_hit(mouse);
        if (hovered != m_arrow_hovered) {
            m_arrow_hovered = hovered;
            m_parent.set_as_dirty();
        }
    }
    if (mouse_event.LeftDown() && m_arrow_hovered && m_load_point && !mouse_event.ShiftDown()) {
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), m_desc.at("aim").ToUTF8().data(), UndoRedo::SnapshotType::GizmoAction);
        m_aiming = true;
        return true;
    }
    return GLGizmoPainterBase::on_mouse(mouse_event);
}

void GLGizmoStrength::render_model(GLModel &model, const Transform3d &matrix, const ColorRGBA &color) const
{
    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr)
        return;
    const Camera     &camera      = wxGetApp().plater()->get_camera();
    const Transform3d view_matrix = camera.get_view_matrix();
    shader->start_using();
    shader->set_uniform("emission_factor", 0.1f);
    shader->set_uniform("view_model_matrix", view_matrix * matrix);
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3) * matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
    shader->set_uniform("view_normal_matrix", view_normal_matrix);
    model.set_color(color);
    model.render();
    shader->stop_using();
}

bool GLGizmoStrength::failed() const
{
    return m_result && m_result->ok() && !m_result_stale && m_load_mode == LoadMode::CheckLoad && m_result->safety_factor < 1.;
}

bool GLGizmoStrength::show_result_mesh() const
{
    return m_result && m_result->ok() && !m_result_stale && (m_show_stress || m_show_deflection) && !m_result_mesh.indices.empty();
}

// Rebuilds the result view: triangles bucketed into stress bands, displaced by
// `phase` (0..1) of the drawn deflection.
void GLGizmoStrength::update_result_mesh(double phase)
{
    double scale = 0.;
    if (m_show_deflection) {
        double max_d = 0.;
        for (const Vec3d &d : m_vertex_displacement)
            max_d = std::max(max_d, d.norm());
        max_d *= m_result_load;
        const double size = Slic3r::bounding_box(m_result_mesh).max_size();
        scale = phase * m_result_load * (m_exaggerate && max_d > 0. ? std::max(1., EXAGGERATED_DEFLECTION * size / max_d) : 1.);
    }

    std::array<GLModel::Geometry, StressBands> bands;
    for (GLModel::Geometry &g : bands)
        g.format = {GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3N3};
    for (const Vec3i32 &f : m_result_mesh.indices) {
        std::array<Vec3f, 3> p;
        double               util = 0.;
        for (int k = 0; k < 3; ++k) {
            p[k] = m_result_mesh.vertices[f[k]] + (scale * m_vertex_displacement[f[k]]).cast<float>();
            util += m_vertex_utilization[f[k]] * m_result_load / 3.;
        }
        const size_t band = m_show_stress ? size_t(std::clamp(int(util * (StressBands - 1) + 0.5), 0, int(StressBands) - 1)) : 0;
        GLModel::Geometry &g = bands[band];
        const Vec3f        n = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
        const unsigned int i = unsigned(g.vertices_count());
        for (const Vec3f &v : p)
            g.add_vertex(v, n);
        g.add_triangle(i, i + 1, i + 2);
    }
    for (size_t b = 0; b < StressBands; ++b) {
        m_stress_models[b].reset();
        if (!bands[b].is_empty())
            m_stress_models[b].init_from(std::move(bands[b]));
    }
    m_mesh_phase = phase;
}

void GLGizmoStrength::render_painter_gizmo()
{
    const Selection &selection = m_parent.get_selection();
    const double     t = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start_time).count();

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));

    if (show_result_mesh()) {
        const bool   moving = m_show_deflection && m_animate;
        const double phase  = !m_show_deflection ? 0. : moving ? 0.5 - 0.5 * std::cos(2. * PI * t / ANIMATION_PERIOD_S) : 1.;
        if (m_mesh_phase < 0. || std::abs(phase - m_mesh_phase) > 0.004)
            update_result_mesh(phase);
        for (size_t b = 0; b < StressBands; ++b)
            if (m_stress_models[b].is_initialized())
                render_model(m_stress_models[b], Transform3d::Identity(), m_show_stress ? StressColors[b] : GLVolume::NEUTRAL_COLOR);
        if (moving)
            m_parent.request_extra_frame();
    } else {
        render_triangles(selection);
        m_c->object_clipper()->render_cut();
        m_c->instances_hider()->render_cut();
        if (!m_arrow_hovered && !m_aiming)
            render_cursor();
    }

    if (m_load_point) {
        const bool active = m_arrow_hovered || m_aiming;
        render_model(m_arrow, arrow_matrix(), active ? ColorRGBA::YELLOW() : LoadColor);
        if (active) {
            // Rotate-gizmo style rings show the sphere the arrow swings over.
            const Transform3d at = Geometry::translation_transform(*m_load_point) * Geometry::scale_transform(m_arrow_length);
            render_model(m_ring, at, AxisColors[2]);
            render_model(m_ring, at * Geometry::rotation_transform({0., 0.5 * PI, 0.}), AxisColors[0]);
            render_model(m_ring, at * Geometry::rotation_transform({0.5 * PI, 0., 0.}), AxisColors[1]);
        }
    }
    if (m_result && m_result->ok() && !m_result_stale) {
        ColorRGBA color = CriticalColor;
        if (failed()) {
            color.a(float(0.35 + 0.65 * (0.5 + 0.5 * std::sin(2. * PI * FLASH_HZ * t))));
            m_parent.request_extra_frame();
        }
        render_model(m_marker, Geometry::translation_transform(m_result->critical_point) * Geometry::scale_transform(0.04 * m_arrow_length / 0.35),
                     color);
    }

    glsafe(::glDisable(GL_BLEND));
}

// Printed layers of the sliced object, in world coordinates, when slicing is
// complete and current: these carry modifiers, variable layer height and Orca's own
// solid infill decisions. Null when the object has not been sliced since its last change.
std::optional<SA::PrintedLayers> GLGizmoStrength::sliced_layers(double z_min) const
{
    Plater *plater = wxGetApp().plater();
    if (plater->is_background_process_slicing())
        return std::nullopt;
    const ModelObject   *mo = m_c->selection_info()->model_object();
    const ModelInstance *mi = mo->instances[m_parent.get_selection().get_instance_idx()];
    for (const PrintObject *po : plater->fff_print().objects()) {
        if (po->model_object()->id() != mo->id() || !po->is_step_done(posPrepareInfill) || po->layers().empty())
            continue;
        const auto inst = std::find_if(po->instances().begin(), po->instances().end(),
                                       [mi](const PrintInstance &pi) { return pi.model_instance->id() == mi->id(); });
        if (inst == po->instances().end())
            return std::nullopt;
        return SA::layers_from_print_object(*po, *inst, z_min);
    }
    return std::nullopt;
}

void GLGizmoStrength::run_analysis()
{
    wxBusyCursor wait;

    const ModelObject             *mo     = m_c->selection_info()->model_object();
    const std::vector<Transform3d> trafos = mesh_trafo_matrices();
    indexed_triangle_set           mesh;
    SA::LoadCase                   lc;
    size_t                         idx = 0;
    for (const ModelVolume *mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;
        indexed_triangle_set its = mv->mesh().its;
        its_transform(its, trafos[idx], true);
        its_merge(mesh, its);
        for (auto [state, out] : {std::pair{EnforcerBlockerType::ENFORCER, &lc.fixture}, std::pair{EnforcerBlockerType::BLOCKER, &lc.load}}) {
            indexed_triangle_set painted = m_triangle_selectors[idx]->get_facets(state);
            its_transform(painted, trafos[idx]);
            its_merge(*out, painted);
        }
        ++idx;
    }
    // Analysed at 1 N: results scale to any load without running again.
    lc.force = m_direction;

    DynamicPrintConfig cfg = wxGetApp().preset_bundle->full_config();
    cfg.apply(mo->config.get(), true);
    const auto       *types    = cfg.option<ConfigOptionStrings>("filament_type");
    const int         extruder = cfg.has("extruder") ? std::max(1, cfg.opt_int("extruder")) : 1;
    const std::string type     = types && !types->values.empty() ? types->values[std::min<size_t>(extruder - 1, types->values.size() - 1)] : "";
    const SA::Material *material = SA::find_material(type);
    m_material_note = material ? material->name :
                                 (boost::format(_u8L("%1% (unknown filament type %2%)")) % SA::default_material().name % type).str();

    std::optional<SA::PrintedLayers> layers = sliced_layers(Slic3r::bounding_box(mesh).min.z());
    m_from_slices                           = layers.has_value();
    if (m_from_slices) {
        m_print_note = _u8L("From the sliced layers, with modifiers and solid infill");
    } else {
        // Laid out from the object's settings as Orca would print it.
        SA::PrintSettings print;
        const double      nozzle = cfg.opt_float("nozzle_diameter", 0);
        print.layer_height       = cfg.opt_float("layer_height");
        print.wall_loops         = cfg.opt_int("wall_loops");
        print.infill             = cfg.option<ConfigOptionPercent>("sparse_infill_density")->value / 100.;
        print.wall_width         = cfg.get_abs_value("inner_wall_line_width", nozzle);
        if (print.wall_width <= 0.)
            print.wall_width = 1.125 * nozzle;
        auto shell_layers = [&](const char *layers_key, const char *thickness_key) {
            return std::max(cfg.opt_int(layers_key), int(std::ceil(cfg.opt_float(thickness_key) / print.layer_height - 1e-6)));
        };
        print.top_layers    = shell_layers("top_shell_layers", "top_shell_thickness");
        print.bottom_layers = shell_layers("bottom_shell_layers", "bottom_shell_thickness");
        layers              = SA::layers_from_mesh(mesh, print);
        m_print_note = (boost::format(_u8L("%1% walls, %2% top and %3% bottom layers, %4% infill, %5% mm layers")) % print.wall_loops %
                        print.top_layers % print.bottom_layers % (std::to_string(int(std::round(print.infill * 100.))) + "%") %
                        print.layer_height).str();
    }

    m_base_result = SA::analyse(*layers, lc, material ? *material : SA::default_material());
    m_result_mesh = std::move(mesh);
    m_vertex_utilization.clear();
    m_vertex_displacement.clear();
    if (m_base_result->ok())
        for (const stl_vertex &v : m_result_mesh.vertices) {
            m_vertex_utilization.push_back(SA::utilization_at(*m_base_result, v.cast<double>()));
            m_vertex_displacement.push_back(SA::displacement_at(*m_base_result, v.cast<double>()));
        }
    m_result_stale = false;
    update_result();
}

void GLGizmoStrength::update_result()
{
    m_mesh_phase = -1.;
    m_result     = m_base_result;
    if (!m_result || !m_result->ok())
        return;
    if (m_load_mode == LoadMode::MaxLoad) {
        if (m_result->safety_factor <= 0.) {
            m_result->error = _u8L("This part has no closed wall around the cut to resist the twist, so it takes no load like this.");
            return;
        }
        m_result_load = m_result->safety_factor / std::max(1.f, m_target_fos);
    } else {
        m_result_load = m_magnitude * unit_to_newton();
        if (m_result_load <= 0.) {
            m_result->error = _u8L("Enter a load greater than zero.");
            return;
        }
    }
    SA::scale_load(*m_result, m_result_load);
    m_parent.set_as_dirty();
}

void GLGizmoStrength::render_load_input()
{
    ImGui::AlignTextToFramePadding();
    m_imgui->text(_L("Load"));
    ImGui::SameLine(m_imgui->scaled(4.f));
    if (m_imgui->radio_button(_L("Maximum load"), m_load_mode == LoadMode::MaxLoad)) {
        m_load_mode = LoadMode::MaxLoad;
        update_result();
    }
    ImGui::SameLine();
    if (m_imgui->radio_button(_L("Check a load"), m_load_mode == LoadMode::CheckLoad)) {
        m_load_mode = LoadMode::CheckLoad;
        update_result();
    }

    ImGui::AlignTextToFramePadding();
    if (m_load_mode == LoadMode::MaxLoad) {
        m_imgui->text(_L("Safety factor"));
        ImGui::SameLine(m_imgui->scaled(4.f));
        ImGui::PushItemWidth(m_imgui->scaled(4.f));
        if (ImGui::BBLDragFloat("##target_fos", &m_target_fos, 0.05f, 1.f, 10.f, "%.2f")) {
            m_target_fos = std::clamp(m_target_fos, 1.f, 10.f);
            update_result();
        }
    } else {
        m_imgui->text(_L("Load"));
        ImGui::SameLine(m_imgui->scaled(4.f));
        ImGui::PushItemWidth(m_imgui->scaled(4.f));
        if (ImGui::BBLDragFloat("##magnitude", &m_magnitude, 0.1f, 0.f, 10000.f, "%.2f")) {
            m_magnitude = std::max(0.f, m_magnitude);
            update_result();
        }
    }
    ImGui::SameLine();
    int unit = int(m_unit);
    if (m_imgui->combo(std::string("##unit"), {_u8L("kg"), _u8L("lb"), _u8L("N")}, unit, 0, 0.f, m_imgui->scaled(3.f))) {
        m_unit = Unit(unit);
        update_result();
    }
}

void GLGizmoStrength::render_direction_input()
{
    ImGui::AlignTextToFramePadding();
    m_imgui->text(_L("Direction"));
    static const std::array<std::pair<const char *, Vec3d>, 6> presets{{{"-Z", -Vec3d::UnitZ()},
                                                                        {"+Z", Vec3d::UnitZ()},
                                                                        {"-X", -Vec3d::UnitX()},
                                                                        {"+X", Vec3d::UnitX()},
                                                                        {"-Y", -Vec3d::UnitY()},
                                                                        {"+Y", Vec3d::UnitY()}}};
    for (size_t i = 0; i < presets.size(); ++i) {
        ImGui::SameLine(i == 0 ? m_imgui->scaled(4.f) : 0.f);
        if (m_imgui->button(presets[i].first)) {
            Plater::TakeSnapshot snapshot(wxGetApp().plater(), m_desc.at("aim").ToUTF8().data(), UndoRedo::SnapshotType::GizmoAction);
            set_direction(presets[i].second);
        }
    }

    // Free direction, normalised after each edit.
    const char *labels[3] = {"X", "Y", "Z"};
    Vec3f       dir       = m_direction.cast<float>();
    ImGui::Dummy(ImVec2(0.f, 0.f));
    for (int axis = 0; axis < 3; ++axis) {
        ImGui::SameLine(axis == 0 ? m_imgui->scaled(4.f) : 0.f);
        m_imgui->text(labels[axis]);
        ImGui::SameLine();
        ImGui::PushItemWidth(m_imgui->scaled(2.5f));
        if (ImGui::BBLDragFloat((std::string("##dir") + labels[axis]).c_str(), &dir[axis], 0.01f, -1.f, 1.f, "%.2f"))
            set_direction(dir.cast<double>());
    }
}

void GLGizmoStrength::render_result()
{
    if (!m_result)
        return;
    ImGui::Separator();
    if (!m_result->ok()) {
        m_imgui->warning_text(wxString::FromUTF8(m_result->error));
        return;
    }
    const SA::Result &r = *m_result;
    if (m_result_stale)
        m_imgui->text_colored(ImGuiWrapper::COL_RED, _L("Results are out of date. Analyze again."));

    if (m_load_mode == LoadMode::MaxLoad) {
        m_imgui->text_colored(ImGuiWrapper::COL_ORCA, (boost::format(_u8L("Maximum load %1%")) % format_load(m_result_load)).str());
        m_imgui->text((boost::format(_u8L("at safety factor %1$.2f")) % r.safety_factor).str());
    } else {
        const std::string fos = (boost::format(_u8L("Safety factor %1$.2f")) % r.safety_factor).str();
        if (failed()) {
            // Flash the verdict so a failing part cannot be missed.
            const double t     = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start_time).count();
            const float  pulse = float(0.5 + 0.5 * std::sin(2. * PI * FLASH_HZ * t));
            const std::string text = fos + "  " + _u8L("Will break");
            const ImVec2 pos  = ImGui::GetCursorScreenPos();
            const ImVec2 size = ImGui::CalcTextSize(text.c_str());
            const float  pad  = m_imgui->scaled(0.2f);
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(pos.x - pad, pos.y - pad), ImVec2(pos.x + size.x + pad, pos.y + size.y + pad),
                                                      ImGui::GetColorU32(ImVec4(0.85f, 0.1f, 0.1f, 0.35f + 0.65f * pulse)), 3.f);
            m_imgui->text_colored(ImVec4(1.f, 1.f, 1.f, 1.f), text);
            m_parent.request_extra_frame();
        } else {
            const bool low = r.safety_factor < m_target_fos;
            m_imgui->text_colored(low ? ImGuiWrapper::COL_ORANGE_LIGHT : ImGuiWrapper::COL_ORCA,
                                  fos + "  " + (low ? _u8L("Little margin") : _u8L("Should hold")));
        }
        m_imgui->text((boost::format(_u8L("Load %1%")) % format_load(m_result_load)).str());
    }

    const char       *axes[3]  = {"X", "Y", "Z"};
    const std::string modes[4] = {_u8L("bending"), _u8L("tension"), _u8L("shear"), _u8L("torsion")};
    m_imgui->text((boost::format(_u8L("Peak stress %1$.1f MPa of %2$.1f MPa allowed")) % r.stress % r.allowable).str());
    m_imgui->text((boost::format(_u8L("Weakest cut: %1% = %2$.1f mm, mostly %3%")) % axes[r.axis] % r.critical_point[r.axis] %
                   modes[int(r.mode)]).str());
    if (r.axis == 2)
        m_imgui->text(_L("The weakest cut runs between layers."));
    if (r.kt > 1.01)
        m_imgui->text((boost::format(_u8L("Stress raised %1$.1fx by a hole")) % r.kt).str());
    m_imgui->text((boost::format(_u8L("Deflection at load %1$.2f mm")) % r.deflection).str());

    ImGui::Separator();
    if (m_imgui->bbl_checkbox(_L("Stress colors"), m_show_stress))
        m_mesh_phase = -1.;
    ImGui::SameLine();
    if (m_imgui->bbl_checkbox(_L("Deflected shape"), m_show_deflection))
        m_mesh_phase = -1.;
    if (m_show_deflection) {
        if (m_imgui->bbl_checkbox(_L("Animate"), m_animate))
            m_mesh_phase = -1.;
        ImGui::SameLine();
        if (m_imgui->bbl_checkbox(_L("Exaggerate"), m_exaggerate))
            m_mesh_phase = -1.;
    }
    if (m_show_stress)
        m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _L("Blue: little stress. Red: at the breaking point."));

    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, m_material_note + ". " + m_print_note + ".");
    if (!m_from_slices)
        m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _L("Slice the plate to include modifiers and per-part settings."));
    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _L("Estimate from beam theory, not FEA."));
}

void GLGizmoStrength::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c->selection_info()->model_object())
        return;

    const float approx_height = m_imgui->scaled(20.f);
    y                         = std::min(y, bottom_limit - approx_height);
    GizmoImguiSetNextWIndowPos(x, y, ImGuiCond_Always, 0.0f, 0.0f);

    ImGuiWrapper::push_toolbar_style(m_parent.get_scale());
    const float f_scale = m_parent.get_gizmos_manager().get_layout_scale();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 4.0f * f_scale));
    GizmoImguiBegin(get_name(), ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    m_imgui->text_colored(ImVec4(FixtureColor.r(), FixtureColor.g(), FixtureColor.b(), 1.f), m_desc.at("fixture") + ": " + _L("Left mouse button"));
    m_imgui->text_colored(ImVec4(LoadColor.r(), LoadColor.g(), LoadColor.b(), 1.f), m_desc.at("load") + ": " + _L("Right mouse button"));

    // Whole faces are what a fixture or a load usually covers, so smart fill comes first.
    ImGui::AlignTextToFramePadding();
    m_imgui->text(_L("Tool type"));
    ImGui::SameLine(m_imgui->scaled(4.f));
    if (m_imgui->radio_button(_L("Face"), m_tool_type == ToolType::SMART_FILL))
        m_tool_type = ToolType::SMART_FILL;
    ImGui::SameLine();
    if (m_imgui->radio_button(_L("Brush"), m_tool_type == ToolType::BRUSH))
        m_tool_type = ToolType::BRUSH;
    if (m_tool_type == ToolType::BRUSH) {
        ImGui::AlignTextToFramePadding();
        m_imgui->text(m_desc.at("cursor_size"));
        ImGui::SameLine(m_imgui->scaled(4.f));
        ImGui::PushItemWidth(m_imgui->scaled(7.f));
        m_imgui->bbl_slider_float_style("##cursor_radius", &m_cursor_radius, CursorRadiusMin, CursorRadiusMax, "%.2f", 1.0f, true);
    }

    ImGui::Separator();
    render_load_input();
    render_direction_input();
    ImGui::Separator();

    if (m_imgui->button(_L("Analyze")))
        run_analysis();
    ImGui::SameLine();
    GLGizmoUtils::render_tooltip_button(m_imgui, m_parent, m_shortcuts, x, y);
    ImGui::SameLine();
    if (m_imgui->button(m_desc.at("remove_all"))) {
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Reset selection", UndoRedo::SnapshotType::GizmoAction);
        for (auto &selector : m_triangle_selectors) {
            selector->reset();
            selector->request_update_render_data(true);
        }
        update_model_object();
    }
    ImGui::SameLine();
    GLGizmoUtils::begin_right_aligned_buttons({_L("Done")});
    if (m_imgui->button(_L("Done")))
        m_parent.reset_all_gizmos();

    render_result();

    ImGui::PopStyleVar(1);
    GizmoImguiEnd();
    ImGuiWrapper::pop_toolbar_style();
}

} // namespace Slic3r::GUI
