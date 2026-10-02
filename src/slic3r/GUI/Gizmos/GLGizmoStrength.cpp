#include "GLGizmoStrength.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
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

const ColorRGBA AxisColors[3] = {{0.9f, 0.2f, 0.2f, 0.6f}, {0.2f, 0.75f, 0.2f, 0.6f}, {0.25f, 0.45f, 1.f, 0.6f}};
const ColorRGBA CriticalColor = {0.9f, 0.1f, 0.1f, 1.f};

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
    ar(m_paint, m_direction.x(), m_direction.y(), m_direction.z(), m_magnitude, m_unit);
}

void GLGizmoStrength::on_load(cereal::BinaryInputArchive &ar)
{
    ar(m_paint, m_direction.x(), m_direction.y(), m_direction.z(), m_magnitude, m_unit);
    invalidate_result();
    GLGizmoPainterBase::on_load(ar);
}

void GLGizmoStrength::update_model_object()
{
    const ModelObject *mo = m_c->selection_info()->model_object();
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

    if (first_update)
        m_result.reset();
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

Vec3d GLGizmoStrength::force() const
{
    const double scale = m_unit == Unit::Kilogram ? KG_TO_N : m_unit == Unit::Pound ? LB_TO_N : 1.;
    return m_direction * double(m_magnitude) * scale;
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

void GLGizmoStrength::render_painter_gizmo()
{
    const Selection &selection = m_parent.get_selection();

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));

    render_triangles(selection);
    m_c->object_clipper()->render_cut();
    m_c->instances_hider()->render_cut();
    if (!m_arrow_hovered && !m_aiming)
        render_cursor();

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
    if (m_result && m_result->ok() && !m_result_stale)
        render_model(m_marker, Geometry::translation_transform(m_result->critical_point) * Geometry::scale_transform(0.04 * m_arrow_length / 0.35),
                     CriticalColor);

    glsafe(::glDisable(GL_BLEND));
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
    lc.force = force();

    // The analysis follows the settings the object will actually be printed with.
    DynamicPrintConfig cfg = wxGetApp().preset_bundle->full_config();
    cfg.apply(mo->config.get(), true);
    SA::PrintSettings print;
    const double      nozzle = cfg.opt_float("nozzle_diameter", 0);
    print.layer_height       = cfg.opt_float("layer_height");
    print.wall_loops         = cfg.opt_int("wall_loops");
    print.infill             = cfg.option<ConfigOptionPercent>("sparse_infill_density")->value / 100.;
    print.wall_width         = cfg.get_abs_value("inner_wall_line_width", nozzle);
    if (print.wall_width <= 0.)
        print.wall_width = 1.125 * nozzle;

    const auto  *types    = cfg.option<ConfigOptionStrings>("filament_type");
    const int    extruder = cfg.has("extruder") ? std::max(1, cfg.opt_int("extruder")) : 1;
    const std::string type = types && !types->values.empty() ? types->values[std::min<size_t>(extruder - 1, types->values.size() - 1)] : "";
    const SA::Material *material = SA::find_material(type);
    m_material_note = material ? material->name :
                                 (boost::format(_u8L("%1% (unknown filament type %2%)")) % SA::default_material().name % type).str();

    m_print_note = (boost::format(_u8L("%1% walls, %2% infill, %3% mm layers")) % print.wall_loops %
                    (std::to_string(int(std::round(print.infill * 100.))) + "%") % print.layer_height).str();

    m_result       = SA::analyse(mesh, lc, material ? *material : SA::default_material(), print);
    m_result_stale = false;
    m_parent.set_as_dirty();
}

void GLGizmoStrength::render_direction_input()
{
    ImGui::AlignTextToFramePadding();
    m_imgui->text(_L("Load"));
    ImGui::SameLine(m_imgui->scaled(4.f));
    ImGui::PushItemWidth(m_imgui->scaled(4.f));
    if (ImGui::BBLDragFloat("##magnitude", &m_magnitude, 0.1f, 0.f, 10000.f, "%.2f")) {
        m_magnitude = std::max(0.f, m_magnitude);
        invalidate_result();
    }
    ImGui::SameLine();
    int unit = int(m_unit);
    if (m_imgui->combo(std::string("##unit"), {_u8L("kg"), _u8L("lb"), _u8L("N")}, unit, 0, 0.f, m_imgui->scaled(3.f))) {
        m_unit = Unit(unit);
        invalidate_result();
    }

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

    const ImVec4 color = r.safety_factor < 1. ? ImGuiWrapper::COL_RED :
                         r.safety_factor < 2. ? ImGuiWrapper::COL_ORANGE_LIGHT :
                                                ImGuiWrapper::COL_ORCA;
    const wxString verdict = r.safety_factor < 1. ? _L("Likely to break") :
                             r.safety_factor < 2. ? _L("Little margin") :
                                                    _L("Should hold");
    m_imgui->text_colored(color, (boost::format(_u8L("Safety factor %1$.2f")) % r.safety_factor).str() + "  " + verdict.ToUTF8().data());

    const char *axes[3]  = {"X", "Y", "Z"};
    const std::string modes[4] = {_u8L("bending"), _u8L("tension"), _u8L("shear"), _u8L("torsion")};
    m_imgui->text((boost::format(_u8L("Peak stress %1$.1f MPa of %2$.1f MPa allowed")) % r.stress % r.allowable).str());
    m_imgui->text((boost::format(_u8L("Weakest cut: %1% = %2$.1f mm, mostly %3%")) % axes[r.axis] % r.critical_point[r.axis] %
                   modes[int(r.mode)]).str());
    if (r.axis == 2)
        m_imgui->text(_L("The weakest cut runs between layers."));
    if (r.kt > 1.01)
        m_imgui->text((boost::format(_u8L("Stress raised %1$.1fx by a hole")) % r.kt).str());
    m_imgui->text((boost::format(_u8L("Deflection at load %1$.2f mm")) % r.deflection).str());
    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, m_material_note + ", " + m_print_note);
    m_imgui->text_colored(ImGuiWrapper::COL_GREY_LIGHT, _L("Estimate from beam theory, not FEA."));
}

void GLGizmoStrength::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c->selection_info()->model_object())
        return;

    const float approx_height = m_imgui->scaled(16.f);
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
