#pragma once

#include "GLGizmoPainterBase.hpp"

#include "libslic3r/Feature/StrengthAnalysis/StrengthAnalysis.hpp"
#include "slic3r/GUI/I18N.hpp"

#include <cereal/types/map.hpp>

#include <array>
#include <chrono>
#include <optional>

namespace Slic3r::GUI {

// Paint where a part is held and where it is loaded, aim the load, and get either
// the largest load it takes at a chosen safety factor or the safety factor at a
// given load, for the part as it will be printed.
//
// The load case is a what-if check, not part of the print, so it lives in this
// gizmo for the session (with undo/redo) and never touches the model or the
// project file.
class GLGizmoStrength : public GLGizmoPainterBase
{
public:
    GLGizmoStrength(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id);

    void render_painter_gizmo() override;
    bool on_mouse(const wxMouseEvent &mouse_event) override;

protected:
    void        on_render_input_window(float x, float y, float bottom_limit) override;
    std::string on_get_name() const override;

    wxString handle_snapshot_action_name(bool shift_down, Button button_down) const override;

    std::string get_gizmo_entering_text() const override { return _u8L("Entering strength check"); }
    std::string get_gizmo_leaving_text() const override { return _u8L("Leaving strength check"); }
    std::string get_action_snapshot_name() const override { return _u8L("Strength check editing"); }

    // Left button paints the fixture, right button the load.
    ColorRGBA get_cursor_sphere_left_button_color() const override { return { FixtureColor.r(), FixtureColor.g(), FixtureColor.b(), 0.25f }; }
    ColorRGBA get_cursor_sphere_right_button_color() const override { return { LoadColor.r(), LoadColor.g(), LoadColor.b(), 0.25f }; }

    void on_load(cereal::BinaryInputArchive &ar) override;
    void on_save(cereal::BinaryOutputArchive &ar) const override;

private:
    bool on_init() override;

    void update_model_object() override;
    void update_from_model_object(bool first_update) override;

    void             on_opening() override {}
    void             on_shutdown() override;
    PainterGizmoType get_painter_type() const override;

    enum class Unit { Kilogram, Pound, Newton };
    // Newcomers rarely know what load to expect, so the default answers "how much can it take?".
    enum class LoadMode { MaxLoad, CheckLoad };

    // Paint of one model part, with the facet count it was painted on so that
    // paint never gets applied to a mesh that has since changed.
    struct PartPaint
    {
        size_t                                  facets = 0;
        TriangleSelector::TriangleSplittingData data;
        template<class Archive> void serialize(Archive &ar) { ar(facets, data); }
    };
    using Paint = std::vector<PartPaint>;

    static constexpr size_t StressBands = 8;

    double      unit_to_newton() const;
    std::string format_load(double newtons) const;
    Transform3d arrow_matrix() const;
    // Mouse over the arrow, in screen space.
    bool arrow_hit(const Vec2d &mouse) const;
    void aim_arrow(const Vec2d &mouse);
    void set_direction(const Vec3d &direction);
    void invalidate_result() { m_result_stale = m_base_result.has_value(); }
    void update_load_point();
    std::optional<Feature::StrengthAnalysis::PrintedLayers> sliced_layers(double z_min) const;
    void run_analysis();
    // Rescales the unit-load result to the current load mode and value.
    void update_result();
    bool failed() const;
    bool show_result_mesh() const;
    void update_result_mesh(double phase);
    void render_model(GLModel &model, const Transform3d &matrix, const ColorRGBA &color) const;
    void render_load_input();
    void render_direction_input();
    void render_result();

    static const ColorRGBA FixtureColor;
    static const ColorRGBA LoadColor;

    std::map<size_t, Paint> m_paint; // keyed by ModelObject id
    Vec3d                   m_direction  = -Vec3d::UnitZ();
    float                   m_magnitude  = 1.f;
    Unit                    m_unit       = Unit::Kilogram;
    LoadMode                m_load_mode  = LoadMode::MaxLoad;
    float                   m_target_fos = 1.5f;

    std::optional<Vec3d> m_load_point; // world, centroid of the painted load
    double               m_arrow_length  = 20.;
    bool                 m_arrow_hovered = false;
    bool                 m_aiming        = false;

    // The analysis runs at 1 N; everything in it is linear in the load.
    std::optional<Feature::StrengthAnalysis::Result> m_base_result;
    std::optional<Feature::StrengthAnalysis::Result> m_result; // at m_result_load
    double                                           m_result_load  = 0.; // N
    bool                                             m_result_stale = false;
    std::string                                      m_material_note;
    std::string                                      m_print_note;
    bool                                             m_from_slices = false;

    // Result views: stress colours and the deflected shape, optionally animated.
    bool                                  m_show_stress = false;
    bool                                  m_show_deflection = false;
    bool                                  m_animate     = true;
    bool                                  m_exaggerate  = true;
    indexed_triangle_set                  m_result_mesh; // world, all model parts
    std::vector<double>                   m_vertex_utilization;
    std::vector<Vec3d>                    m_vertex_displacement;
    double                                m_mesh_phase = -1.;
    std::array<GLModel, StressBands>      m_stress_models;
    std::chrono::steady_clock::time_point m_start_time = std::chrono::steady_clock::now();

    GLModel m_arrow;
    GLModel m_ring;
    GLModel m_marker;

    std::map<std::string, wxString>            m_desc;
    std::vector<std::pair<wxString, wxString>> m_shortcuts;
};

} // namespace Slic3r::GUI
