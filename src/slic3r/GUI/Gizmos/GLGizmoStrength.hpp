#pragma once

#include "GLGizmoPainterBase.hpp"

#include "libslic3r/Feature/StrengthAnalysis/StrengthAnalysis.hpp"
#include "slic3r/GUI/I18N.hpp"

#include <cereal/types/map.hpp>

#include <optional>

namespace Slic3r::GUI {

// Paint where a part is held and where it is loaded, aim the load, and get a
// quick safety factor for the part as it will be printed.
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

    // Paint of one model part, with the facet count it was painted on so that
    // paint never gets applied to a mesh that has since changed.
    struct PartPaint
    {
        size_t                                  facets = 0;
        TriangleSelector::TriangleSplittingData data;
        template<class Archive> void serialize(Archive &ar) { ar(facets, data); }
    };
    using Paint = std::vector<PartPaint>;

    Vec3d  force() const;
    Transform3d arrow_matrix() const;
    // Mouse over the arrow, in screen space.
    bool   arrow_hit(const Vec2d &mouse) const;
    void   aim_arrow(const Vec2d &mouse);
    void   set_direction(const Vec3d &direction);
    void   invalidate_result() { m_result_stale = m_result.has_value(); }
    void   update_load_point();
    void   run_analysis();
    void   render_model(GLModel &model, const Transform3d &matrix, const ColorRGBA &color) const;
    void   render_direction_input();
    void   render_result();

    static const ColorRGBA FixtureColor;
    static const ColorRGBA LoadColor;

    std::map<size_t, Paint> m_paint; // keyed by ModelObject id
    Vec3d                   m_direction = -Vec3d::UnitZ();
    float                   m_magnitude = 1.f;
    Unit                    m_unit      = Unit::Kilogram;

    std::optional<Vec3d> m_load_point;  // world, centroid of the painted load
    double               m_arrow_length = 20.;
    bool                 m_arrow_hovered = false;
    bool                 m_aiming        = false;

    std::optional<Feature::StrengthAnalysis::Result> m_result;
    bool                                             m_result_stale = false;
    std::string                                      m_material_note;
    std::string                                      m_print_note;

    GLModel m_arrow;
    GLModel m_ring;
    GLModel m_marker;

    std::map<std::string, wxString>            m_desc;
    std::vector<std::pair<wxString, wxString>> m_shortcuts;
};

} // namespace Slic3r::GUI
