#ifndef BOUNDING_WIDGET_3D_H
#define BOUNDING_WIDGET_3D_H

#include "state.h"
#include "volume_rendering_2.h"
#include "point_line_rendering.h"

namespace igl { namespace opengl { namespace glfw { class Viewer; }}}

class Bounding_Polygon_Menu;

class Bounding_Widget_3d {

public:
    Bounding_Widget_3d(State& state);

    void initialize(igl::opengl::glfw::Viewer* viewer, Bounding_Polygon_Menu* parent);
    bool pre_draw(float current_cut_index);
    bool post_draw(const glm::vec4& viewport, BoundingCage::KeyFrameIterator current_kf, bool draw_straight=false);

    VolumeRenderer volume_renderer;
    PointLineRenderer renderer_2d;

private:
    void update_volume_geometry(const Eigen::MatrixXd& cage_V, const Eigen::MatrixXi& cage_F);
    void update_2d_geometry(BoundingCage::KeyFrameIterator current_kf);

    int cage_polyline_id;
    int current_kf_polyline_id;
    int skeleton_polyline_id;

    State& _state;
    igl::opengl::glfw::Viewer* _viewer;
    Bounding_Polygon_Menu* _parent;

    glm::vec4 _last_viewport;
};

#endif // BOUNDING_WIDGET_3D_H
