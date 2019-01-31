#ifndef BOUNDING_WIDGET_3D_H
#define BOUNDING_WIDGET_3D_H

#include "state.h"
#include "volume_rendering_2.h"


namespace igl { namespace opengl { namespace glfw { class Viewer; }}}

class Bounding_Widget_3d {

public:
    Bounding_Widget_3d(State& state);

    void initialize(igl::opengl::glfw::Viewer* viewer);
    bool pre_draw(float current_cut_index);
    bool post_draw(const glm::vec4& viewport);

    vr::VolumeRenderer volume_renderer;

private:
    void update_bounding_geometry(const Eigen::MatrixXd& cage_V, const Eigen::MatrixXi& cage_F);

    State& _state;
    igl::opengl::glfw::Viewer* _viewer;

    glm::vec4 _last_viewport;
};

#endif // BOUNDING_WIDGET_3D_H
