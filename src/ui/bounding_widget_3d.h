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
    bool post_draw();

    vr::VolumeRenderer volume_renderer;

private:
    State& _state;
    igl::opengl::glfw::Viewer* _viewer;


};

#endif // BOUNDING_WIDGET_3D_H
