#ifndef BOUNDING_POLYGON_WIDGET_H
#define BOUNDING_POLYGON_WIDGET_H

#include <Eigen/Core>
#include "glad/glad.h"
#include "state.h"

namespace igl { namespace opengl { namespace glfw { class Viewer; }}}

class Bounding_Polygon_Widget {
public:
    Bounding_Polygon_Widget(State& state);

    void initialize(igl::opengl::glfw::Viewer* viewer);

    bool mouse_move(int mouse_x, int mouse_y);
    bool mouse_down(int button, int modifier);
    bool mouse_up(int button, int modifier);

    bool post_draw(BoundingCage::KeyFrameIterator it, int current_vertex_id);

private:
    State& state;

    Eigen::Vector2f* current_edit_element = nullptr;

    bool is_currently_on_slice = false;
    int current_slice_id = 0;

    igl::opengl::glfw::Viewer* viewer;
    struct {
        GLuint vao = 0;
        //GLuint vbo = 0;
        GLuint program = 0;

        GLint window_size_location = -1;
        GLint ll_location = -1;
        GLint lr_location = -1;
        GLint ul_location = -1;
        GLint ur_location = -1;

        GLint texture_location = -1;
        GLint tf_location = -1;
    } plane;

    struct {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint program = 0;

        GLint window_size_location = -1;
        GLint color = -1;
    } polygon;


};

#endif // BOUNDING_POLYGON_WIDGET_H
