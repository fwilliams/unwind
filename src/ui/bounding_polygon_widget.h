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
    bool mouse_scroll(float delta_y);

    bool post_draw(BoundingCage::KeyFrameIterator it, int current_vertex_id);

private:
    static constexpr const int NoElement = -1;

    State& state;

    BoundingCage::KeyFrameIterator current_active_keyframe;
    int current_edit_element = NoElement;

    //bool is_currently_on_slice = false;
    //int current_slice_id = 0;

    igl::opengl::glfw::Viewer* viewer;

    GLuint empty_vao = 0;
    struct {
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

        GLint color_location = -1;
    } polygon;

    struct {
        GLuint fbo = 0;
        GLuint texture = 0;
        glm::ivec2 texture_size = { 1024, 1024 };
    } offscreen;

    struct {
        GLuint program = 0;
        GLint position_location = -1;
        GLint size_location = -1;
        GLint texture_location = -1;
    } blit;

    glm::vec2 slice_position = { 0.f, 0.f };
    float slice_size = 500.f;

    struct {
        glm::vec2 offset;
        float zoom = 1.f;
    } view;

    struct {
        glm::ivec2 current_position;
        glm::vec2 down_position;
        bool is_down = false;
        float scroll = 0.f;
    } mouse_state;
};

#endif // BOUNDING_POLYGON_WIDGET_H
