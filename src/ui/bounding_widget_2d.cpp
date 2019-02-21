#include "bounding_widget_2d.h"

#include "bounding_polygon_plugin.h"

#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui.h>
#include <utils/glm_conversion.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>

#include <vector>

#include <utils/gl/volume_exporter.h>

#pragma optimize ("", off)

namespace {

constexpr const char* PlaneVertexShader = R"(
#version 150
// Create two triangles that are filling the entire screen [-1, 1]
vec2 positions[6] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 1.0, -1.0),
    vec2( 1.0,  1.0),

    vec2(-1.0, -1.0),
    vec2( 1.0,  1.0),
    vec2(-1.0,  1.0)
);


uniform vec3 ll;
uniform vec3 lr;
uniform vec3 ul;
uniform vec3 ur;

out vec3 uv;

void main() {
    vec2 p = positions[gl_VertexID];
    gl_Position = vec4(p, 0.0, 1.0);

    switch (gl_VertexID) {
        case 0:
        case 3:
            uv = ll;
            break;
        case 1:
            uv = lr;
            break;
        case 2:
        case 4:
            uv = ur;
            break;
        case 5:
            uv = ul;
            break;
    }
}
)";

constexpr const char* PlaneFragmentShader = R"(
#version 150
in vec3 uv;

out vec4 out_color;

uniform sampler3D tex;
uniform sampler1D tf;

void main() {
    // All areas outside the actual texture area should be black
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
        out_color = vec4(0.0, 0.0, 0.0, 0.0);
    }
    else {
        float v = texture(tex, uv).r;
        out_color = vec4(vec3(v), 1.0);
    }
}
)";

constexpr const char* PolygonVertexShader = R"(
#version 150
in vec2 position;

void main() {
    gl_Position = vec4(position, 0.0, 1.0);
}
)";

constexpr const char* PolygonFragmentShader = R"(
#version 150

out vec4 out_color;
uniform vec4 color;

void main() {
  out_color = color;
}
)";

constexpr const char* BlitVertexShader = R"(
#version 150

in vec2 in_position;
in vec2 in_uv;

out vec2 uv;

void main() {
    gl_Position = vec4(in_position, 0.0, 1.0);
    uv = in_uv;
}
)";

constexpr const char* BlitFragmentShader = R"(
#version 150

in vec2 uv;

out vec4 out_color;

uniform sampler2D tex;
void main() {
    out_color = texture(tex, uv);
}   

)";

struct BlitData {
    float data[4]; // pos[2] + uv[2]
};

} // namespace

Bounding_Polygon_Widget::Bounding_Polygon_Widget(State& state) : state(state) {}

glm::vec2 Bounding_Polygon_Widget::convert_position_mainwindow_to_keyframe(const glm::vec2& p) const {
    glm::vec2 window_ll = position;
    glm::vec2 window_ur = position + size;

    // Map mouse into [0, 1]^2 in the subwindow
    glm::vec2 normalized_mouse = (p - window_ll) / (window_ur - window_ll);

    // Convert to [-1, 1]
    glm::vec2 mapped_mouse = (normalized_mouse - glm::vec2(0.5f)) * 2.f;

    return mapped_mouse * view.zoom + view.offset;
}

glm::vec2 Bounding_Polygon_Widget::convert_position_keyframe_to_ndc(const glm::vec2& p) const {
    glm::vec2 ret = p - view.offset;
    return ret / view.zoom;
}

bool Bounding_Polygon_Widget::is_point_in_widget(glm::ivec2 p) const {

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);

    const glm::ivec2 p_tx(p.x, window_height - p.y);
    const glm::ivec2 ll = position;
    const glm::ivec2 ur = position + size;
    return p_tx.x >= ll.x && p_tx.x <= ur.x && p_tx.y >= ll.y && p_tx.y <= ur.y;
}

void Bounding_Polygon_Widget::update_selection() {
    // Helper function to find the closest vertex in vertices to p
    auto closest_vertex = [](const glm::vec2& p, const Eigen::MatrixXd& vertices) -> std::pair<int, float> {
        float min_dist = std::numeric_limits<float>::max();
        int index = -1;

        for (int i = 0; i < vertices.rows(); ++i) {
            float d = glm::distance(G2f(vertices.row(i)), p);
            if (d < min_dist) {
                index = i;
                min_dist = d;
            }
        }

        return std::make_pair(index, min_dist);
    };

    glm::ivec2 window_size;
    glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);
    glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y }; // In main window pixel space
    glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);                    // In keyframe ndc

    BoundingCage::KeyFrameIterator kf = selection.current_active_keyframe;
    std::pair<int, float> cv = closest_vertex(kf_mouse, kf->bounding_box_vertices_rotated_2d());

    selection.matched_vertex = std::get<1>(cv) < view.zoom * SelectionRadius;
    float dist_to_center = glm::distance(kf_mouse, G2f(selection.current_active_keyframe->centroid_2d()));

    selection.matched_center = dist_to_center < view.zoom * SelectionRadius;
    selection.closest_vertex_index = std::get<0>(cv);
}

void Bounding_Polygon_Widget::deinitialize() {
    std::vector<GLuint> vertex_arrays = {
        empty_vao,
        polygon.vao,
        blit.vao
    };
    std::vector<GLuint> buffers = {
        polygon.vbo,
        blit.vbo
    };

    glDeleteProgram(plane.program);
    glDeleteProgram(polygon.program);
    glDeleteVertexArrays(vertex_arrays.size(), vertex_arrays.data());
    glDeleteBuffers(buffers.size(), buffers.data());
    glDeleteTextures(1, &offscreen.texture);
    glDeleteFramebuffers(1, &offscreen.fbo);
}

void Bounding_Polygon_Widget::initialize(igl::opengl::glfw::Viewer* viewer, Bounding_Polygon_Menu *parent) {
    this->viewer = viewer;
    this->parent = parent;

    igl::opengl::create_shader_program(PlaneVertexShader,
                                       PlaneFragmentShader, {}, plane.program);

    plane.window_size_location = glGetUniformLocation(plane.program, "window_size");
    plane.ll_location = glGetUniformLocation(plane.program, "ll");
    plane.lr_location = glGetUniformLocation(plane.program, "lr");
    plane.ul_location = glGetUniformLocation(plane.program, "ul");
    plane.ur_location = glGetUniformLocation(plane.program, "ur");
    plane.texture_location = glGetUniformLocation(plane.program, "tex");
    plane.tf_location = glGetUniformLocation(plane.program, "tf");

    glGenVertexArrays(1, &empty_vao);


    igl::opengl::create_shader_program(PolygonVertexShader, PolygonFragmentShader, {},
                                       polygon.program);

    polygon.color_location = glGetUniformLocation(polygon.program, "color");

    glGenVertexArrays(1, &polygon.vao);
    glBindVertexArray(polygon.vao);

    glGenBuffers(1, &polygon.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, polygon.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, false, 2 * sizeof(GLfloat), nullptr);
    glBindVertexArray(0);


    igl::opengl::create_shader_program(BlitVertexShader, BlitFragmentShader, {}, blit.program);
    blit.texture_location = glGetUniformLocation(blit.program, "tex");

    glGenVertexArrays(1, &blit.vao);
    glBindVertexArray(blit.vao);

    glGenBuffers(1, &blit.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, blit.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(BlitData), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, false, sizeof(BlitData), reinterpret_cast<void*>(2 * sizeof(float)));
    glBindVertexArray(0);


    glGenTextures(1, &offscreen.texture);
    glBindTexture(GL_TEXTURE_2D, offscreen.texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, offscreen.texture_size.x,
                 offscreen.texture_size.y, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &offscreen.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, offscreen.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           offscreen.texture, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

bool Bounding_Polygon_Widget::mouse_move(int mouse_x, int mouse_y, bool in_focus) {
    glm::ivec2 window_size;
    glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);

    mouse_state.current_position = glm::ivec2(mouse_x, mouse_y);
    if (!in_focus) { //!is_point_in_widget(glm::ivec2(mouse_x, mouse_y)) ||
        return false;
    }

    update_selection();

    if (mouse_state.is_right_button_down) {
        //constexpr float InteractionScaleFactor = 0.005f;

        glm::vec2 current_mouse = { mouse_x, window_size.y - mouse_y };
        glm::vec2 delta_mouse_pixels = mouse_state.down_position - current_mouse;
        glm::vec2 delta_mouse_kf = delta_mouse_pixels / glm::vec2(window_size) * view.zoom * 2.f;
        //delta_mouse_kf *= InteractionScaleFactor;

        mouse_state.down_position = current_mouse;

        view.offset += delta_mouse_kf;

        //view.offset = glm::clamp(view.offset, glm::vec2(-1.f), glm::vec2(1.f));

        return true;
    }

    if (mouse_state.is_left_button_down && mouse_state.is_rotate_modifier_down) {
        BoundingCage::KeyFrameIterator kf = selection.current_active_keyframe;

        // current_mouse is in pixel coordinates
        glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };

        glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);
        glm::vec2 kf_mouse_o = convert_position_mainwindow_to_keyframe(mouse_state.down_position);
        glm::vec2 v = kf_mouse - G2f(kf->centroid_2d());
        glm::vec2 vo = kf_mouse_o - G2f(kf->centroid_2d());

        if (glm::length(vo) < 1e-3 || glm::length(v) < 1e-3) {
            return true;
        }
        vo = vo / glm::length(vo);
        v = v / glm::length(v);

        float sign = -glm::sign(glm::dot(glm::vec3(0.0, 0.0, 1.0), glm::cross(glm::vec3(v, 1.0), glm::vec3(vo, 1.0))));
        double cos_theta = glm::dot(vo, v);
        double theta = sign * acos(std::min(std::max(cos_theta, -1.0), 1.0));

        if (!kf->in_bounding_cage()) {
            state.cage.insert_keyframe(kf);
        }
        kf->set_angle(mouse_state.down_angle + theta);
        parent->cage_dirty = true;

        return true;
    }

    // We early out for the movement, so that we don't accidentally edit a node while
    // shifting things around
    if (mouse_state.is_left_button_down && selection.current_edit_element >= 0) {
        // current_mouse is in pixel coordinates
        glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };

        // Zooming and panning
        mouse_state.down_position = current_mouse;

        BoundingCage::KeyFrameIterator kf = selection.current_active_keyframe;
        Eigen::RowVector2d kf_mouse = E2d(convert_position_mainwindow_to_keyframe(current_mouse));
        Eigen::Vector4d bbox = state.cage.keyframe_bounding_box();
        Eigen::RowVector2d assign_mouse = kf->rotated_coords(kf_mouse - kf->centroid_2d());

        switch (selection.closest_vertex_index) {
        case 0:
            bbox[0] = std::min(assign_mouse[0], bbox[1]);
            bbox[2] = std::min(assign_mouse[1], bbox[3]);
            break;
        case 1:
            bbox[1] = std::max(assign_mouse[0], bbox[0]);
            bbox[2] = std::min(assign_mouse[1], bbox[3]);
            break;
        case 2:
            bbox[1] = std::max(assign_mouse[0], bbox[0]);
            bbox[3] = std::max(assign_mouse[1], bbox[2]);
            break;
        case 3:
            bbox[0] = std::min(assign_mouse[0], bbox[1]);
            bbox[3] = std::max(assign_mouse[1], bbox[2]);
            break;
        default:
            state.logger->error("Bad vertex ID in selection. This should never happen. Aborting.");
            exit(EXIT_FAILURE);
        }
        state.cage.set_keyframe_bounding_box(bbox);
        parent->cage_dirty = true;
    }

    if (mouse_state.is_left_button_down && selection.current_edit_element == CenterElement) {
        // current_mouse is in pixel coordinates
        glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };
        // Zooming and panning
        mouse_state.down_position = current_mouse;

        if (!selection.current_active_keyframe->in_bounding_cage()) {
            selection.current_active_keyframe = state.cage.insert_keyframe(selection.current_active_keyframe);
        }

        glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);
        bool success = selection.current_active_keyframe->move_centroid_2d(E2d(kf_mouse));
        if (!success) {
            // TODO: Handle this case if ever we decide to force the centroid to be inside the polygon
        }
        parent->cage_dirty = true;
    }

    return true;
}

bool Bounding_Polygon_Widget::mouse_down(int button, int modifier, bool in_focus) {
    glm::ivec2 window_size;
    glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);

    bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;
    bool right_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS;

    if (left_mouse || right_mouse) {
        mouse_state.is_left_button_down = left_mouse;
        mouse_state.is_right_button_down = right_mouse;

        // current_mouse is in pixel coordinates with the origin at the bottom left
        glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };
        mouse_state.down_position = current_mouse;

        // Selection to move a point on the bounding polygon
        if (left_mouse && selection.matched_vertex) {
            selection.current_edit_element = selection.closest_vertex_index;
        }

        if (left_mouse && selection.matched_center) {
            selection.current_edit_element = CenterElement;
        }

        if (left_mouse && mouse_state.is_rotate_modifier_down) {
            mouse_state.down_angle = selection.current_active_keyframe->angle();
        }
    }

    return in_focus;
}

bool Bounding_Polygon_Widget::mouse_up(int button, int modifier, bool in_focus) {
    selection.current_edit_element = NoElement;
    mouse_state.is_left_button_down = false;
    mouse_state.is_right_button_down = false;
    return false;
}

bool Bounding_Polygon_Widget::mouse_scroll(float delta_y, bool in_focus) {
    mouse_state.scroll = delta_y;

    if (!in_focus) {
        return false;
    }

    if (mouse_state.scroll != 0.f) {
        constexpr float InteractionScaleFactor = 1.f;

        view.zoom += InteractionScaleFactor * -mouse_state.scroll;
        view.zoom = glm::clamp(view.zoom, std::numeric_limits<float>::epsilon(), MaxZoomLevel);

        mouse_state.scroll = 0.f;
    }

    return true;
}

bool Bounding_Polygon_Widget::key_down(int key, int modifiers, bool in_focus) {
    if (!mouse_state.is_rotate_modifier_down && (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)) {
        mouse_state.is_rotate_modifier_down = true;
        mouse_state.which_modifier = key;
    }
    return false;
}

bool Bounding_Polygon_Widget::key_up(int key, int modifiers, bool in_focus) {
    if (mouse_state.is_rotate_modifier_down && key == mouse_state.which_modifier) {
        mouse_state.is_rotate_modifier_down = false;
        mouse_state.which_modifier = 0;
    }
    return false;
}

bool Bounding_Polygon_Widget::post_draw(BoundingCage::KeyFrameIterator kf, bool in_focus) {

    // Helper function to render a closed polygon in one of 3 ways:
    // 1) Just render the vertices
    // 2) Just render the edges
    // 3) Render the vertices and edges
    enum PolygonDrawMode {
        Points,
        Lines,
        PointsAndLines
    };
    auto draw_polygon = [&](const std::vector<glm::vec2>& pixel_space_points, glm::vec4 color,
                            float point_size, float line_width, PolygonDrawMode mode) {
        glUseProgram(polygon.program);
        glBindVertexArray(polygon.vao);

        std::vector<glm::vec2> vertex_data;

        for (int i = 0; i < pixel_space_points.size(); i++) {
            vertex_data.push_back(convert_position_keyframe_to_ndc(pixel_space_points[i]));
        }

        std::vector<float> vertex_data_data;
        for (const glm::vec2& v : vertex_data) {
            vertex_data_data.push_back(v.x);
            vertex_data_data.push_back(v.y);
        }

        glBindBuffer(GL_ARRAY_BUFFER, polygon.vbo);
        glBufferData(GL_ARRAY_BUFFER, vertex_data_data.size() * sizeof(float),
                     vertex_data_data.data(), GL_STATIC_DRAW);

        glPointSize(point_size);
        glLineWidth(line_width);

        glUniform4f(polygon.color_location, color.x, color.y, color.z, color.w);

        switch (mode) {
        case PolygonDrawMode::Points:
            glDrawArrays(GL_POINTS, 0, vertex_data.size());
            break;
        case PolygonDrawMode::Lines:
            glDrawArrays(GL_LINE_LOOP, 0, vertex_data.size());
            break;
        case PolygonDrawMode::PointsAndLines:
            glDrawArrays(GL_POINTS, 0, vertex_data.size());
            glDrawArrays(GL_LINE_LOOP, 0, vertex_data.size());
            break;
        default:
            state.logger->error("Invalid PolygonDrawMode! This should never happen.");
            state.logger->flush();
            exit(1);
        }
    };

    selection.current_active_keyframe = kf;


    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    // All 2D UI gets rendered into a framebuffer texture which we then blit to the screen
    glBindFramebuffer(GL_FRAMEBUFFER, offscreen.fbo);
    glViewport(0, 0, offscreen.texture_size.x, offscreen.texture_size.y);
    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    //
    // Render the slice of the volume for this keyframe into an OpenGL texture
    //
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice");
    {
        glUseProgram(plane.program);
        glBindVertexArray(empty_vao);

        glm::vec3 volume_dims = G3f(state.low_res_volume.dims()); //glm::vec3(state.volume_rendering.parameters.volume_dimensions);

        Eigen::Vector2d offset(view.offset.x, view.offset.y);
        Eigen::Vector2d LL = Eigen::Vector2d(-1.0, -1.0) * view.zoom + offset;
        Eigen::Vector2d UL = Eigen::Vector2d(-1.0,  1.0) * view.zoom + offset;
        Eigen::Vector2d LR = Eigen::Vector2d( 1.0, -1.0) * view.zoom + offset;
        Eigen::Vector2d UR = Eigen::Vector2d( 1.0,  1.0) * view.zoom + offset;

        Eigen::Vector3d LL3 = kf->origin() + LL[0]*kf->right_3d() + LL[1]*kf->up_3d();
        Eigen::Vector3d UL3 = kf->origin() + UL[0]*kf->right_3d() + UL[1]*kf->up_3d();
        Eigen::Vector3d LR3 = kf->origin() + LR[0]*kf->right_3d() + LR[1]*kf->up_3d();
        Eigen::Vector3d UR3 = kf->origin() + UR[0]*kf->right_3d() + UR[1]*kf->up_3d();

        glm::vec3 ll = G3f(LL3) / volume_dims;
        glm::vec3 ul = G3f(UL3) / volume_dims;
        glm::vec3 lr = G3f(LR3) / volume_dims;
        glm::vec3 ur = G3f(UR3) / volume_dims;

        glUniform3fv(plane.ll_location, 1, glm::value_ptr(ll));
        glUniform3fv(plane.lr_location, 1, glm::value_ptr(lr));
        glUniform3fv(plane.ul_location, 1, glm::value_ptr(ul));
        glUniform3fv(plane.ur_location, 1, glm::value_ptr(ur));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, state.low_res_volume.volume_texture);
        glUniform1i(plane.texture_location, 0);

        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        glUseProgram(0);
    }
    glPopDebugGroup();


    //
    // Render the bounding-box, center, and axes into the same texture
    //
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render polygon");
    {
        const glm::vec2 centroid_2d = G2f(kf->centroid_2d());
        const glm::vec2 r_axis = G2f(kf->right_rotated_2d()), u_axis = G2f(kf->up_rotated_2d());

        // Render the center of the bounding box
        float ctr_point_size = selection.matched_center ? selected_center_point_size : center_point_size;
        glm::vec4 ctr_color = selection.matched_center ? selected_center_point_color : center_point_color;
        std::vector<glm::vec2> selected_vertices_2d;
        selected_vertices_2d.push_back(centroid_2d);
        draw_polygon(selected_vertices_2d, ctr_color, ctr_point_size, 1.f, PolygonDrawMode::Points);

        // Render the rotated bounding box
        Eigen::MatrixXd bbox_v = kf->bounding_box_vertices_rotated_2d();
        std::vector<glm::vec2> bbox_vertices;
        for (int i = 0; i < bbox_v.rows(); i++) { bbox_vertices.push_back(G2f(bbox_v.row(i))); }
        draw_polygon(bbox_vertices, bbox_color, 5.f, 2.f, PolygonDrawMode::PointsAndLines);

        // Render the up and right axes
        std::vector<glm::vec2> vertical_line =   { centroid_2d - 100000.f*u_axis, centroid_2d + 100000.f*u_axis };
        std::vector<glm::vec2> horizontal_line = { centroid_2d - 100000.f*r_axis, centroid_2d + 100000.f*r_axis };
        draw_polygon(vertical_line, up_axis_color, 1.f, 1.f, PolygonDrawMode::Lines);
        draw_polygon(horizontal_line, right_axis_color, 1.f, 1.f, PolygonDrawMode::Lines);

        // Render rotation handle
        if (mouse_state.is_rotate_modifier_down && in_focus) {
            glm::ivec2 window_size;
            glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);
            glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };
            glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);

            std::vector<glm::vec2> handle_line = { centroid_2d, kf_mouse };
            draw_polygon(handle_line, rotation_handle_color, 1.f, 1.f, PolygonDrawMode::Lines);

            if (mouse_state.is_left_button_down) {
                glm::vec2 kf_mouse_o = convert_position_mainwindow_to_keyframe(mouse_state.down_position);
                std::vector<glm::vec2> handle_line_o = { centroid_2d, kf_mouse_o };
                draw_polygon(handle_line_o, rotation_handle_reference_color, 1.f, 1.f, PolygonDrawMode::Lines);
            }
        }
    }
    glPopDebugGroup();

    // Restore the framebuffer and viewport
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);



    //
    // Blit the texture we just rendered to the screen
    //
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Texture Blit");
    {
        int width;
        int height;
        glfwGetWindowSize(viewer->window, &width, &height);

        glUseProgram(blit.program);
        glBindVertexArray(blit.vao);

        glBindBuffer(GL_ARRAY_BUFFER, blit.vbo);
        {
            glm::vec2 size_ndc = size / glm::vec2(width, height) * 2.f;
            glm::vec2 pos_ndc = (position / glm::vec2(width, height) - glm::vec2(0.5)) * 2.f;

            BlitData box_ll = {
                pos_ndc.x, pos_ndc.y,
                0.f, 0.f
            };

            BlitData box_lr = {
                pos_ndc.x + size_ndc.x, pos_ndc.y,
                1.f, 0.f
            };

            BlitData box_ul = {
                pos_ndc.x, pos_ndc.y + size_ndc.y,
                0.f, 1.f
            };

            BlitData box_ur = {
                pos_ndc.x + size_ndc.x,  pos_ndc.y + size_ndc.y,
                1.f, 1.f
            };

            std::array<BlitData, 6> blit_data = {
                box_ll, box_lr, box_ur, box_ll, box_ur, box_ul

            };

            glBufferData(GL_ARRAY_BUFFER, blit_data.size() * sizeof(BlitData),
                         blit_data.data(), GL_STATIC_DRAW);


        }
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, offscreen.texture);
        glUniform1i(blit.texture_location, 0);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        glUseProgram(0);
    }
    glPopDebugGroup();
    glEnable(GL_DEPTH_TEST);

    return false;
}
