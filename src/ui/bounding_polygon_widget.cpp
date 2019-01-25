#include "bounding_polygon_widget.h"
#include "volume_exporter.h"

#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui.h>
#include <utils/glm_conversion.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "volume_exporter.h"

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
        out_color = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else {
        float v = texture(tex, uv).r;
        out_color = vec4(vec3(v * 1.5), 1.0);
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

std::vector<glm::vec2> vertices_2d_to_glm(const Eigen::MatrixXd& vertices) {
    std::vector<glm::vec2> result(vertices.rows());
    for (int i = 0; i < vertices.rows(); ++i) {
        result[i] = G2f(vertices.row(i));
    }
    return result;
}

std::vector<glm::vec3> vertices_3d_to_glm(const Eigen::MatrixXd& vertices) {
    std::vector<glm::vec3> result(vertices.rows());
    for (int i = 0; i < vertices.rows(); ++i) {
        result[i] = G3f(vertices.row(i));
    }
    return result;
}

} // namespace


glm::vec2 Bounding_Polygon_Widget::convert_position_mainwindow_to_keyframe(const glm::vec2& p) {
    glm::vec2 window_ll = position;
    glm::vec2 window_ur = position + size;

    // Map mouse into [0, 1]^2 in the subwindow
    glm::vec2 normalized_mouse = (p - window_ll) / (window_ur - window_ll);

    // Convert to [-1, 1]
    glm::vec2 mapped_mouse = (normalized_mouse - glm::vec2(0.5f)) * 2.f;

    return mapped_mouse * view.zoom + view.offset;
}

glm::vec2 Bounding_Polygon_Widget::convert_position_keyframe_to_ndc(const glm::vec2& p) {
    glm::vec2 ret = p - view.offset;
    return ret / view.zoom;
}

bool Bounding_Polygon_Widget::intersects(const glm::ivec2& p) const {
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);

    const glm::ivec2 p_tx(p.x, window_height - p.y);
    const glm::ivec2 ll = position;
    const glm::ivec2 ur = position + size;
    return p_tx.x >= ll.x && p_tx.x <= ur.x && p_tx.y >= ll.y && p_tx.y <= ur.y;
}

std::tuple<int, float, glm::vec2> Bounding_Polygon_Widget::closest_edge(const glm::vec2& p) {
    std::vector<glm::vec2> vertices = vertices_2d_to_glm(selection.current_active_keyframe->vertices_2d());

    float min_dist = std::numeric_limits<float>::max();
    int index = -1;
    glm::vec2 point;

    for (int i = 0; i < vertices.size(); i++) {
        glm::vec2 v1 = vertices[i];
        glm::vec2 v2 = vertices[(i + 1) % vertices.size()];

        glm::vec2 e = glm::normalize(v2 - v1);
        glm::vec2 v_to_p = p - v1;

        glm::vec2 lp = v1 + e*glm::dot(e, v_to_p);
        float lam = glm::clamp(glm::dot(lp - v1, e), 0.f, glm::distance(v1, v2));
        lp = vertices[i] + lam*e;

        float dist = glm::distance(p, lp);

        if (dist < min_dist) {
            min_dist = dist;
            index = i;
            point = lp;
        }
    }

    return std::make_tuple(index, min_dist, point);
}

Bounding_Polygon_Widget::Bounding_Polygon_Widget(State& state) : state(state) {}

void Bounding_Polygon_Widget::initialize(igl::opengl::glfw::Viewer* viewer) {
    this->viewer = viewer;

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

bool Bounding_Polygon_Widget::mouse_move(int mouse_x, int mouse_y) {
    glm::ivec2 window_size;
    glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);

    mouse_state.current_position = glm::ivec2(mouse_x, mouse_y);

    if (!intersects(mouse_state.current_position)) {
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

    // We early out for the movement, so that we don't accidentally edit a node while
    // shifting things around
    if (mouse_state.is_left_button_down && selection.current_edit_element >= 0) {
        // current_mouse is in pixel coordinates
        glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y };
        // Zooming and panning
        mouse_state.down_position = current_mouse;

        glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);
        glm::vec2 ctr = G2f(selection.current_active_keyframe->centroid_2d());
        std::vector<glm::vec2> bbox_v = bbox_vertices();
//        bbox_v[selection.closest_edge_index] = kf_mouse;
        for (int i = 0; i < 4; i++) { bbox_v[i] -= ctr; }

//        switch (selection.closest_vertex_index) {
//        case 0:
//            min_u = std::min(current_mouse[0], max_u);
//            min_v = std::min(current_mouse[1], max_v);
//            break;
//        case 1:
//            max_u = std::max(current_mouse[0], min_u);
//            min_v = std::min(current_mouse[1], max_v);
//            break;
//        case 2:
//            max_u = std::max(current_mouse[0], min_u);
//            max_v = std::max(current_mouse[1], min_v);
//            break;
//        case 3:
//            min_u = std::min(current_mouse[0], max_u);
//            max_v = std::max(current_mouse[1], min_v);
//            break;
//        }

//        vertices.push_back(ctr + glm::vec2(min_u, min_v));
//        vertices.push_back(ctr + glm::vec2(max_u, min_v));
//        vertices.push_back(ctr + glm::vec2(max_u, max_v));
//        vertices.push_back(ctr + glm::vec2(min_u, max_v));

        glm::vec2 assign_mouse = kf_mouse - ctr;
        switch (selection.closest_vertex_index) {
        case 0:
            min_u = std::min(assign_mouse[0], max_u);
            min_v = std::min(assign_mouse[1], max_v);
            break;
        case 1:
            max_u = std::max(assign_mouse[0], min_u);
            min_v = std::min(assign_mouse[1], max_v);
            break;
        case 2:
            max_u = std::max(assign_mouse[0], min_u);
            max_v = std::max(assign_mouse[1], min_v);
            break;
        case 3:
            min_u = std::min(assign_mouse[0], max_u);
            max_v = std::max(assign_mouse[1], min_v);
            break;
        }

        std::cout << "min_u = " << min_u << ", max_u = " << max_u << ", min_v = " << min_v << ", max_v = " << max_v << std::endl;

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

        Eigen::RowVector2d kf_mouse_eigen(kf_mouse.x, kf_mouse.y);
        bool success = selection.current_active_keyframe->move_centroid_2d(kf_mouse_eigen);
        if (!success) {
            // TODO: Handle this case if ever we decide to force the centroid to be inside the polygon
        }
    }
    return false;
}

bool Bounding_Polygon_Widget::mouse_down(int button, int modifier) {
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
            state.logger->debug("SELECTION MATCHED A VERTEX {}", selection.closest_vertex_index);
        }

        if (left_mouse && selection.matched_center) {
            selection.current_edit_element = CenterElement;
            state.logger->debug("SELECTION MATCHED A CENTER");
        }
    }

    return intersects(mouse_state.current_position);
}

bool Bounding_Polygon_Widget::mouse_up(int button, int modifier) {
    selection.current_edit_element = NoElement;
    mouse_state.is_left_button_down = false;
    mouse_state.is_right_button_down = false;
    return false;
}

bool Bounding_Polygon_Widget::mouse_scroll(float delta_y) {
    mouse_state.scroll = delta_y;

    if (!intersects(mouse_state.current_position)) {
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



std::vector<glm::vec2> Bounding_Polygon_Widget::bbox_vertices() {
    glm::vec2 ctr = G2f(selection.current_active_keyframe->centroid_2d());
    std::vector<glm::vec2> vertices;
    vertices.push_back(ctr + glm::vec2(min_u, min_v));
    vertices.push_back(ctr + glm::vec2(max_u, min_v));
    vertices.push_back(ctr + glm::vec2(max_u, max_v));
    vertices.push_back(ctr + glm::vec2(min_u, max_v));

    return vertices;
}

std::pair<int, float> Bounding_Polygon_Widget::closest_vertex(const glm::vec2& p) {
    // Convert vertices from Eigen to glm

    std::vector<glm::vec2> vertices = bbox_vertices();

    float min_dist = std::numeric_limits<float>::max();
    int index = -1;

    for (int i = 0; i < vertices.size(); ++i) {
        float d = glm::distance(vertices[i], p);
        if (d < min_dist) {
            index = i;
            min_dist = d;
        }
    }

    return std::make_pair(index, min_dist);
}


void Bounding_Polygon_Widget::update_selection() {
    glm::ivec2 window_size;
    glfwGetWindowSize(viewer->window, &window_size.x, &window_size.y);
    glm::vec2 current_mouse = { viewer->current_mouse_x, window_size.y - viewer->current_mouse_y }; // In main window pixel space
    glm::vec2 kf_mouse = convert_position_mainwindow_to_keyframe(current_mouse);                    // In keyframe ndc

    std::pair<int, float> cv = closest_vertex(kf_mouse);
    std::tuple<int, float, glm::vec2> ce = closest_edge(kf_mouse);

    selection.matched_vertex = std::get<1>(cv) < view.zoom * SelectionRadius;
    selection.matched_edge = std::get<1>(ce) < view.zoom * SelectionRadius;
    float dist_to_center = glm::distance(kf_mouse, G2f(selection.current_active_keyframe->centroid_2d()));

    selection.matched_center = dist_to_center < view.zoom * SelectionRadius;

    if (selection.matched_center && selection.matched_edge) {
        if (std::get<1>(cv) < dist_to_center) {
            selection.matched_center = false;
        } else {
            selection.matched_center = true;
        }
    }

    selection.closest_edge_index = std::get<0>(ce);
    selection.closest_vertex_index = std::get<0>(cv);
    selection.closest_edge_point = std::get<2>(ce);
}

void Bounding_Polygon_Widget::draw_polygon(const std::vector<glm::vec2>& pixel_space_points,
                                           glm::vec4 color, float point_size, float line_width, PolygonDrawMode mode) {
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
}

bool Bounding_Polygon_Widget::post_draw(BoundingCage::KeyFrameIterator kf, int current_vertex_id) {
    selection.current_active_keyframe = kf;


    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);


    glBindFramebuffer(GL_FRAMEBUFFER, offscreen.fbo);
    glViewport(0, 0, offscreen.texture_size.x, offscreen.texture_size.y);
    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice");
    glUseProgram(plane.program);
    glBindVertexArray(empty_vao);

    glm::vec3 x_axis = glm::vec3(G3f(kf->right()));
    glm::vec3 y_axis = glm::vec3(G3f(kf->up()));
    glm::vec3 kf_center = glm::vec3(G3f(kf->center()));
    //glm::vec3 center = kf_center / glm::vec3(state.volume_rendering.parameters.volume_dimensions);


    glm::vec3 center = kf_center + view.offset.x * x_axis + view.offset.y * y_axis;
    glm::vec3 ll = center - x_axis * view.zoom - y_axis * view.zoom;
    glm::vec3 lr = center + x_axis * view.zoom - y_axis * view.zoom;
    glm::vec3 ul = center - x_axis * view.zoom + y_axis * view.zoom;
    glm::vec3 ur = center + x_axis * view.zoom + y_axis * view.zoom;

    //float z = ((view.zoom - 1.f) * -1.f) + 1.f;
    ////glm::vec3 ll = center + (-1.f + view.offset.x) * z * x_axis + (-1.f + view.offset.y) * z * y_axis;
    //glm::vec3 ll = center + (-1.f ) * z * x_axis + (-1.f ) * z * y_axis;
    ll /= glm::vec3(state.volume_rendering.parameters.volume_dimensions);
    ////glm::vec3 ul = center + (1.f + view.offset.x) * z * x_axis + (-1.f + view.offset.y) * z * y_axis;
    //glm::vec3 ul = center + ( 1.f ) * z * x_axis + (-1.f ) * z * y_axis;
    ul /= glm::vec3(state.volume_rendering.parameters.volume_dimensions);
    ////glm::vec3 lr = center + (-1.f + view.offset.x) * z * x_axis + (1.f + view.offset.y) * z * y_axis;
    //glm::vec3 lr = center + (-1.f) * z * x_axis + ( 1.f ) * z * y_axis;
    lr /= glm::vec3(state.volume_rendering.parameters.volume_dimensions);
    ////glm::vec3 ur = center + (1.f + view.offset.x) * z * x_axis + (1.f + view.offset.y) * z * y_axis;
    //glm::vec3 ur = center + ( 1.f ) * z * x_axis + ( 1.f ) * z * y_axis;
    ur /= glm::vec3(state.volume_rendering.parameters.volume_dimensions);

    glUniform3fv(plane.ll_location, 1, glm::value_ptr(ll));
    glUniform3fv(plane.lr_location, 1, glm::value_ptr(lr));
    glUniform3fv(plane.ul_location, 1, glm::value_ptr(ul));
    glUniform3fv(plane.ur_location, 1, glm::value_ptr(ur));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, state.volume_rendering.volume_texture);
    glUniform1i(plane.texture_location, 0);

    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    glUseProgram(0);
    glPopDebugGroup();



    //
    // Render Bounding Polygon
    //

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render polygon");
    {
//        std::vector<glm::vec2> vertices_2d = vertices_2d_to_glm(selection.current_active_keyframe->vertices_2d());
//        draw_polygon(vertices_2d, polygon_line_color, polygon_point_size, polygon_line_width, PolygonDrawMode::PointsAndLines);
//        if (selection.matched_vertex && selection.current_edit_element == NoElement) {
//            std::vector<glm::vec2> selected_vertices_2d;
//            selected_vertices_2d.push_back(vertices_2d[selection.closest_vertex_index]);
//            draw_polygon(selected_vertices_2d, available_point_color, selected_point_size, 1.f, PolygonDrawMode::Points);
//        } else if (selection.current_edit_element != NoElement) {
//            std::vector<glm::vec2> selected_vertices_2d;
//            selected_vertices_2d.push_back(vertices_2d[selection.closest_vertex_index]);
//            draw_polygon(selected_vertices_2d, selected_point_color, selected_point_size, 1.f, PolygonDrawMode::Points);
//        }

//        if (selection.matched_edge && !selection.matched_vertex && selection.current_edit_element == NoElement) {
//            std::vector<glm::vec2> selected_vertices_2d;
//            selected_vertices_2d.push_back(selection.closest_edge_point);
//            draw_polygon(selected_vertices_2d, available_point_color, split_point_size, 1.f, PolygonDrawMode::Points);
//        }

        if (selection.matched_center) {
            std::vector<glm::vec2> selected_vertices_2d;
            selected_vertices_2d.push_back(G2f(selection.current_active_keyframe->centroid_2d()));
            draw_polygon(selected_vertices_2d, selected_center_point_color, selected_center_point_size, 1.f, PolygonDrawMode::Points);
        } else {
            std::vector<glm::vec2> selected_vertices_2d;
            selected_vertices_2d.push_back(G2f(selection.current_active_keyframe->centroid_2d()));
            draw_polygon(selected_vertices_2d, center_point_color, center_point_size, 1.f, PolygonDrawMode::Points);
        }

        glm::vec2 ctr = G2f(selection.current_active_keyframe->centroid_2d());
        std::vector<glm::vec2> bbox_vertices;
        bbox_vertices.push_back(ctr + glm::vec2(min_u, min_v));
        bbox_vertices.push_back(ctr + glm::vec2(max_u, min_v));
        bbox_vertices.push_back(ctr + glm::vec2(max_u, max_v));
        bbox_vertices.push_back(ctr + glm::vec2(min_u, max_v));

        glm::vec4 bbox_color = glm::vec4(0.5f, 0.5f, 0.9f, 1.f);

        draw_polygon(bbox_vertices, bbox_color, 5.f, 2.f, PolygonDrawMode::PointsAndLines);

//        glm::vec4 bounds = keyframe_bounds(state.cage);
//        glm::vec2 ctr = G2f(selection.current_active_keyframe->centroid_2d());
//        std::vector<glm::vec2> bbox_vertices;
//        bbox_vertices.push_back(ctr + glm::vec2(bounds[0], bounds[2]));
//        bbox_vertices.push_back(ctr + glm::vec2(bounds[1], bounds[2]));
//        bbox_vertices.push_back(ctr + glm::vec2(bounds[1], bounds[3]));
//        bbox_vertices.push_back(ctr + glm::vec2(bounds[0], bounds[3]));

//        glm::vec4 bbox_color = glm::vec4(0.5f, 0.5f, 0.9f, 1.f);

//        draw_polygon(bbox_vertices, bbox_color, 5.f, 2.f, PolygonDrawMode::PointsAndLines);
    }

    glPopDebugGroup();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);




    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Texture Blit");
    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);


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
    glPopDebugGroup();

    glEnable(GL_DEPTH_TEST);

    //    ImGui::SliderFloat2("Window Size", glm::value_ptr(size), 0.f, h);
    //    ImGui::SliderFloat2("Window Position", glm::value_ptr(position), 0.f, h);

    return false;
}
