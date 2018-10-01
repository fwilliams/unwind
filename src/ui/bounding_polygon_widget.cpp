#include "bounding_polygon_widget.h"

#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui.h>
#include <utils/glm_conversion.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#pragma optimize ("", off)

namespace {

constexpr float SelectionRadius = 0.15f;

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

uniform vec2 window_size;
const float Size = 500.f;

void main() {
    vec2 p = position;
    p.x *= Size / window_size.x;
    p.y *= Size / window_size.y;

    gl_Position = vec4(p, 0.0, 1.0);
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

vec2 positions[6] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 1.0, -1.0),
    vec2( 1.0,  1.0),

    vec2(-1.0, -1.0),
    vec2( 1.0,  1.0),
    vec2(-1.0,  1.0)
);

out vec2 uv;

uniform vec2 position;
uniform vec2 size;

void main() {
    vec2 p = positions[gl_VertexID];
    uv = (p + vec2(1.0)) / 2;

    p = p * size + position;
    gl_Position = vec4(p, 0, 1.0);

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

std::vector<glm::vec2> convert_vertices_2d(const Eigen::MatrixXd& vertices) {
    std::vector<glm::vec2> result(vertices.rows());
    for (int i = 0; i < vertices.rows(); ++i) {
        result[i] = G2f(vertices.row(i));
    }
    return result;
}

std::vector<glm::vec3> convert_vertices_3d(const Eigen::MatrixXd& vertices) {
    std::vector<glm::vec3> result(vertices.rows());
    for (int i = 0; i < vertices.rows(); ++i) {
        result[i] = G3f(vertices.row(i));
    }
    return result;
}

} // namespace

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

    polygon.window_size_location = glGetUniformLocation(polygon.program, "window_size");
    polygon.color = glGetUniformLocation(polygon.program, "color");

    glGenVertexArrays(1, &polygon.vao);
    glBindVertexArray(polygon.vao);

    glGenBuffers(1, &polygon.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, polygon.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, false, 2 * sizeof(GLfloat), nullptr);
    glBindVertexArray(0);


    igl::opengl::create_shader_program(BlitVertexShader, BlitFragmentShader, {}, blit.program);
    GLuint blit_program = 0;
    blit.position_location = glGetUniformLocation(blit.program, "position");
    blit.size_location = glGetUniformLocation(blit.program, "size");
    blit.texture_location = glGetUniformLocation(blit.program, "tex");



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
    glm::ivec2 size;
    glfwGetWindowSize(viewer->window, &size.x, &size.y);

    bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;

    if (left_mouse && current_edit_element != NoElement) {
        glm::vec2 current_mouse = { mouse_x, mouse_y };
        glm::vec2 normalized_mouse = (current_mouse / glm::vec2(size) - 0.5f) * 2.f;
        glm::vec2 mapped_mouse = normalized_mouse / (500.f / glm::vec2(size));
        mapped_mouse = glm::clamp(mapped_mouse, glm::vec2(-1.f), glm::vec2(1.f));
        mapped_mouse.y *= -1.f;

        const bool validate_2d = true;
        const bool validate_3d = false;
        bool success = current_active_keyframe->move_point_2d(current_edit_element,
            Eigen::RowVector2d(mapped_mouse.x, mapped_mouse.y), validate_2d, validate_3d);

        if (!success) {
            current_edit_element = NoElement;
        }
        return true;
    }

    return false;
}

bool Bounding_Polygon_Widget::mouse_down(int button, int modifier) {
    glm::ivec2 size;
    glfwGetWindowSize(viewer->window, &size.x, &size.y);

    bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;


    if (left_mouse) {
        glm::vec2 current_mouse = { viewer->current_mouse_x, viewer->current_mouse_y };
        glm::vec2 normalized_mouse = (current_mouse / glm::vec2(size) - 0.5f) * 2.f;
        glm::vec2 mapped_mouse = normalized_mouse / (500.f / glm::vec2(size));
        mapped_mouse.y *= -1.f;

        std::vector<glm::vec2> vertices = convert_vertices_2d(current_active_keyframe->vertices_2d());

        for (int i = 0; i < vertices.size(); ++i) {
            float d = glm::distance(vertices[i], mapped_mouse);
            if (d < SelectionRadius) {
                current_edit_element = i;
                break;
            }
        }
    }

    return false;
}

bool Bounding_Polygon_Widget::mouse_up(int button, int modifier) {
    current_edit_element = NoElement;
    return false;
}

bool Bounding_Polygon_Widget::post_draw(BoundingCage::KeyFrameIterator kf, int current_vertex_id) {
    current_active_keyframe = kf;

    glDisable(GL_DEPTH_TEST);


    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice");
    glBindFramebuffer(GL_FRAMEBUFFER, offscreen.fbo);
    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(plane.program);
    glBindVertexArray(empty_vao);

    glm::vec3 x_axis = glm::vec3(G3f(kf->right()));
    glm::vec3 y_axis = glm::vec3(G3f(kf->up()));
    glm::vec3 kf_center = glm::vec3(G3f(kf->center()));
    glm::vec3 center = kf_center / glm::vec3(state.volume_rendering.parameters.volume_dimensions);

    glm::vec3 ll = center + -1.f * x_axis + -1.f * y_axis;
    glm::vec3 ul = center + 1.f * x_axis + -1.f * y_axis;
    glm::vec3 lr = center + -1.f * x_axis + 1.f * y_axis;
    glm::vec3 ur = center + 1.f * x_axis + 1.f * y_axis;

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
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glPopDebugGroup();


    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Texture Blit");
    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);


    glUseProgram(blit.program);
    glBindVertexArray(empty_vao);
    glUniform2f(blit.position_location, slice_position.x / w, slice_position.y / h);
    glUniform2f(blit.size_location, slice_size / w, slice_size / h);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, offscreen.texture);
    glUniform1i(blit.texture_location, 0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    glUseProgram(0);



    glPopDebugGroup();

    glEnable(GL_DEPTH_TEST);


    ImGui::SliderFloat("Window Size", &slice_size, -h, h);
    ImGui::SliderFloat2("Window Position", glm::value_ptr(slice_position), 0.f, h);


    //int width;
    //int height;
    //glfwGetWindowSize(viewer->window, &width, &height);
    //float w = static_cast<float>(width);
    //float h = static_cast<float>(height);


    //glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice");


    //glPopDebugGroup();

    //const Eigen::MatrixXd PV = kf->vertices_2d();
    //glDisable(GL_DEPTH_TEST);

    //glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render overlay");
    //glUseProgram(polygon.program);
    //glBindVertexArray(polygon.vao);

    //glm::vec2 size = glm::vec2(

    //    //glm::length(ul - ll) * glm::vec3(state.volume_rendering.parameters.volume_dimensions),
    //    //glm::length(ur - lr) * glm::vec3(state.volume_rendering.parameters.volume_dimensions)
    //);
    //std::vector<glm::vec2> vertex_data = convert_vertices_2d(current_active_keyframe->vertices_2d());
    //for (glm::vec2& v : vertex_data) {
    //    //v /= size;
    //    //v = (kf_center.x + v.x * x_axis + v.y * y_axis) / glm::vec3(state.volume_rendering.parameters.volume_dimensions);


    //    //v *= state.volume_rendering.parameters.volume_dimensions_rcp;


    //    
    //}

    //glBindBuffer(GL_ARRAY_BUFFER, polygon.vbo);
    //glBufferData(GL_ARRAY_BUFFER, vertex_data.size() * sizeof(glm::vec2),
    //    vertex_data.data(), GL_STATIC_DRAW);

    //glPointSize(8.f);
    //glLineWidth(3.f);


    //glUniform2f(polygon.window_size_location, static_cast<float>(width), static_cast<float>(height));
    //glUniform4f(polygon.color, 0.85f, 0.85f, 0.f, 1.f);

    //glDrawArrays(GL_LINE_LOOP, 0, vertex_data.size());
    //glDrawArrays(GL_POINTS, 0, vertex_data.size());

    //glPopDebugGroup();

    //bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;
    //glm::vec2 mouse = { viewer->current_mouse_x / w, viewer->current_mouse_y / h };
    //mouse -= 0.5f;
    //mouse *= 2.f;

    //// map to rendering window
    //glm::vec2 mapped_mouse = mouse / (500.f / glm::vec2(w, h));
    //mapped_mouse.y *= -1.f;

    //// mouse
    //glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(GLfloat), glm::value_ptr(mapped_mouse), GL_STATIC_DRAW);
    //glPointSize(12.f);
    //glUniform4f(polygon.color, 0.15f, 0.85f, 0.15f, 1.f);
    //glDrawArrays(GL_POINTS, 0, 1);

    //for (const glm::vec2& p : vertex_data) {
    //    const float d = glm::distance(p, mapped_mouse);

    //    if (d < SelectionRadius) {
    //        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render overlay highlight");
    //        glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(GLfloat), glm::value_ptr(p), GL_STATIC_DRAW);
    //        glPointSize(12.f);
    //        glUniform4f(polygon.color, 0.f, 0.25f, 0.75f, 1.f);
    //        glDrawArrays(GL_POINTS, 0, 1);
    //        glPopDebugGroup();
    //    }
    //}

    //glBindVertexArray(0);
    //glUseProgram(0);
    //glEnable(GL_DEPTH_TEST);

    //glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice UI");

    ////if (ImGui::SliderInt("Number of vertices", &state.bounding_polygon.nPoints, 0, 10)) {
    ////    state.bounding_polygon.polygon_slices.clear();
    ////}
    //ImGui::SameLine();
    //ImGui::Text("%s", "Please note that changing this value removes all previous vertices");

    ////if (ImGui::Button("Add Slice")) {
    ////    auto it = std::lower_bound(state.bounding_polygon.polygon_slices.begin(),
    ////        state.bounding_polygon.polygon_slices.end(), current_vertex_id,
    ////        [](const State::BoundingPolygon::Slice& slice, int id) {
    ////            return slice.vertex_id < id;
    ////        });

    ////    if (state.bounding_polygon.polygon_slices.empty() || (it->vertex_id != current_vertex_id)) {
    ////        State::BoundingPolygon::Slice slice;
    ////        slice.vertex_id = current_vertex_id;
    ////        slice.polygon = create_default_points(state.bounding_polygon.nPoints, 0.5f);
    ////        state.bounding_polygon.polygon_slices.insert(it, slice);
    ////        current_slice_id = std::distance(state.bounding_polygon.polygon_slices.begin(), it);
    ////    }
    ////}

    ////auto it = std::find_if(state.bounding_polygon.polygon_slices.begin(),
    ////    state.bounding_polygon.polygon_slices.end(),
    ////    [id = current_vertex_id](const State::BoundingPolygon::Slice& slice) {
    ////        return slice.vertex_id == id;
    ////    }
    ////);
    ////is_currently_on_slice = it != state.bounding_polygon.polygon_slices.end();
    ////if (is_currently_on_slice) {
    ////    current_slice_id = std::distance(state.bounding_polygon.polygon_slices.begin(), it);
    ////}

    ////ImGui::Separator();
    ////ImGui::Text("%s", "Polygon bounds");
    ////ImGui::Text("%i", static_cast<int>(state.bounding_polygon.polygon_slices.size()));

    ////if (ImGui::Button("< Prev")) {
    ////    current_slice_id = std::max(current_slice_id - 1, 0);
    ////    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    ////} 
    ////ImGui::SameLine();
    ////int slices = static_cast<int>(state.bounding_polygon.polygon_slices.size() - 1);
    ////if (ImGui::SliderInt("#sliceid", &current_slice_id, 0, slices)) {
    ////    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    ////}
    ////ImGui::SameLine();
    ////if (ImGui::Button("Next >")) {
    ////    current_slice_id = std::min(current_slice_id + 1, slices);
    ////    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    ////}
    //glPopDebugGroup();
    return false;
}
