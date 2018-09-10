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


uniform vec2 window_size;

uniform vec3 ll;
uniform vec3 lr;
uniform vec3 ul;
uniform vec3 ur;

out vec3 uv;

const float Size = 500;

void main() {
    vec2 p = positions[gl_VertexID];
    p.x *= Size / window_size.x;
    p.y *= Size / window_size.y;

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

    glGenVertexArrays(1, &plane.vao);


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
}

bool Bounding_Polygon_Widget::mouse_move(int mouse_x, int mouse_y) {
    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);

    bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;

    float normalized_mouse_x = ((mouse_x / w) - 0.5f) * 2.f;
    float normalized_mouse_y = -(((mouse_y / h) - 0.5f) * 2.f);

    if (left_mouse && is_currently_on_slice && current_edit_element) {
        // map to rendering window
        float mapped_mouse_x = normalized_mouse_x / (500.f / w);
        float mapped_mouse_y = normalized_mouse_y / (500.f / h);

        current_edit_element->x() = std::max(std::min(mapped_mouse_x, 1.f), -1.f);
        current_edit_element->y() = std::max(std::min(mapped_mouse_y, 1.f), -1.f);
        return true;
    }

    return false;
}

bool Bounding_Polygon_Widget::mouse_down(int button, int modifier) {
    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);

    bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;

    float normalized_mouse_x = ((viewer->current_mouse_x / w) - 0.5f) * 2.f;
    float normalized_mouse_y = -(((viewer->current_mouse_y / h) - 0.5f) * 2.f);

    if (left_mouse && is_currently_on_slice) {
        // map to rendering window
        float mapped_mouse_x = normalized_mouse_x / (500.f / w);
        float mapped_mouse_y = normalized_mouse_y / (500.f / h);

        //for (Eigen::Vector2f& p : state.bounding_polygon.polygon_slices[current_slice_id].polygon) {
        //    float d = sqrt((p.x() - mapped_mouse_x) * (p.x() - mapped_mouse_x) +
        //                   (p.y() - mapped_mouse_y) * (p.y() - mapped_mouse_y));

        //    if (d < SelectionRadius) {
        //        current_edit_element = &p;
        //        return true;
        //    }
        //}
    }

    return false;
}

bool Bounding_Polygon_Widget::mouse_up(int button, int modifier) {
    current_edit_element = nullptr;
    return false;
}

bool Bounding_Polygon_Widget::post_draw(BoundingCage::KeyFrameIterator kf, int current_vertex_id) {
    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);


    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice");
    glUseProgram(plane.program);
    glBindVertexArray(plane.vao);

    glUniform2f(plane.window_size_location, w, h);

    glm::vec3 x_axis = glm::vec3(G3f(kf->orientation().row(0)));
    glm::vec3 y_axis = glm::vec3(G3f(kf->orientation().row(1)));
    glm::vec3 center = glm::vec3(G3f(kf->center())) / glm::vec3(state.volume_rendering.parameters.volume_dimensions);

    glm::vec3 ll = center + -1.f * x_axis + -1.f * y_axis;
    glm::vec3 ul = center +  1.f * x_axis + -1.f * y_axis;
    glm::vec3 lr = center + -1.f * x_axis +  1.f * y_axis;
    glm::vec3 ur = center +  1.f * x_axis +  1.f * y_axis;

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

    if (is_currently_on_slice) {
        const Eigen::MatrixXd PV = kf->vertices_2d();
        glDisable(GL_DEPTH_TEST);

        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render overlay");
        glUseProgram(polygon.program);
        glBindVertexArray(polygon.vao);

        //std::vector<GLfloat> data;
        //data.reserve(state.bounding_polygon.nPoints * 2);
        //const std::vector<Eigen::Vector2f>& p = state.bounding_polygon.polygon_slices[current_slice_id].polygon;
        //for (const Eigen::Vector2f& v : p) {
        //    data.push_back(v[0]);
        //    data.push_back(v[1]);
        //}

        glBindBuffer(GL_ARRAY_BUFFER, polygon.vbo);
        //glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat),
        //    data.data(), GL_STATIC_DRAW);

        glPointSize(8.f);
        glLineWidth(3.f);


        glUniform2f(polygon.window_size_location, static_cast<float>(width), static_cast<float>(height));
        glUniform4f(polygon.color, 0.85f, 0.85f, 0.f, 1.f);

        //glDrawArrays(GL_LINE_LOOP, 0, state.bounding_polygon.nPoints);
        //glDrawArrays(GL_POINTS, 0, state.bounding_polygon.nPoints);

        glPopDebugGroup();

        bool left_mouse = glfwGetMouseButton(viewer->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;
        float mouse_x = ((viewer->current_mouse_x / w) - 0.5f) * 2.f;
        float mouse_y = -(((viewer->current_mouse_y / h) - 0.5f) * 2.f);

        // map to rendering window
        float mapped_mouse_x = mouse_x / (500.f / w);
        float mapped_mouse_y = mouse_y / (500.f / h);

        // mouse
        GLfloat data_mouse[2] = { mapped_mouse_x, mapped_mouse_y };
        glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(GLfloat), data_mouse, GL_STATIC_DRAW);
        glPointSize(12.f);
        glUniform4f(polygon.color, 0.15f, 0.85f, 0.15f, 1.f);
        glDrawArrays(GL_POINTS, 0, 1);

        //for (const Eigen::Vector2f& p : state.bounding_polygon.polygon_slices[current_slice_id].polygon) {
        //    float d = sqrt((p.x() - mapped_mouse_x) * (p.x() - mapped_mouse_x) +
        //                   (p.y() - mapped_mouse_y) * (p.y() - mapped_mouse_y));

        //    if (d < SelectionRadius) {
        //        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render overlay highlight");
        //        GLfloat data[2] = { p.x(), p.y() };
        //        glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(GLfloat), data, GL_STATIC_DRAW);
        //        glPointSize(12.f);
        //        glUniform4f(polygon.color, 0.f, 0.25f, 0.75f, 1.f);
        //        glDrawArrays(GL_POINTS, 0, 1);
        //        glPopDebugGroup();
        //    }
        //}

        glBindVertexArray(0);
        glUseProgram(0);
        glEnable(GL_DEPTH_TEST);
    }

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render slice UI");

    //if (ImGui::SliderInt("Number of vertices", &state.bounding_polygon.nPoints, 0, 10)) {
    //    state.bounding_polygon.polygon_slices.clear();
    //}
    ImGui::SameLine();
    ImGui::Text("%s", "Please note that changing this value removes all previous vertices");

    //if (ImGui::Button("Add Slice")) {
    //    auto it = std::lower_bound(state.bounding_polygon.polygon_slices.begin(),
    //        state.bounding_polygon.polygon_slices.end(), current_vertex_id,
    //        [](const State::BoundingPolygon::Slice& slice, int id) {
    //            return slice.vertex_id < id;
    //        });

    //    if (state.bounding_polygon.polygon_slices.empty() || (it->vertex_id != current_vertex_id)) {
    //        State::BoundingPolygon::Slice slice;
    //        slice.vertex_id = current_vertex_id;
    //        slice.polygon = create_default_points(state.bounding_polygon.nPoints, 0.5f);
    //        state.bounding_polygon.polygon_slices.insert(it, slice);
    //        current_slice_id = std::distance(state.bounding_polygon.polygon_slices.begin(), it);
    //    }
    //}

    //auto it = std::find_if(state.bounding_polygon.polygon_slices.begin(),
    //    state.bounding_polygon.polygon_slices.end(),
    //    [id = current_vertex_id](const State::BoundingPolygon::Slice& slice) {
    //        return slice.vertex_id == id;
    //    }
    //);
    //is_currently_on_slice = it != state.bounding_polygon.polygon_slices.end();
    //if (is_currently_on_slice) {
    //    current_slice_id = std::distance(state.bounding_polygon.polygon_slices.begin(), it);
    //}

    //ImGui::Separator();
    //ImGui::Text("%s", "Polygon bounds");
    //ImGui::Text("%i", static_cast<int>(state.bounding_polygon.polygon_slices.size()));

    //if (ImGui::Button("< Prev")) {
    //    current_slice_id = std::max(current_slice_id - 1, 0);
    //    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    //}
    //ImGui::SameLine();
    //int slices = static_cast<int>(state.bounding_polygon.polygon_slices.size() - 1);
    //if (ImGui::SliderInt("#sliceid", &current_slice_id, 0, slices)) {
    //    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    //}
    //ImGui::SameLine();
    //if (ImGui::Button("Next >")) {
    //    current_slice_id = std::min(current_slice_id + 1, slices);
    //    current_vertex_id = state.bounding_polygon.polygon_slices[current_slice_id].vertex_id;
    //}
    glPopDebugGroup();
    return false;
}
