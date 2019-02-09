#include "point_line_rendering.h"

#include <igl/opengl/create_shader_program.h>

#include <glm/gtc/type_ptr.hpp>

#include <iostream>


constexpr const char* PolygonVertexShader = R"(
#version 150
in vec3 in_position;
in vec4 in_color;

out vec4 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform vec4 global_color;

void main() {
    gl_Position = proj * view * model * vec4(in_position, 1.0);
    color = in_color + global_color;
}
)";

constexpr const char* PolygonFragmentShader = R"(
#version 150

in vec4 color;
out vec4 fragcolor;

void main() {
  fragcolor = color;
}
)";

void PointLineRenderer::init() {
    igl::opengl::create_shader_program(PolygonVertexShader,
                                       PolygonFragmentShader,
                                       {{ "in_position", 0}, {"in_color", 1}},
                                       _gl_state.program);
    _gl_state.uniform_location.global_color = glGetUniformLocation(_gl_state.program, "global_color");
    _gl_state.uniform_location.model = glGetUniformLocation(_gl_state.program, "model");
    _gl_state.uniform_location.view = glGetUniformLocation(_gl_state.program, "view");
    _gl_state.uniform_location.proj = glGetUniformLocation(_gl_state.program, "proj");
}

bool PointLineRenderer::update_polyline_3d(int polyline_id, GLfloat* vertices, GLfloat* colors, GLsizei num_vertices, PolylineStyle style) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "update_polyline_3d");
    if (polyline_id >= _polylines.size() || polyline_id < 0) {
        return false;
    }

    Polyline& polyline = _polylines[polyline_id];

    if (polyline.vao == 0) {
        return false;
    }

    GLsizei num_vertices_bytes = sizeof(GLfloat)*num_vertices*3;
    GLsizei num_colors_bytes = sizeof(GLfloat)*num_vertices*4;

    polyline.style = style;
    polyline.num_vertices = num_vertices;

    glBindBuffer(GL_ARRAY_BUFFER, polyline.pos_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_vertices_bytes, vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, polyline.clr_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_colors_bytes, colors, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glPopDebugGroup();
}

bool PointLineRenderer::update_polyline_3d(int polyline_id, GLfloat* vertices, glm::vec4 color, GLsizei num_vertices, PolylineStyle style) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "update_polyline_3d");
    if (polyline_id >= _polylines.size() || polyline_id < 0) {
        return false;
    }

    Polyline& polyline = _polylines[polyline_id];

    if (polyline.vao == 0) {
        return false;
    }

    GLsizei num_vertices_bytes = sizeof(GLfloat)*num_vertices*3;

    polyline.style = style;
    polyline.num_vertices = num_vertices;
    polyline.global_color = color;

    glBindBuffer(GL_ARRAY_BUFFER, polyline.pos_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_vertices_bytes, vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(polyline.vao);
    glDisableVertexAttribArray(1);
    glBindVertexArray(0);

    glPopDebugGroup();
}


int PointLineRenderer::add_polyline_3d(GLfloat* vertices, GLfloat* colors, GLsizei num_vertices, PolylineStyle style) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "add_polyline_3d");

    Polyline polyline;

    glGenVertexArrays(1, &polyline.vao);
    glBindVertexArray(polyline.vao);

    GLsizei num_vertices_bytes = sizeof(GLfloat)*num_vertices*3;
    GLsizei num_colors_bytes = sizeof(GLfloat)*num_vertices*4;

    glGenBuffers(1, &polyline.pos_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, polyline.pos_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_vertices_bytes, vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat)*3, nullptr);

    glGenBuffers(1, &polyline.clr_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, polyline.clr_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_colors_bytes, colors, GL_STATIC_DRAW);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(GLfloat)*4, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    polyline.global_color = glm::vec4(0.0);
    polyline.style = style;
    polyline.num_vertices = num_vertices;

    int polyline_id;
    if (_free_list.size() > 0) {
        polyline_id = _free_list[_free_list.size()-1];
        _free_list.pop_back();
        _polylines[polyline_id] = polyline;
    } else {
        polyline_id = _polylines.size();
        _polylines.push_back(polyline);
    }

    glPopDebugGroup();

    return polyline_id;
}

int PointLineRenderer::add_polyline_3d(GLfloat* vertices, glm::vec4 color, GLsizei num_vertices, PolylineStyle style) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "add_polyline_3d");

    Polyline polyline;

    glGenVertexArrays(1, &polyline.vao);
    glBindVertexArray(polyline.vao);

    GLsizei num_vertices_bytes = sizeof(GLfloat)*num_vertices*3;
    GLsizei num_colors_bytes = sizeof(GLfloat)*num_vertices*4;

    glGenBuffers(1, &polyline.pos_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, polyline.pos_vbo);
    glBufferData(GL_ARRAY_BUFFER, num_vertices_bytes, vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat)*3, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    polyline.global_color = color;
    polyline.style = style;
    polyline.num_vertices = num_vertices;

    int polyline_id;
    if (_free_list.size() > 0) {
        polyline_id = _free_list[_free_list.size()-1];
        _free_list.pop_back();
        _polylines[polyline_id] = polyline;
    } else {
        polyline_id = _polylines.size();
        _polylines.push_back(polyline);
    }

    glPopDebugGroup();

    return polyline_id;
}


void PointLineRenderer::draw(glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix) {
    GLfloat old_line_width, old_point_size;
    GLboolean old_line_smooth;
    glGetFloatv(GL_LINE_WIDTH, &old_line_width);
    glGetFloatv(GL_POINT_SIZE, &old_point_size);
    glGetBooleanv(GL_LINE_SMOOTH, &old_line_smooth);

    if (_line_antialiasing_enabled) {
        glEnable(GL_LINE_SMOOTH);
    } else {
        glDisable(GL_LINE_SMOOTH);
    }

    glUseProgram(_gl_state.program);

    glUniformMatrix4fv(_gl_state.uniform_location.model, 1, GL_FALSE, glm::value_ptr(model_matrix));
    glUniformMatrix4fv(_gl_state.uniform_location.view, 1, GL_FALSE, glm::value_ptr(view_matrix));
    glUniformMatrix4fv(_gl_state.uniform_location.proj, 1, GL_FALSE, glm::value_ptr(proj_matrix));

    for (int i = 0; i < _polylines.size(); i++) {
        Polyline polyline = _polylines[i];

        if (polyline.vao == 0) { continue; }

        GLenum primitive_type = polyline.style.primitive;
        glLineWidth(polyline.style.line_width);
        glPointSize(polyline.style.point_size);

        glBindVertexArray(polyline.vao);
        glUniform4fv(_gl_state.uniform_location.global_color, 1, glm::value_ptr(polyline.global_color));

        glDrawArrays(primitive_type, 0, polyline.num_vertices);

        if (primitive_type != PointLineRenderer::POINTS && polyline.style.render_points) {
            glDrawArrays(GL_POINTS, 0, polyline.num_vertices);
        }
    }

    if (old_line_smooth) {
        glEnable(GL_LINE_SMOOTH);
    } else {
        glDisable(GL_LINE_SMOOTH);
    }
    glLineWidth(old_line_width);
    glPointSize(old_point_size);
    glBindVertexArray(0);
    glUseProgram(0);
}
