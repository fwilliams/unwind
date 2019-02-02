#ifndef RENDERING_2D_H
#define RENDERING_2D_H

#include <vector>
#include <glad/glad.h>

#include <glm/glm.hpp>


class Renderer2d {
public:
    enum LineSpec {
        POINTS = GL_POINTS,
        LINES = GL_LINES,
        LINE_STRIP = GL_LINE_STRIP,
        LINE_LOOP = GL_LINE_LOOP
    };

    struct PolylineStyle {
        LineSpec primitive = LineSpec::POINTS;
        bool render_points = false;
        float line_width = 1.0;
        float point_size = 1.0;
    };

    struct Polyline {
        GLuint pos_vbo;
        GLuint clr_vbo;
        GLuint vao;

        glm::vec4 global_color;

        PolylineStyle style;
        size_t num_vertices;
    };

private:
    struct {
        GLuint program;

        struct {
            GLint model;
            GLint view;
            GLint proj;
            GLint global_color;
        } uniform_location;
    } _gl_state;

    bool _line_antialiasing_enabled = true;
    std::vector<Polyline> _polylines;
    std::vector<int> _free_list;

public:
    void init();

    void set_line_antialiasing(bool enabled) { _line_antialiasing_enabled = enabled; }

    int add_polyline_3d(GLfloat* vertices, GLfloat* colors, GLsizei num_vertices, PolylineStyle style);
    int add_polyline_3d(GLfloat* vertices, glm::vec4 color, GLsizei num_vertices, PolylineStyle style);
    bool update_polyline_3d(int polyline_id, GLfloat* vertices, GLfloat* colors, GLsizei num_vertices, PolylineStyle style);
    bool update_polyline_3d(int polyline_id, GLfloat* vertices, glm::vec4 color, GLsizei num_vertices, PolylineStyle style);

    bool delete_polyline(int polyline_id);

    void draw(int polyline_id, glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix);
    void draw(int* polyline_ids, std::size_t num_polylines, glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix);
    void draw(glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix);
};

#endif // RENDERING_2D_H
