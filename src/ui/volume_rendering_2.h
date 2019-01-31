#ifndef VOLUME_RENDERING_2_H
#define VOLUME_RENDERING_2_H

#include <glad/glad.h>
#include <igl/opengl/create_shader_program.h>
#include <glm/glm.hpp>
#include <vector>

namespace vr {

struct TfNode {
    float t;
    glm::vec4 rgba;
};

class VolumeRenderer {
    glm::ivec3 _volume_dimensions;
    GLfloat _sampling_rate = 10.0f;
    GLuint _num_bounding_indices = 0;

    struct GLState {
        struct RayEndpointsPass {
            GLuint vao = 0;
            GLuint vbo = 0;
            GLuint ibo = 0;

            GLuint entry_framebuffer = 0;
            GLuint entry_texture = 0;

            GLuint exit_framebuffer = 0;
            GLuint exit_texture = 0;

            GLuint program = 0;
            struct {
                GLint model_matrix = 0;
                GLint view_matrix = 0;
                GLint projection_matrix = 0;
            } uniform_location;
        } ray_endpoints_pass;

        struct VolumePass {
            GLuint program = 0;
            GLuint transfer_function_texture;
            GLuint volume_texture = 0;
            GLuint value_init_texture = 0;

            GLuint vao;

            struct {
                GLint entry_texture = 0;
                GLint exit_texture = 0;
                GLint volume_texture = 0;
                GLint transfer_function = 0;
                GLint value_init_texture = 0;

                GLint volume_dimensions = 0;
                GLint volume_dimensions_rcp = 0;
                GLint sampling_rate = 0;

                GLint light_position = 0;
                GLint light_color_ambient = 0;
                GLint light_color_diffuse = 0;
                GLint light_color_specular = 0;
                GLint light_exponent_specular = 0;

                GLint hack = 0;
            } uniform_location;
        } volume_pass;

    } _gl_state;



public:
    const GLState& gl_state() const { return _gl_state; }
    const glm::ivec3& volume_dims() const { return _volume_dimensions; }

    void resize_framebuffer(const glm::ivec2& viewport_size);

    void init(const glm::ivec2& viewport_size,
              const char* fragment_shader = nullptr,
              const char* picking_shader = nullptr);

    void set_volume_data(const glm::ivec3& volume_dims, const double* texture_data);
    void set_transfer_function(const std::vector<TfNode>& transfer_function);
    void render_bounding_box(const glm::mat4 &model_matrix, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix);
    void render_volume(const glm::vec3& light_position, GLuint multipass_tex=0, GLfloat hack = 0);

    void set_bounding_geometry(GLfloat* vertices, GLsizei num_vertices, GLint* indices, GLsizei num_indices);
};

}
#endif // VOLUME_RENDERING_2_H
