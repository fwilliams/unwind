#ifndef VOLUME_RENDERING_2_H
#define VOLUME_RENDERING_2_H

#include <glad/glad.h>
#include <igl/opengl/create_shader_program.h>
#include <glm/glm.hpp>
#include <vector>
#include <fstream>


struct TfNode {
    float t;
    glm::vec4 rgba;
};

class VolumeRenderer {
    glm::ivec3 _volume_dimensions;
    GLfloat _sampling_rate = 10.0f;
    GLuint _num_bounding_indices = 0;

    int _current_multipass_buf = -1;
    GLuint _current_volume_tex = 0;
    glm::ivec3 _current_volume_dims;

    GLfloat _step_size = 0.0;

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

            GLuint vao;

            struct {
                GLint entry_texture = 0;
                GLint exit_texture = 0;
                GLint volume_texture = 0;
                GLint transfer_function = 0;
                GLint value_init_texture = 0;
                GLint final = 0;

                GLint volume_dimensions = 0;
                GLint volume_dimensions_rcp = 0;
                GLint sampling_rate = 0;

                GLint light_position = 0;
                GLint light_color_ambient = 0;
                GLint light_color_diffuse = 0;
                GLint light_color_specular = 0;
                GLint light_exponent_specular = 0;
            } uniform_location;
        } volume_pass;

        struct MultipassState {
            GLuint framebuffer[2];
            GLuint texture[2];
        } multipass;
    } _gl_state;

    void ray_endpoint_pass(const glm::mat4 &model_matrix, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix);
    void volume_pass(const glm::vec3& light_position, const glm::ivec3 &volume_dims, GLuint volume_tex, GLuint multipass_tex);

public:
    const GLState& gl_state() const { return _gl_state; }

    void set_step_size(float step_size) {
        _step_size = step_size;
    }

    void resize_framebuffer(const glm::ivec2& viewport_size);

    void init(const glm::ivec2& viewport_size,
              const char* fragment_shader = nullptr,
              const char* picking_shader = nullptr);

    void destroy();

    void set_transfer_function(const std::vector<TfNode>& transfer_function);

    void set_bounding_geometry(GLfloat* vertices, GLsizei num_vertices, GLint* indices, GLsizei num_faces);
    // TODO: Allow setting multiple geometric objects
    //    void set_bounding_geometry(const std::vector<GLfloat*>& vertices,
    //                               const std::vector<GLsizei>& num_vertices,
    //                               const std::vector<GLint*>& indices,
    //                               const std::vector<GLsizei>& num_indices);

    void begin(const glm::ivec3 &volume_dims, GLuint tex);
    void render_pass(const glm::mat4 &model_matrix,
                     const glm::mat4 &view_matrix,
                     const glm::mat4 &proj_matrix,
                     const glm::vec3& light_position,
                     bool final);
};


#endif // VOLUME_RENDERING_2_H
