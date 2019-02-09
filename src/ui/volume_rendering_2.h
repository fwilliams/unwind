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

class VolumeTexture {
    GLuint _tex;
    glm::ivec3 _volume_dimensions;

public:
    GLint texture() const { return _tex; }
    const glm::ivec3& volume_dimensions() { return _volume_dimensions; }

    void init() {
        glGenTextures(1, &_tex);
        glBindTexture(GL_TEXTURE_3D, _tex);

        GLfloat transparent_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, transparent_color);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glBindTexture(GL_TEXTURE_3D, 0);
    }

    void destroy() {
        glDeleteTextures(1, &_tex);
    }

    bool set_data(const glm::ivec3& voldims, GLenum format, GLenum internal_format, GLenum type, void* data) {
        if (_tex == 0) {
            return false;
        }

        _volume_dimensions = voldims;

        glBindTexture(GL_TEXTURE_3D, _tex);
        glTexImage3D(GL_TEXTURE_3D, 0, internal_format, voldims[0], voldims[1], voldims[2], 0, format, type, data);
        glBindTexture(GL_TEXTURE_3D, 0);

        return true;
    }

    bool resize(const glm::ivec3& voldims, GLenum format, GLenum internal_format, GLenum type) {
        return set_data(voldims, format, internal_format, type, nullptr);
    }

    bool write_to_file(std::string filename, GLenum format) {
        if (_tex == 0) {
            return false;
        }

        std::vector<float> out_data(_volume_dimensions[0]*_volume_dimensions[1]*_volume_dimensions[2]);
        glBindTexture(GL_TEXTURE_3D, _tex);
        glGetTexImage(GL_TEXTURE_3D, 0, format, GL_FLOAT, (void*)out_data.data());
        glBindTexture(GL_TEXTURE_3D, 0);

        std::ofstream fout;
        fout.open(filename, std::ios::binary);
        fout.write(reinterpret_cast<char*>(out_data.data()), sizeof(float)*out_data.size());
        fout.close();

        return true;
    }
};

class VolumeRenderer {
    glm::ivec3 _volume_dimensions;
    GLfloat _sampling_rate = 10.0f;
    GLuint _num_bounding_indices = 0;

    int _current_multipass_buf = -1;

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

public:
    const GLState& gl_state() const { return _gl_state; }
    const glm::ivec3& volume_dims() const { return _volume_dimensions; }

    void resize_framebuffer(const glm::ivec2& viewport_size);

    void init(const glm::ivec2& viewport_size,
              bool use_multipass,
              const char* fragment_shader = nullptr,
              const char* picking_shader = nullptr);

    void destroy();

    void set_volume_data(const glm::ivec3& volume_dims, const double* texture_data);
    void set_transfer_function(const std::vector<TfNode>& transfer_function);

    GLuint set_volume_texture(GLuint volume_tex, bool delete_previous=false);
    GLint volume_texture() const { return _gl_state.volume_pass.volume_texture; }

    void set_bounding_geometry(GLfloat* vertices, GLsizei num_vertices, GLint* indices, GLsizei num_indices);
    // TODO: Allow setting multiple geometric objects
    //    void set_bounding_geometry(const std::vector<GLfloat*>& vertices,
    //                               const std::vector<GLsizei>& num_vertices,
    //                               const std::vector<GLint*>& indices,
    //                               const std::vector<GLsizei>& num_indices);

    void render_bounding_box(const glm::mat4 &model_matrix, const glm::mat4 &view_matrix, const glm::mat4 &proj_matrix);
    void render_volume(const glm::vec3& light_position, GLuint multipass_tex=0);


    void begin_multipass();
    void render_multipass(const glm::mat4 &model_matrix,
                          const glm::mat4 &view_matrix,
                          const glm::mat4 &proj_matrix,
                          const glm::vec3& light_position,
                          bool final);
};


#endif // VOLUME_RENDERING_2_H
