#ifndef __VOLUME_RENDERING_H__
#define __VOLUME_RENDERING_H__

#include <igl/opengl/load_shader.h>
#include <igl/opengl/create_shader_program.h>
#include <Eigen/Core>
#include <array>
#include <vector>

namespace volumerendering {

struct Bounding_Box {
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
};

struct Transfer_Function {
    struct Node {
        float t;
        float rgba[4];
    };

    std::vector<Node> nodes;
    bool is_dirty = false;
    GLuint texture = 0;
};

struct Parameters {
    std::array<GLint, 3> volume_dimensions = { 0, 0, 0 };
    std::array<GLfloat, 3> volume_dimensions_rcp = { 0.f, 0.f, 0.f };

    std::array<float, 3> normalized_volume_dimensions = { 0.f, 0.f, 0.f };

    GLfloat sampling_rate = 10.0;
};

struct Volume_Rendering {
    GLuint volume_texture = 0;

    Bounding_Box bounding_box;
    Transfer_Function transfer_function;
    Parameters parameters;

    struct {
        GLuint program_object = 0;
        struct {
            GLint entry_texture = 0;
            GLint exit_texture = 0;
            GLint volume_texture = 0;
            GLint transfer_function = 0;

            GLint volume_dimensions = 0;
            GLint volume_dimensions_rcp = 0;
            GLint sampling_rate = 0;

            GLint light_position = 0;
            GLint light_color_ambient = 0;
            GLint light_color_diffuse = 0;
            GLint light_color_specular = 0;
            GLint light_exponent_specular = 0;
        } uniform_location;
    } program;

    GLuint picking_framebuffer = 0;
    GLuint picking_texture = 0;

    struct {
        GLuint program_object = 0;
        struct {
            GLint entry_texture = 0;
            GLint exit_texture = 0;
            GLint volume_texture = 0;
            GLint transfer_function = 0;

            GLint volume_dimensions = 0;
            GLint volume_dimensions_rcp = 0;
            GLint sampling_rate = 0;
        } uniform_location;
    } picking_program;
};


void initialize(Volume_Rendering& volume_rendering, Eigen::Vector4f viewport_size,
    const char* fragment_shader = nullptr, const char* picking_shader = nullptr);

void upload_volume_data(GLuint volume_texture, const Eigen::RowVector3i& tex_size,
    const Eigen::VectorXd& texture);

void update_transfer_function(Transfer_Function& transfer_function);

void render_bounding_box(const Volume_Rendering& volume_rendering,
    Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
    Eigen::Matrix4f proj_matrix);

void render_volume(const Volume_Rendering& volume_rendering, Eigen::Matrix4f model_matrix,
    Eigen::Matrix4f view_matrix, Eigen::Matrix4f proj_matrix,
    Eigen::Vector3f light_position);

Eigen::Vector3f pick_volume_location(const Volume_Rendering& volume_rendering,
    Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
    Eigen::Matrix4f proj_matrix, Eigen::Vector2i mouse_position);


} // namespace volumerendering

#endif
