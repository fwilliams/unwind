#ifndef __VOLUME_RENDERING_H__
#define __VOLUME_RENDERING_H__

#include <igl/opengl/load_shader.h>
#include <igl/opengl/create_shader_program.h>
#include <Eigen/Core>
#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace volumerendering {

struct TfNode {
    float t;
    glm::vec4 rgba;
};


struct Parameters {
    glm::ivec3 volume_dimensions = { 0, 0, 0 };

    glm::vec3 light_position;
    float highlight_factor;
    GLfloat sampling_rate = 10.0;

    glm::vec3 ambient = glm::vec3(0.5, 0.5, 0.5);
    glm::vec3 diffuse = glm::vec3(0.8, 0.8, 0.8);
    glm::vec3 specular = glm::vec3(0.0, 0.0, 0.0);
    float specular_exponent = 10.0;

    // When to emphasize selected components:
    // 0 = None
    // 1 = OnSelection
    // 2 = OnNonSelection
    int emphasize_by_selection = 1;

    // Color components based on their identifier
    bool color_by_id = true;
};

struct SelectionRenderer {
    struct GeometryPass {
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
    } bounding_box;
    //Parameters parameters;

    struct VolumePass {
        GLuint program_object = 0;
        GLuint transfer_function_texture;

        GLuint selection_list_ssbo;
        GLuint contour_information_ssbo;

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

            GLuint index_volume = 0;
            GLuint color_by_identifier = 0;
            GLuint selection_emphasis_type = 0;
            GLuint highlight_factor = 0;
        } uniform_location;
    } program;

    struct PickingPass {
        GLuint program_object = 0;

        GLuint picking_framebuffer = 0;
        GLuint picking_texture = 0;

        struct {
            GLint entry_texture = 0;
            GLint exit_texture = 0;
            GLint volume_texture = 0;
            GLint transfer_function = 0;

            GLint volume_dimensions = 0;
            GLint volume_dimensions_rcp = 0;
            GLint sampling_rate = 0;
            GLuint index_volume = 0;
        } uniform_location;
    } ___picking_pass;

    struct GLState {
        struct PickingPass {
            GLuint program_object = 0;

            GLuint picking_framebuffer = 0;
            GLuint picking_texture = 0;

            struct {
                GLint entry_texture = 0;
                GLint exit_texture = 0;
                GLint volume_texture = 0;
                GLint transfer_function = 0;

                GLint volume_dimensions = 0;
                GLint volume_dimensions_rcp = 0;
                GLint sampling_rate = 0;
                GLuint index_volume = 0;
            } uniform_location;
        } picking_pass;
    } _gl_state;

    // Buffer contents:
    // [0]: number of features
    // [...]: A linearized map from voxel identifier -> feature number
    void set_contour_data(uint32_t* contour_features, size_t num_features);
    void set_selection_data(uint32_t* selection_list, size_t num_features);
    void resize_framebuffer(glm::ivec2 framebuffer_size);
    void set_transfer_function(const std::vector<TfNode>& tf);

    void initialize(const glm::ivec2& viewport_size);
    void destroy();

    void geometry_pass(glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix);
    void volume_pass(Parameters parameters, GLuint index_texture, GLuint volume_texture);
    glm::vec3 picking_pass(Parameters parameters, glm::ivec2 mouse_position, GLuint index_texture, GLuint volume_texture);

};

} // namespace volumerendering

#endif
