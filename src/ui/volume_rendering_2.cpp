#include "volume_rendering_2.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <iostream>

namespace vr {

// Vertex shader that is used to trigger the volume rendering by rendering a static
// screen-space filling quad.
constexpr const char* VolumeRenderingVertexShader = R"(
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

  out vec2 uv;

  void main() {
    // Clipspace \in [-1, 1]
    gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);

    // uv coordinates \in [0, 1]
    uv = (positions[gl_VertexID] + 1.0) / 2.0;
  }
)";

// Shader that performs the actual volume rendering
// Steps:
// 1. Compute the ray direction by exit point color - entry point color
// 2. Sample the volume along the ray
// 3. Convert sample to color using the transfer function
// 4. Compute central difference gradient
// 5. Use the gradient for Phong shading
// 6. Perform front-to-back compositing
// 7. Stop if either the ray is exhausted or the combined transparency is above an
//    early-ray termination threshold (0.99 in this case)
// Shader that performs the actual volume rendering
// Steps:
// 1. Compute the ray direction by exit point color - entry point color
// 2. Sample the volume along the ray
// 3. Convert sample to color using the transfer function
// 4. Compute central difference gradient
// 5. Use the gradient for Phong shading
// 6. Perform front-to-back compositing
// 7. Stop if either the ray is exhausted or the combined transparency is above an
//    early-ray termination threshold (0.99 in this case)
constexpr const char* VolumeRenderingFragmentShader = R"(
#version 150
  in vec2 uv;
  out vec4 out_color;

  uniform sampler2D entry_texture;
  uniform sampler2D exit_texture;

  uniform sampler3D volume_texture;
  uniform sampler1D transfer_function;

  uniform ivec3 volume_dimensions;
  uniform vec3 volume_dimensions_rcp;
  uniform float sampling_rate;

  struct Light_Parameters {
    vec3 position;
    vec3 ambient_color;
    vec3 diffuse_color;
    vec3 specular_color;
    float specular_exponent;
  };
  uniform Light_Parameters light_parameters;


  // Early-ray termination
  const float ERT_THRESHOLD = 0.99;
  const float REF_SAMPLING_INTERVAL = 150.0;

  vec3 centralDifferenceGradient(vec3 pos) {
    vec3 f;
    f.x = texture(volume_texture, pos + vec3(volume_dimensions_rcp.x, 0.0, 0.0)).a;
    f.y = texture(volume_texture, pos + vec3(0.0, volume_dimensions_rcp.y, 0.0)).a;
    f.z = texture(volume_texture, pos + vec3(0.0, 0.0, volume_dimensions_rcp.z)).a;

    vec3 b;
    b.x = texture(volume_texture, pos - vec3(volume_dimensions_rcp.x, 0.0, 0.0)).a;
    b.y = texture(volume_texture, pos - vec3(0.0, volume_dimensions_rcp.y, 0.0)).a;
    b.z = texture(volume_texture, pos - vec3(0.0, 0.0, volume_dimensions_rcp.z)).a;

    return (f - b) / 2.0;
  }

  vec3 blinn_phong(Light_Parameters light, vec3 material_ambient_color,
                   vec3 material_diffuse_color, vec3 material_specular_color,
                   vec3 position, vec3 normal, vec3 direction_to_camera)
  {
    vec3 direction_to_light = normalize(light.position - position);
    vec3 ambient = material_ambient_color * light.ambient_color;
    vec3 diffuse = material_diffuse_color * light.diffuse_color *
                   max(dot(normal, direction_to_light), 0.0);
    vec3 specular;
    {
      vec3 half_way_vector = normalize(direction_to_camera + direction_to_light);
      specular = material_specular_color * light.specular_color *
                 pow(max(dot(normal, half_way_vector), 0.0), light.specular_exponent);
    }

    return ambient + diffuse + specular;
  }

  void main() {
    vec3 entry = texture(entry_texture, uv).rgb;
    vec3 exit = texture(exit_texture, uv).rgb;
    if (entry == exit) {
      out_color = vec4(0.0, 0.0, 0.0, 0.0);
      return;
    }

    // Combined final color that the volume rendering computed
    vec4 result = vec4(0.0);

    vec3 ray_direction = exit - entry;

    float t_end = length(ray_direction);
    float t_incr = min(
      t_end,
      t_end / (sampling_rate * length(ray_direction * volume_dimensions))
    );
    t_incr = 0.01;

    vec3 normalized_ray_direction = normalize(ray_direction);

    float t = 0.0;
    while (t < t_end) {
      vec3 sample_pos = entry + t * normalized_ray_direction;
      float value = texture(volume_texture, sample_pos).r;
      vec4 color = texture(transfer_function, value);
      if (color.a > 0) {
        // Gradient
        vec3 gradient = centralDifferenceGradient(sample_pos);

        // Lighting
        //color.rgb = blinn_phong(light_parameters, color.rgb, color.rgb, vec3(1.0),
        //                        sample_pos, gradient, -normalized_ray_direction);

        // Front-to-back Compositing
        color.a = 1.0 - pow(1.0 - color.a, t_incr * REF_SAMPLING_INTERVAL);
        result.rgb = result.rgb + (1.0 - result.a) * color.a * color.rgb;
        result.a = result.a + (1.0 - result.a) * color.a;
      }

      if (result.a > ERT_THRESHOLD) {
        t = t_end;
      }
      else {
        t += t_incr;
      }
    }

    //result.a = 1.0;
    out_color = result;
  }
)";

// Shader transforming the vertices from model coordinates to clip space
constexpr const char* EntryBoxVertexShader = R"(
    #version 150
    in vec3 in_position;
    out vec3 color;

    uniform mat4 model_matrix;
    uniform mat4 view_matrix;
    uniform mat4 projection_matrix;

    void main() {
        gl_Position = projection_matrix * view_matrix * model_matrix * vec4(in_position, 1.0);
        color = in_position.xyz;
  }
)";

// Using Krueger-Westermann rendering encodes the position of the vertex as its color
constexpr const char* EntryBoxFragmentShader = R"(
#version 150
  in vec3 color;
  out vec4 out_color;

  void main() {
    out_color = vec4(color, 1.0);
  }
)";

void VolumeRenderer::set_transfer_function(const std::vector<TfNode> &transfer_function) {
    /* The input transfer_function is a sequence of nodes on a graph as shown below.
     * Each 'o' corresponds to a TfNode
     *    a
     *    l  ^
     *    p  |                     a[4]
     *    h  |                      o
     *    a  |                     / \
     *       |   a[1]     a[2]    /   \
     *       |     o-------o     /     \
     *       |    /    ^    \   /       \
     *       |   /     |     \ /         \
     *       |  /      |      o           \
     *       | /       |     a[3]          \
     *       |/        |                    \
     *       o---------|---------------------o---------------o>     t \in [0, 1]
     *       ^         |                    a[5]             ^
     *     first       |                                    last
     *      a[0]       |                                    a[6]
     *
     *        local t' renormalized \in [0 = a[1], 1 = a[2]]
     *        linear interpolation using t' by (1 - t') * a[1] + t' * a[2]
     */

    // Generate the transfer function texture
    constexpr const int TRANSFER_FUNCTION_WIDTH = 512;
    std::vector<TfNode>::const_iterator current = transfer_function.begin();
    std::vector<TfNode>::const_iterator next = current + 1;

    std::vector<std::array<uint8_t, 4>> transfer_function_data(TRANSFER_FUNCTION_WIDTH);
    for (int i = 0; i < TRANSFER_FUNCTION_WIDTH; ++i) {
        const float t = static_cast<float>(i) / (TRANSFER_FUNCTION_WIDTH - 1);

        if (t > next->t) {
            current = next;
            next++;
        }

        const float t_prime = (t - current->t) / (next->t - current->t);

        const float rgba[] = {
            (1 - t_prime) * current->rgba[0] + t_prime * next->rgba[0],
            (1 - t_prime) * current->rgba[1] + t_prime * next->rgba[1],
            (1 - t_prime) * current->rgba[2] + t_prime * next->rgba[2],
            (1 - t_prime) * current->rgba[3] + t_prime * next->rgba[3]
        };

        transfer_function_data[i] = {
            static_cast<uint8_t>(rgba[0] * 255.f),
            static_cast<uint8_t>(rgba[1] * 255.f),
            static_cast<uint8_t>(rgba[2] * 255.f),
            static_cast<uint8_t>(rgba[3] * 255.f)
        };
    }
    glBindTexture(GL_TEXTURE_1D, _gl_state.volume_pass.transfer_function_texture);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, TRANSFER_FUNCTION_WIDTH, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, transfer_function_data.data());
    glBindTexture(GL_TEXTURE_1D, 0);
}

void VolumeRenderer::resize_framebuffer(const glm::ivec2& viewport_size) {
    // Entry point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, viewport_size.x, viewport_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);

    // Exit point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, viewport_size.x, viewport_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void VolumeRenderer::set_bounding_geometry(GLfloat* vertices, GLsizei num_vertices, GLint* indices, GLsizei num_faces) {
    _num_bounding_indices = num_faces*3;

    glBindBuffer(GL_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*num_vertices*3, vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint)*num_faces*3, indices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void VolumeRenderer::init(const glm::ivec2 &viewport_size, const char *fragment_shader, const char *picking_shader) {
    constexpr GLsizei NUM_VERTICES = 8;
    constexpr GLsizei NUM_FACES = 12;
    std::array<GLfloat, NUM_VERTICES*3> vertex_data = {
        0.f, 0.f, 0.f,
        0.f, 0.f, 1.f,
        0.f, 1.f, 0.f,
        0.f, 1.f, 1.f,
        1.f, 0.f, 0.f,
        1.f, 0.f, 1.f,
        1.f, 1.f, 0.f,
        1.f, 1.f, 1.f
    };
    std::array<GLuint, NUM_FACES*3> index_data = {
        0, 6, 4,
        0, 2, 6,
        0, 3, 2,
        0, 1, 3,
        2, 7, 6,
        2, 3, 7,
        4, 6, 7,
        4, 7, 5,
        0, 4, 5,
        0, 5, 1,
        1, 5, 7,
        1, 7, 3
    };
    _num_bounding_indices = NUM_FACES*3;

    glGenVertexArrays(1, &_gl_state.ray_endpoints_pass.vao);
    glBindVertexArray(_gl_state.ray_endpoints_pass.vao);

    glGenBuffers(1, &_gl_state.ray_endpoints_pass.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*NUM_VERTICES*3, vertex_data.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat)*3, nullptr);
    glEnableVertexAttribArray(0);

    // Creating the index buffer object
    glGenBuffers(1, &_gl_state.ray_endpoints_pass.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint)*NUM_FACES*3, index_data.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // Shader to render the bounding box entry and exit points
    igl::opengl::create_shader_program(EntryBoxVertexShader,
                                       EntryBoxFragmentShader,
                                       {{ "in_position", 0 }},
                                       _gl_state.ray_endpoints_pass.program);
    _gl_state.ray_endpoints_pass.uniform_location.model_matrix = glGetUniformLocation(
        _gl_state.ray_endpoints_pass.program, "model_matrix");
    _gl_state.ray_endpoints_pass.uniform_location.view_matrix = glGetUniformLocation(
        _gl_state.ray_endpoints_pass.program, "view_matrix");
    _gl_state.ray_endpoints_pass.uniform_location.projection_matrix = glGetUniformLocation(
        _gl_state.ray_endpoints_pass.program, "projection_matrix");





    // Shader to render the actual volume
    igl::opengl::create_shader_program(VolumeRenderingVertexShader,
                                       VolumeRenderingFragmentShader, {},
                                       _gl_state.volume_pass.program);
    _gl_state.volume_pass.uniform_location.entry_texture = glGetUniformLocation(
        _gl_state.volume_pass.program, "entry_texture");
    _gl_state.volume_pass.uniform_location.exit_texture = glGetUniformLocation(
        _gl_state.volume_pass.program, "exit_texture");
    _gl_state.volume_pass.uniform_location.volume_texture = glGetUniformLocation(
        _gl_state.volume_pass.program, "volume_texture");
    _gl_state.volume_pass.uniform_location.volume_dimensions = glGetUniformLocation(
        _gl_state.volume_pass.program, "volume_dimensions");
    _gl_state.volume_pass.uniform_location.volume_dimensions_rcp = glGetUniformLocation(
        _gl_state.volume_pass.program, "volume_dimensions_rcp");
    _gl_state.volume_pass.uniform_location.transfer_function = glGetUniformLocation(
        _gl_state.volume_pass.program, "transfer_function");
    _gl_state.volume_pass.uniform_location.sampling_rate = glGetUniformLocation(
        _gl_state.volume_pass.program, "sampling_rate");
    _gl_state.volume_pass.uniform_location.light_position = glGetUniformLocation(
        _gl_state.volume_pass.program, "light_parameters.position");
    _gl_state.volume_pass.uniform_location.light_color_ambient = glGetUniformLocation(
        _gl_state.volume_pass.program, "light_parameters.ambient_color");
    _gl_state.volume_pass.uniform_location.light_color_diffuse = glGetUniformLocation(
        _gl_state.volume_pass.program, "light_parameters.diffuse_color");
    _gl_state.volume_pass.uniform_location.light_color_specular = glGetUniformLocation(
        _gl_state.volume_pass.program, "light_parameters.specular_color");
    _gl_state.volume_pass.uniform_location.light_exponent_specular = glGetUniformLocation(
        _gl_state.volume_pass.program, "light_parameters.specular_exponent");


    // Entry point texture and frame buffer
    glGenTextures(1, &_gl_state.ray_endpoints_pass.entry_texture);
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
        GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &_gl_state.ray_endpoints_pass.entry_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.ray_endpoints_pass.entry_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        _gl_state.ray_endpoints_pass.entry_texture, 0);


    // Exit point texture and frame buffer
    glGenTextures(1, &_gl_state.ray_endpoints_pass.exit_texture);
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
        GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &_gl_state.ray_endpoints_pass.exit_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.ray_endpoints_pass.exit_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        _gl_state.ray_endpoints_pass.exit_texture, 0);


    // Volume texture
    glGenTextures(1, &_gl_state.volume_pass.volume_texture);
    glBindTexture(GL_TEXTURE_3D, _gl_state.volume_pass.volume_texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Picking texture and framebuffer
//    glGenTextures(1, &volume_rendering.picking_texture);
//    glBindTexture(GL_TEXTURE_2D, volume_rendering.picking_texture);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
//        GL_FLOAT, nullptr);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//    glGenFramebuffers(1, &volume_rendering.picking_framebuffer);
//    glBindFramebuffer(GL_FRAMEBUFFER, volume_rendering.picking_framebuffer);
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
//        volume_rendering.picking_texture, 0);
//    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // Initialize transfer function
    // Texture
    glGenTextures(1, &_gl_state.volume_pass.transfer_function_texture);
    glBindTexture(GL_TEXTURE_1D, _gl_state.volume_pass.transfer_function_texture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);

    // Generate a reasonable default transfer function
    TfNode first = { 0.f, { 0.f, 0.f, 0.f, 0.f } };
    TfNode last = { 1.f, { 1.f, 1.f, 1.f, 1.f } };
    std::vector<TfNode> initial_transfer_function;
    initial_transfer_function.push_back(std::move(first));
    initial_transfer_function.push_back(std::move(last));
    set_transfer_function(initial_transfer_function);
}

void VolumeRenderer::set_volume_data(const glm::ivec3 &volume_dims, const double *texture_data) {
    _volume_dimensions = volume_dims;

    std::size_t num_elements = volume_dims[0]*volume_dims[1]*volume_dims[2];
    std::vector<std::uint8_t> volume_data(num_elements);
    std::transform(
        texture_data,
        texture_data + num_elements,
        volume_data.begin(),
        [](double d) { return static_cast<uint8_t>(d * std::numeric_limits<uint8_t>::max()); }
    );
    glBindTexture(GL_TEXTURE_3D, _gl_state.volume_pass.volume_texture);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, volume_dims[0], volume_dims[1], volume_dims[2], 0,
                 GL_RED, GL_UNSIGNED_BYTE, reinterpret_cast<void*>(volume_data.data()));
    glBindTexture(GL_TEXTURE_3D, 0);

}

void VolumeRenderer::render_bounding_box(const glm::mat4& model_matrix, const glm::mat4& view_matrix, const glm::mat4& proj_matrix) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render Bounding Box TEST");
    {
        GLint old_viewport[4];
        glGetIntegerv(GL_VIEWPORT, old_viewport);
        GLint fb_tex_w, fb_tex_h;
        glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.exit_texture);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &fb_tex_w);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &fb_tex_h);
        glBindTexture(GL_TEXTURE_2D, 0);

        glViewport(0, 0, fb_tex_w, fb_tex_h);

        glBindVertexArray(_gl_state.ray_endpoints_pass.vao);
        glBindBuffer(GL_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _gl_state.ray_endpoints_pass.ibo);

        glUseProgram(_gl_state.ray_endpoints_pass.program);

        glm::vec3 normalized_volume_dimensions = glm::vec3(_volume_dimensions) / static_cast<float>(glm::compMax(_volume_dimensions));
        glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
        glm::mat4 scaling = glm::scale(glm::mat4(1.f), normalized_volume_dimensions);

        glUniformMatrix4fv(_gl_state.ray_endpoints_pass.uniform_location.model_matrix, 1,
            GL_FALSE, glm::value_ptr(model_matrix * scaling * translate));

        glUniformMatrix4fv(_gl_state.ray_endpoints_pass.uniform_location.view_matrix, 1,
            GL_FALSE, glm::value_ptr(view_matrix));

        glUniformMatrix4fv(_gl_state.ray_endpoints_pass.uniform_location.projection_matrix,
            1, GL_FALSE, glm::value_ptr(proj_matrix));

        // Render entry points of bounding box
        glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.ray_endpoints_pass.entry_framebuffer);
        glClear(GL_COLOR_BUFFER_BIT);
        glCullFace(GL_FRONT);
        glDrawElements(GL_TRIANGLES, _num_bounding_indices, GL_UNSIGNED_INT, nullptr);

        // Render exit points of bounding box
        glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.ray_endpoints_pass.exit_framebuffer);
        glClear(GL_COLOR_BUFFER_BIT);
        glCullFace(GL_BACK);
        glDrawElements(GL_TRIANGLES, _num_bounding_indices * 3, GL_UNSIGNED_INT, nullptr);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
    }
    glPopDebugGroup();
}

void VolumeRenderer::render_volume(const glm::vec3& light_position) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render Volume TEST");

    //
    //  Setup
    //
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    glUseProgram(_gl_state.volume_pass.program);

    // Bind the entry points texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.entry_texture);
    glUniform1i(_gl_state.volume_pass.uniform_location.entry_texture, 0);

    // Bind the exit points texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _gl_state.ray_endpoints_pass.exit_texture);
    glUniform1i(_gl_state.volume_pass.uniform_location.entry_texture, 1);

    // Bind the volume texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, _gl_state.volume_pass.volume_texture);
    glUniform1i(_gl_state.volume_pass.uniform_location.volume_texture, 2);

    // Bind the transfer function texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_1D, _gl_state.volume_pass.transfer_function_texture);
    glUniform1i(_gl_state.volume_pass.uniform_location.transfer_function, 3);

    // Bind rendering parameters
    glm::vec3 volume_dims_rcp = glm::vec3(1.0) / glm::vec3(_volume_dimensions);
    glUniform1f(_gl_state.volume_pass.uniform_location.sampling_rate, _sampling_rate);
    glUniform3iv(_gl_state.volume_pass.uniform_location.volume_dimensions, 1, glm::value_ptr(_volume_dimensions));
    glUniform3fv(_gl_state.volume_pass.uniform_location.volume_dimensions_rcp, 1, glm::value_ptr(volume_dims_rcp));
    glUniform3fv(_gl_state.volume_pass.uniform_location.light_position, 1, glm::value_ptr(light_position));
    glUniform3f(_gl_state.volume_pass.uniform_location.light_color_ambient, 0.5f, 0.5f, 0.5f);
    glUniform3f(_gl_state.volume_pass.uniform_location.light_color_diffuse, 0.8f, 0.8f, 0.8f);
    glUniform3f(_gl_state.volume_pass.uniform_location.light_color_specular, 1.f, 1.f, 1.f);
    glUniform1f(_gl_state.volume_pass.uniform_location.light_exponent_specular, 10.f);

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glPopDebugGroup();
}

}
