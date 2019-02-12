#include "volume_rendering.h"

#include <fstream>
#include <iostream>
#include <vector>

#include <igl/opengl/load_shader.h>
#include <igl/opengl/create_shader_program.h>

#include <glm/gtc/matrix_transform.hpp>

#include "utils/utils.h"



// Shader transforming the vertices from model coordinates to clip space
constexpr const char* VertexShader = R"(
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
constexpr const char* RAY_ENDPOINT_PASS_FRAGMENT_SHADER = R"(
#version 150
  in vec3 color;
  out vec4 out_color;

  void main() {
    out_color = vec4(color, 1.0);
  }
)";


// Vertex shader that is used to trigger the volume rendering by rendering a static
// screen-space filling quad.
constexpr const char* VOLUME_PASS_VERTEX_SHADER = R"(
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

    // uv coordinate s\in [0, 1]
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
constexpr const char* VOLUME_PASS_FRAGMENT_SHADER = R"(
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
      out_color = vec4(0.0, 0.0, 0.0, 1.0);
      return;
    }

    // Combined final color that the volume rendering computed
    vec4 result = vec4(0.0);
    
    vec3 ray_direction = exit - entry;

    float t_end = length(ray_direction);
    float t_incr = sampling_rate;
    /*
    min(
      t_end,
      t_end / (sampling_rate * length(ray_direction * volume_dimensions))
    );
    */

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
                                //sample_pos, gradient, -normalized_ray_direction);

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
    
    result.a = 1.0;
    out_color = result;
  }
)";

// Shader that performs a light-weight volume rendering and stores the position of the
// first hit point
// Steps:
// 1. Compute the ray direction by exit point color - entry point color
// 2. Sample the volume along the ray
// 3. Convert sample to color using the transfer function
// 4. Exit the first time the alpha value is > 0 and write the current position as
//    color
constexpr const char* VolumeRenderingPickingFragmentShader = R"(
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

  void main() {
    vec3 entry = texture(entry_texture, uv).rgb;
    vec3 exit = texture(exit_texture, uv).rgb;
    if (entry == exit) {
      out_color = vec4(0.0, 0.0, 0.0, 1.0);
      return;
    }

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
        out_color = vec4(sample_pos, 1.0);
        return;
      }      
      t += t_incr;
    }

    out_color = vec4(0.0, 0.0, 0.0, 1.0);
  }
)";


using namespace igl::opengl;

namespace volumerendering {

void SelectionRenderer::initialize(const glm::ivec2& viewport_size,
                const char* fragment_shader, const char* picking_shader)
{
    //
    //   Bounding box information
    //
    glGenVertexArrays(1, &bounding_box.vao);
    glBindVertexArray(bounding_box.vao);

    // Creating the vertex buffer object
    glGenBuffers(1, &bounding_box.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, bounding_box.vbo);

    // Unit cube centered around 0.5 \in [0,1]
    const GLfloat vertexData[] = {
        0.f, 0.f, 0.f,
        0.f, 0.f, 1.f,
        0.f, 1.f, 0.f,
        0.f, 1.f, 1.f,
        1.f, 0.f, 0.f,
        1.f, 0.f, 1.f,
        1.f, 1.f, 0.f,
        1.f, 1.f, 1.f
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * sizeof(GLfloat), nullptr);
    glEnableVertexAttribArray(0);


    // Creating the index buffer object
    glGenBuffers(1, &bounding_box.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bounding_box.ibo);

    // Specifying the 12 faces of the unit cube
    const GLubyte iboData[] = {
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
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(iboData), iboData, GL_STATIC_DRAW);

    glBindVertexArray(0);

    bounding_box.program = igl::opengl::create_shader_program(
        VertexShader,
        RAY_ENDPOINT_PASS_FRAGMENT_SHADER,
        { { "in_position", 0 } }
    );

    bounding_box.uniform_location.model_matrix = glGetUniformLocation(
        bounding_box.program, "model_matrix");
    bounding_box.uniform_location.view_matrix = glGetUniformLocation(
        bounding_box.program, "view_matrix");
    bounding_box.uniform_location.projection_matrix =
        glGetUniformLocation(bounding_box.program, "projection_matrix");


    // If the user specified a fragment shader, use that, otherwise, use the default one
    igl::opengl::create_shader_program(VOLUME_PASS_VERTEX_SHADER,
        fragment_shader ? fragment_shader : VOLUME_PASS_FRAGMENT_SHADER, {},
        program.program_object);

    program.uniform_location.entry_texture = glGetUniformLocation(
        program.program_object, "entry_texture");
    program.uniform_location.exit_texture = glGetUniformLocation(
        program.program_object, "exit_texture");
    program.uniform_location.volume_texture = glGetUniformLocation(
        program.program_object, "volume_texture");
    program.uniform_location.volume_dimensions = glGetUniformLocation(
        program.program_object, "volume_dimensions");
    program.uniform_location.volume_dimensions_rcp =
        glGetUniformLocation(
            program.program_object, "volume_dimensions_rcp"
        );
    program.uniform_location.transfer_function = glGetUniformLocation(
        program.program_object, "transfer_function");
    program.uniform_location.sampling_rate = glGetUniformLocation(
        program.program_object, "sampling_rate");
    program.uniform_location.light_position = glGetUniformLocation(
        program.program_object, "light_parameters.position");
    program.uniform_location.light_color_ambient = glGetUniformLocation(
        program.program_object, "light_parameters.ambient_color");
    program.uniform_location.light_color_diffuse = glGetUniformLocation(
        program.program_object, "light_parameters.diffuse_color");
    program.uniform_location.light_color_specular = glGetUniformLocation(
        program.program_object, "light_parameters.specular_color");
    program.uniform_location.light_exponent_specular =
        glGetUniformLocation(
            program.program_object, "light_parameters.specular_exponent"
        );

    igl::opengl::create_shader_program(VOLUME_PASS_VERTEX_SHADER,
        picking_shader ? picking_shader : VolumeRenderingPickingFragmentShader, {},
        picking_program.program_object);

    picking_program.uniform_location.entry_texture =
        glGetUniformLocation(
            picking_program.program_object, "entry_texture"
        );
    picking_program.uniform_location.exit_texture =
        glGetUniformLocation(
            picking_program.program_object, "exit_texture"
        );
    picking_program.uniform_location.volume_texture =
        glGetUniformLocation(
            picking_program.program_object, "volume_texture"
        );
    picking_program.uniform_location.volume_dimensions =
        glGetUniformLocation(
            picking_program.program_object, "volume_dimensions"
        );
    picking_program.uniform_location.volume_dimensions_rcp =
        glGetUniformLocation(
            picking_program.program_object, "volume_dimensions_rcp"
        );
    picking_program.uniform_location.transfer_function =
        glGetUniformLocation(
            picking_program.program_object, "transfer_function"
        );
    picking_program.uniform_location.sampling_rate =
        glGetUniformLocation(
            picking_program.program_object, "sampling_rate"
        );


    // Entry point texture and frame buffer
    glGenTextures(1, &bounding_box.entry_texture);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
        GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &bounding_box.entry_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.entry_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        bounding_box.entry_texture, 0);


    // Exit point texture and frame buffer
    glGenTextures(1, &bounding_box.exit_texture);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
        GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &bounding_box.exit_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.exit_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        bounding_box.exit_texture, 0);


    // Picking texture and framebuffer
    glGenTextures(1, &picking_texture);
    glBindTexture(GL_TEXTURE_2D, picking_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB,
        GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &picking_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, picking_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        picking_texture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // Initialize transfer function
    // Texture
    glGenTextures(1, &transfer_function.texture);
    glBindTexture(GL_TEXTURE_1D, transfer_function.texture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);

    // Create the initial nodes
    Transfer_Function::Node first = { 0.f, { 0.f, 0.f, 0.f, 0.f } };
    Transfer_Function::Node last = { 1.f, { 1.f, 1.f, 1.f, 1.f } };
    transfer_function.nodes.push_back(std::move(first));
    transfer_function.nodes.push_back(std::move(last));
    transfer_function.is_dirty = true;
}


void SelectionRenderer::destroy() {
    std::vector<GLuint> buffers = {
        bounding_box.vbo,
        bounding_box.ibo,
        _gl_state.contour_information_ssbo,
        _gl_state.selection_list_ssbo };
    std::vector<GLuint> textures = {
        bounding_box.entry_texture,
        bounding_box.exit_texture,
        transfer_function.texture,
        picking_texture,
    };
    std::vector<GLuint> framebuffers = {
        bounding_box.entry_framebuffer,
        bounding_box.exit_texture,
        picking_framebuffer
    };

    glDeleteBuffers(buffers.size(), buffers.data());
    glDeleteTextures(textures.size(), textures.data());
    glDeleteFramebuffers(framebuffers.size(), framebuffers.data());
    glDeleteProgram(program.program_object);
    glDeleteProgram(picking_program.program_object);
    glDeleteProgram(bounding_box.program);

    picking_framebuffer = 0;
    picking_texture = 0;
    bounding_box = Bounding_Box();
    parameters = Parameters();
    program = VolumeProgram();
    picking_program = PickingProgram();
    _gl_state = GLState();
}

void update_transfer_function(Transfer_Function& transfer_function) {
    constexpr const int TransferFunctionWidth = 512;

    assert(
        std::is_sorted(
            transfer_function.nodes.begin(),
            transfer_function.nodes.end(),
            [](const Transfer_Function::Node& a, const Transfer_Function::Node& b) {
                return a.t < b.t;
            }
        )
    );
    assert(transfer_function.nodes.size() >= 2);
    assert(transfer_function.nodes.front().t == 0.f);
    assert(transfer_function.nodes.back().t == 1.f);


    std::vector<Transfer_Function::Node>::const_iterator current =
        transfer_function.nodes.begin();
    std::vector<Transfer_Function::Node>::const_iterator next = current + 1;

    //    a
    //    l  ^
    //    p  |                     a[4]
    //    h  |                      o
    //    a  |                     / \
    //       |   a[1]     a[2]    /   \
    //       |     o-------o     /     \
    //       |    /    ^    \   /       \
    //       |   /     |     \ /         \
    //       |  /      |      o           \
    //       | /       |     a[3]          \
    //       |/        |                    \
    //       o---------|---------------------o---------------o>     t \in [0, 1]
    //       ^         |                    a[5]             ^
    //     first       |                                    last
    //      a[0]       |                                    a[6]
    //
    //        local t' renormalized \in [0 = a[1], 1 = a[2]]
    //        linear interpolation using t' by (1 - t') * a[1] + t' * a[2]
    //


    std::vector<std::array<uint8_t, 4>> transfer_function_data(TransferFunctionWidth);
    for (int i = 0; i < TransferFunctionWidth; ++i) {
        const float t = static_cast<float>(i) / (TransferFunctionWidth - 1);

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

    glBindTexture(GL_TEXTURE_1D, transfer_function.texture);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, TransferFunctionWidth, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, transfer_function_data.data());
    glBindTexture(GL_TEXTURE_1D, 0);
}

void SelectionRenderer::render_bounding_box(glm::mat4 model_matrix, glm::mat4 view_matrix, glm::mat4 proj_matrix)
{
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render Bounding Box");


    const glm::vec4 color_transparent(0.0);

    // Back up face culling state so we can restore it when we're done
    GLboolean face_culling_enabled = glIsEnabled(GL_CULL_FACE);

    // Backup the previous viewport so we can restore it when we're done
    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    // Get the width and height of the entry and exit point textures so we can set the viewport correctly
    GLint fb_tex_w, fb_tex_h;
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &fb_tex_w);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &fb_tex_h);
    glBindTexture(GL_TEXTURE_2D, 0);



    // We need face culling to render
    glEnable(GL_CULL_FACE);

    // Set the viewport to match the entry and exit framebuffer textures
    glViewport(0, 0, fb_tex_w, fb_tex_h);

    // Bind the bounding geometry
    glBindVertexArray(bounding_box.vao);

    glUseProgram(bounding_box.program);

    // FIXME: Model matrix hack
    glm::mat4 scaling = glm::scale(glm::mat4(1.f), parameters.normalized_volume_dimensions);
    glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
    glm::mat4 model = model_matrix * scaling * translate;

    glUniformMatrix4fv(bounding_box.uniform_location.model_matrix, 1,
        GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(bounding_box.uniform_location.view_matrix, 1,
        GL_FALSE, glm::value_ptr(view_matrix));
    glUniformMatrix4fv(bounding_box.uniform_location.projection_matrix,
        1, GL_FALSE, glm::value_ptr(proj_matrix));

    // Render entry points of bounding box
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.entry_framebuffer);
    glClearBufferfv(GL_COLOR, 0, glm::value_ptr(color_transparent));
    glCullFace(GL_FRONT);
    glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_BYTE, nullptr);

    // Render exit points of bounding box
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.exit_framebuffer);
    glClearBufferfv(GL_COLOR, 0, glm::value_ptr(color_transparent));
    glCullFace(GL_BACK);
    glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_BYTE, nullptr);

    // Restore OpenGL state
    glBindVertexArray(0);
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
    if (face_culling_enabled == GL_FALSE) {
        glDisable(GL_CULL_FACE);
    }

    glPopDebugGroup();
}

void SelectionRenderer::render_volume(GLuint index_texture, GLuint volume_texture) {
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Render Volume");
    //
    //  Setup
    //
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //
    //  Volume rendering
    //
    glUseProgram(program.program_object);


    // Contour Buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.contour_information_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _gl_state.contour_information_ssbo);

    // Selection Buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.selection_list_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _gl_state.selection_list_ssbo);

    glUniform1i(_gl_state.uniform_locations_rendering.color_by_identifier, parameters.color_by_id ? 1 : 0);

    glUniform1i(_gl_state.uniform_locations_rendering.selection_emphasis_type, parameters.emphasize_by_selection);

    glUniform1f(_gl_state.uniform_locations_rendering.highlight_factor, parameters.highlight_factor);



    // Entry points texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glUniform1i(program.uniform_location.entry_texture, 0);

    // Exit points texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glUniform1i(program.uniform_location.entry_texture, 1);

    // Volume texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, volume_texture);
    glUniform1i(program.uniform_location.volume_texture, 2);

    // Transfer function texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_1D, transfer_function.texture);
    glUniform1i(program.uniform_location.transfer_function, 3);

    // Index Texture
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, index_texture);
    glUniform1i(_gl_state.uniform_locations_rendering.index_volume, 4);

    glUniform1f(program.uniform_location.sampling_rate, parameters.sampling_rate);


    // Rendering parameters
    glUniform3iv(program.uniform_location.volume_dimensions, 1,
        glm::value_ptr(parameters.volume_dimensions));
    glUniform3fv(program.uniform_location.volume_dimensions_rcp, 1,
        glm::value_ptr(parameters.volume_dimensions_rcp));
    glUniform3fv(program.uniform_location.light_position, 1,
        glm::value_ptr(parameters.light_position));
    glUniform3fv(program.uniform_location.light_color_ambient, 1,
                 glm::value_ptr(parameters.ambient));
    glUniform3fv(program.uniform_location.light_color_diffuse, 1,
                 glm::value_ptr(parameters.diffuse));
    glUniform3fv(program.uniform_location.light_color_specular, 1,
                 glm::value_ptr(parameters.specular));
    glUniform1f(program.uniform_location.light_exponent_specular,
                parameters.specular_exponent);

    // Bind a vao so we can render
    glBindVertexArray(bounding_box.vao);

    glDrawArrays(GL_TRIANGLES, 0, 6);
    glPopDebugGroup();
}

glm::vec3 SelectionRenderer::pick_volume_location(glm::ivec2 mouse_position, GLuint index_texture, GLuint volume_texture)
{
    glUseProgram(picking_program.program_object);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, index_texture);
    glUniform1i(_gl_state.uniform_locations_picking.index_volume, 4);

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Pick Volume");
    glBindFramebuffer(GL_FRAMEBUFFER, picking_framebuffer);

    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(picking_program.program_object);

    // Entry points texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glUniform1i(picking_program.uniform_location.entry_texture, 0);

    // Exit points texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glUniform1i(picking_program.uniform_location.entry_texture, 1);

    // Volume texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, volume_texture);
    glUniform1i(picking_program.uniform_location.volume_texture, 2);

    // Transfer function texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_1D, transfer_function.texture);
    glUniform1i(picking_program.uniform_location.transfer_function, 3);

    glUniform1f(picking_program.uniform_location.sampling_rate,
        parameters.sampling_rate);


    // Rendering parameters
    glUniform3iv(picking_program.uniform_location.volume_dimensions, 1,
        glm::value_ptr(parameters.volume_dimensions));
    glUniform3fv(picking_program.uniform_location.volume_dimensions_rcp,
        1, glm::value_ptr(parameters.volume_dimensions_rcp));

    // Bind a vao so we can render
    glBindVertexArray(bounding_box.vao);

    glDrawArrays(GL_TRIANGLES, 0, 6);
    glUseProgram(0);

    GLfloat colors[3];
    glReadPixels(mouse_position.x, mouse_position.y, 1, 1, GL_RGB, GL_FLOAT, colors);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glPopDebugGroup();

    return {
        colors[0],
        colors[1],
        colors[2]
    };
}

} // namespace volumerendering
