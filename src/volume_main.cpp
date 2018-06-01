#include <fstream>
#include <iostream>
#include <array>

#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/load_shader.h>
#include <igl/opengl/create_shader_program.h>
#include <igl/colormap.h>

using namespace igl::opengl;
using namespace igl::opengl::glfw;

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
} g_bounding_box;

struct Transfer_Function_Node {
    float t = 0.f;

    float rgba[4];
};

struct Transfer_Function {
    std::vector<Transfer_Function_Node> nodes;
    bool is_dirty = false;
    GLuint texture = 0;
};

struct Volume_Rendering {
    GLuint volume_texture = 0;

    Transfer_Function transfer_function;

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
} g_volume_rendering;

struct Volume_Rendering_Parameters {
    std::array<GLuint, 3> volume_dimensions = { 0, 0, 0 };
    std::array<GLfloat, 3> volume_dimensions_rcp = { 0.f, 0.f, 0.f };

    std::array<float, 3> normalized_volume_dimensions = { 0.f, 0.f, 0.f };

    GLfloat sampling_rate = 10.0;
} g_volume_rendering_parameters;


bool init(igl::opengl::glfw::Viewer& viewer);
void initialize(Bounding_Box& bounding_box, Volume_Rendering& volume_rendering,
    Volume_Rendering_Parameters& volume_rendering_parameters,
    Eigen::Vector4f viewport_size);

void upload_volume_data(const Volume_Rendering& volume_rendering, const Eigen::RowVector3i& tex_size, const Eigen::VectorXd& texture);

void update_transfer_function(Transfer_Function& transfer_function);


void render_bounding_box(const Bounding_Box& bounding_box,
    const Volume_Rendering_Parameters& volume_rendering_parameters,
    Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
    Eigen::Matrix4f proj_matrix);

void render_volume(const Bounding_Box& bounding_box,
    const Volume_Rendering& volume_rendering,
    const Volume_Rendering_Parameters& volume_rendering_parameters,
    Eigen::Matrix4f model_matrix,
    Eigen::Matrix4f view_matrix, Eigen::Matrix4f proj_matrix,
    Eigen::Vector3f light_position);

Eigen::Vector3f pick_volume_location(const Bounding_Box& bounding_box,
    const Volume_Rendering& volume_rendering,
    const Volume_Rendering_Parameters& volume_rendering_parameters,
    Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
    Eigen::Matrix4f proj_matrix, Eigen::Vector2i mouse_position);


bool load_rawfile(const std::string& rawfilename, const Eigen::RowVector3i& dims, Eigen::VectorXd& out, bool normalize = true) {
  const size_t num_bytes = dims[0]*dims[1]*dims[2];

  char* data = new char[num_bytes];
  std::ifstream rawfile(rawfilename, std::ifstream::binary);

  if (!rawfile.good()) {
    std::cerr << "ERROR: RawFile '" << rawfilename << "' does not exist." << std::endl;
    return false;
  }

  rawfile.read(data, num_bytes);
  if (!rawfile) {
    std::cerr << "ERROR: Only read " << rawfile.gcount() <<
                 " bytes from Raw File '" << rawfilename <<
                 "' but expected to read " << num_bytes <<
                 " bytes." << std::endl;
    return false;
  }
  rawfile.close();

  out.resize(num_bytes);
  for (int i = 0; i < num_bytes; i++) {
    out[i] = static_cast<double>(data[i]);
    if (normalize) {
      static_assert(sizeof(char) == sizeof(std::uint8_t), "Your system is fucked"); // This is dumb but why not
      out[i] /= 255.0;
    }
  }

  return true;
}


void initialize(Bounding_Box& bounding_box, Volume_Rendering& volume_rendering,
                       Volume_Rendering_Parameters& volume_rendering_parameters,
                       Eigen::Vector4f viewport_size)
{
    // This should be enabled by default
    glEnable(GL_TEXTURE_1D);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_TEXTURE_3D);

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
    constexpr const char* EntryBoxFragmentShader = R"(
#version 150
  in vec3 color;
  out vec4 out_color;

  void main() {
    out_color = vec4(color, 1.0);
  }
)";
    bounding_box.program = igl::opengl::create_shader_program(
        VertexShader,
        EntryBoxFragmentShader,
        { { "in_position", 0 } }
    );

    bounding_box.uniform_location.model_matrix = glGetUniformLocation(
        bounding_box.program, "model_matrix");
    bounding_box.uniform_location.view_matrix = glGetUniformLocation(
        bounding_box.program, "view_matrix");
    bounding_box.uniform_location.projection_matrix = glGetUniformLocation(
        bounding_box.program, "projection_matrix");






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
      out_color = vec4(0.0, 0.0, 0.0, 1.0);
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

    igl::opengl::create_shader_program(VolumeRenderingVertexShader,
        VolumeRenderingFragmentShader, {},
        volume_rendering.program.program_object);

    volume_rendering.program.uniform_location.entry_texture = glGetUniformLocation(
        volume_rendering.program.program_object, "entry_texture");
    volume_rendering.program.uniform_location.exit_texture = glGetUniformLocation(
        volume_rendering.program.program_object, "exit_texture");
    volume_rendering.program.uniform_location.volume_texture = glGetUniformLocation(
        volume_rendering.program.program_object, "volume_texture");
    volume_rendering.program.uniform_location.volume_dimensions = glGetUniformLocation(
        volume_rendering.program.program_object, "volume_dimensions");
    volume_rendering.program.uniform_location.volume_dimensions_rcp =
        glGetUniformLocation(
            volume_rendering.program.program_object, "volume_dimensions_rcp"
        );
    volume_rendering.program.uniform_location.transfer_function = glGetUniformLocation(
        volume_rendering.program.program_object, "transfer_function");
    volume_rendering.program.uniform_location.sampling_rate = glGetUniformLocation(
        volume_rendering.program.program_object, "sampling_rate");
    volume_rendering.program.uniform_location.light_position = glGetUniformLocation(
        volume_rendering.program.program_object, "light_parameters.position");
    volume_rendering.program.uniform_location.light_color_ambient = glGetUniformLocation(
        volume_rendering.program.program_object, "light_parameters.ambient_color");
    volume_rendering.program.uniform_location.light_color_diffuse = glGetUniformLocation(
        volume_rendering.program.program_object, "light_parameters.diffuse_color");
    volume_rendering.program.uniform_location.light_color_specular = glGetUniformLocation(
        volume_rendering.program.program_object, "light_parameters.specular_color");
    volume_rendering.program.uniform_location.light_exponent_specular =
        glGetUniformLocation(
            volume_rendering.program.program_object, "light_parameters.specular_exponent"
        );



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

    igl::opengl::create_shader_program(VolumeRenderingVertexShader,
        VolumeRenderingPickingFragmentShader, {},
        volume_rendering.picking_program.program_object);

    volume_rendering.picking_program.uniform_location.entry_texture =
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "entry_texture"
        );
    volume_rendering.picking_program.uniform_location.exit_texture =
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "exit_texture"
        );
    volume_rendering.picking_program.uniform_location.volume_texture = 
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "volume_texture"
        );
    volume_rendering.picking_program.uniform_location.volume_dimensions =
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "volume_dimensions"
        );
    volume_rendering.picking_program.uniform_location.volume_dimensions_rcp =
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "volume_dimensions_rcp"
        );
    volume_rendering.picking_program.uniform_location.transfer_function = 
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "transfer_function"
        );
    volume_rendering.picking_program.uniform_location.sampling_rate = 
        glGetUniformLocation(
            volume_rendering.picking_program.program_object, "sampling_rate"
        );


    // Entry point texture and frame buffer
    glGenTextures(1, &bounding_box.entry_texture);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size[2],
        viewport_size[3], 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &bounding_box.entry_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.entry_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        bounding_box.entry_texture, 0);


    // Exit point texture and frame buffer
    glGenTextures(1, &bounding_box.exit_texture);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size[2],
        viewport_size[3], 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &bounding_box.exit_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.exit_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        bounding_box.exit_texture, 0);

    // Volume texture
    glGenTextures(1, &volume_rendering.volume_texture);
    glBindTexture(GL_TEXTURE_3D, volume_rendering.volume_texture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Picking texture and framebuffer
    glGenTextures(1, &volume_rendering.picking_texture);
    glBindTexture(GL_TEXTURE_2D, volume_rendering.picking_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size[2],
        viewport_size[3], 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenFramebuffers(1, &volume_rendering.picking_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, volume_rendering.picking_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        volume_rendering.picking_texture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // Initialize transfer function
    // Texture
    glGenTextures(1, &volume_rendering.transfer_function.texture);
    glBindTexture(GL_TEXTURE_1D, volume_rendering.transfer_function.texture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);

    // Create the initial nodes
    Transfer_Function_Node first = { 0.f, { 0.f, 0.f, 0.f, 0.f } };
    Transfer_Function_Node last = { 1.f, { 1.f, 1.f, 1.f, 1.f } };
    volume_rendering.transfer_function.nodes.push_back(std::move(first));
    volume_rendering.transfer_function.nodes.push_back(std::move(last));
    volume_rendering.transfer_function.is_dirty = true;



    // Temporary
    Eigen::VectorXd data;
    constexpr const char* file = "D:/Segmentangling/2018/data/Sternopygus_arenatus-small.raw";
    Eigen::RowVector3i dims = { 216, 256, 622 };
    volume_rendering_parameters.volume_dimensions = { 216, 256, 622 };
    volume_rendering_parameters.volume_dimensions_rcp = {
        1.f / volume_rendering_parameters.volume_dimensions[0],
        1.f / volume_rendering_parameters.volume_dimensions[1],
        1.f / volume_rendering_parameters.volume_dimensions[2]
    };
    GLuint maxDim = *std::max_element(
        volume_rendering_parameters.volume_dimensions.begin(),
        volume_rendering_parameters.volume_dimensions.end());

    volume_rendering_parameters.normalized_volume_dimensions = {
        volume_rendering_parameters.volume_dimensions[0] / static_cast<float>(maxDim),
        volume_rendering_parameters.volume_dimensions[1] / static_cast<float>(maxDim),
        volume_rendering_parameters.volume_dimensions[2] / static_cast<float>(maxDim),
    };

    load_rawfile(file, dims, data, true);
    upload_volume_data(volume_rendering, dims, data);
    // Temporary end
}

bool init(igl::opengl::glfw::Viewer& viewer) {
    initialize(g_bounding_box, g_volume_rendering, g_volume_rendering_parameters,
        viewer.core.viewport);

    return true;
}

void upload_volume_data(const Volume_Rendering& volume_rendering,
                        const Eigen::RowVector3i& tex_size,
                        const Eigen::VectorXd& texture)
{
    std::vector<uint8_t> volume_data(texture.size());
    std::transform(
        texture.data(),
        texture.data() + texture.size(),
        volume_data.begin(),
        [](double d) {
            return static_cast<uint8_t>(d * std::numeric_limits<uint8_t>::max());
        }
    );

    glBindTexture(GL_TEXTURE_3D, volume_rendering.volume_texture);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, tex_size[0], tex_size[1], tex_size[2], 0,
                 GL_RED, GL_UNSIGNED_BYTE, volume_data.data());
    glBindTexture(GL_TEXTURE_3D, 0);
}

void update_transfer_function(Transfer_Function& transfer_function) {
    constexpr const int TransferFunctionWidth = 512;

    assert(
        std::is_sorted(
            transfer_function.nodes.begin(),
            transfer_function.nodes.end(),
            [](const Transfer_Function_Node& a, const Transfer_Function_Node& b) {
                return a.t < b.t;
            }
        )
    );
    assert(transfer_function.nodes.size() >= 2);
    assert(transfer_function.nodes.front().t == 0.f);
    assert(transfer_function.nodes.back().t == 1.f);


    std::vector<Transfer_Function_Node>::const_iterator current =
        transfer_function.nodes.begin();
    std::vector<Transfer_Function_Node>::const_iterator next = current + 1;

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

void render_bounding_box(const Bounding_Box& bounding_box,
                         const Volume_Rendering_Parameters& volume_rendering_parameters,
                         Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
                         Eigen::Matrix4f proj_matrix)
{
    //
    //  Pre-Rendering
    //
    glBindVertexArray(bounding_box.vao);
    glUseProgram(bounding_box.program);

    Eigen::Matrix4f scaling = Eigen::Matrix4f::Zero();
    scaling.diagonal() <<
        volume_rendering_parameters.normalized_volume_dimensions[0],
        volume_rendering_parameters.normalized_volume_dimensions[1],
        volume_rendering_parameters.normalized_volume_dimensions[2],
        1.f;
    Eigen::Matrix4f model = model_matrix * scaling;

    glUniformMatrix4fv(bounding_box.uniform_location.model_matrix, 1, GL_FALSE,
        model.data());

    glUniformMatrix4fv(bounding_box.uniform_location.view_matrix, 1, GL_FALSE,
        view_matrix.data());

    glUniformMatrix4fv(bounding_box.uniform_location.projection_matrix, 1, GL_FALSE,
        proj_matrix.data());

    // Render entry points of bounding box
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.entry_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);
    glCullFace(GL_FRONT);
    glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_BYTE, nullptr);

    // Render exit points of bounding box
    glBindFramebuffer(GL_FRAMEBUFFER, bounding_box.exit_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);
    glCullFace(GL_BACK);
    glDrawElements(GL_TRIANGLES, 12 * 3, GL_UNSIGNED_BYTE, nullptr);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

void render_volume(const Bounding_Box& bounding_box,
                   const Volume_Rendering& volume_rendering,
                   const Volume_Rendering_Parameters& volume_rendering_parameters,
                   Eigen::Matrix4f model_matrix, Eigen::Matrix4f view_matrix,
                   Eigen::Matrix4f proj_matrix, Eigen::Vector3f light_position)
{
    //
    //  Setup
    //
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //
    //  Volume rendering
    //
    glUseProgram(volume_rendering.program.program_object);

    // Entry points texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glUniform1i(volume_rendering.program.uniform_location.entry_texture, 0);

    // Exit points texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glUniform1i(volume_rendering.program.uniform_location.entry_texture, 1);

    // Volume texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, volume_rendering.volume_texture);
    glUniform1i(volume_rendering.program.uniform_location.volume_texture, 2);

    // Transfer function texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_1D, volume_rendering.transfer_function.texture);
    glUniform1i(volume_rendering.program.uniform_location.transfer_function, 3);

    glUniform1f(volume_rendering.program.uniform_location.sampling_rate,
        volume_rendering_parameters.sampling_rate);


    // Rendering parameters
    glUniform3uiv(volume_rendering.program.uniform_location.volume_dimensions, 3,
        volume_rendering_parameters.volume_dimensions.data());
    glUniform3fv(volume_rendering.program.uniform_location.volume_dimensions_rcp, 3,
        volume_rendering_parameters.volume_dimensions_rcp.data());
    glUniform3fv(volume_rendering.program.uniform_location.light_position, 3,
        light_position.data());
    glUniform3f(
        volume_rendering.program.uniform_location.light_color_ambient, 0.5f, 0.5f, 0.5f
    );
    glUniform3f(
        volume_rendering.program.uniform_location.light_color_diffuse, 0.8f, 0.8f, 0.8f
    );
    glUniform3f(
        volume_rendering.program.uniform_location.light_color_specular, 1.f, 1.f, 1.f
    );
    glUniform1f(
        volume_rendering.program.uniform_location.light_exponent_specular, 10.f
    );

    glDrawArrays(GL_TRIANGLES, 0, 6);
}

Eigen::Vector3f pick_volume_location(const Bounding_Box& bounding_box,
                                     const Volume_Rendering& volume_rendering,
                           const Volume_Rendering_Parameters& volume_rendering_parameters,
                                                             Eigen::Matrix4f model_matrix,
                                                              Eigen::Matrix4f view_matrix,
                                                              Eigen::Matrix4f proj_matrix,
                                                           Eigen::Vector2i mouse_position)
{
    glBindFramebuffer(GL_FRAMEBUFFER, volume_rendering.picking_framebuffer);
    //glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(volume_rendering.picking_program.program_object);

    // Entry points texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bounding_box.entry_texture);
    glUniform1i(volume_rendering.picking_program.uniform_location.entry_texture, 0);

    // Exit points texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bounding_box.exit_texture);
    glUniform1i(volume_rendering.picking_program.uniform_location.entry_texture, 1);

    // Volume texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_3D, volume_rendering.volume_texture);
    glUniform1i(volume_rendering.picking_program.uniform_location.volume_texture, 2);

    // Transfer function texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_1D, volume_rendering.transfer_function.texture);
    glUniform1i(volume_rendering.picking_program.uniform_location.transfer_function, 3);

    glUniform1f(volume_rendering.picking_program.uniform_location.sampling_rate,
        volume_rendering_parameters.sampling_rate);


    // Rendering parameters
    glUniform3uiv(volume_rendering.picking_program.uniform_location.volume_dimensions, 3,
        volume_rendering_parameters.volume_dimensions.data());
    glUniform3fv(volume_rendering.picking_program.uniform_location.volume_dimensions_rcp,
        3, volume_rendering_parameters.volume_dimensions_rcp.data());

    glDrawArrays(GL_TRIANGLES, 0, 6);
    glUseProgram(0);

    GLfloat colors[3];
    glReadPixels(mouse_position[0], mouse_position[1], 1, 1, GL_RGB, GL_FLOAT, colors);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return {
        colors[0],
        colors[1],
        colors[2]
    };
}

bool post_draw(igl::opengl::glfw::Viewer& viewer) {
    //
    //  Setup
    //
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (g_volume_rendering.transfer_function.is_dirty) {
        update_transfer_function(g_volume_rendering.transfer_function);
        g_volume_rendering.transfer_function.is_dirty = false;
    }

    render_bounding_box(g_bounding_box, g_volume_rendering_parameters, viewer.core.model,
        viewer.core.view, viewer.core.proj);

    render_volume(g_bounding_box, g_volume_rendering, g_volume_rendering_parameters,
                  viewer.core.model, viewer.core.view, viewer.core.proj,
                  viewer.core.light_position);

    pick_volume_location(g_bounding_box, g_volume_rendering,
        g_volume_rendering_parameters, viewer.core.model, viewer.core.view,
        viewer.core.proj,
        { viewer.current_mouse_x, viewer.core.viewport[3] - viewer.current_mouse_y });

    return true;
}

int main(int argc, char *argv[])
{
  // Inline mesh of a cube
  const Eigen::MatrixXd V= (Eigen::MatrixXd(8,3)<<
    0.0,0.0,0.0,
    0.0,0.0,1.0,
    0.0,1.0,0.0,
    0.0,1.0,1.0,
    1.0,0.0,0.0,
    1.0,0.0,1.0,
    1.0,1.0,0.0,
    1.0,1.0,1.0).finished();
  const Eigen::MatrixXi F = (Eigen::MatrixXi(12,3)<<
    1,7,5,
    1,3,7,
    1,4,3,
    1,2,4,
    3,8,7,
    3,4,8,
    5,7,8,
    5,8,6,
    1,5,6,
    1,6,2,
    2,6,8,
    2,8,4).finished().array()-1;


  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_face_based(true);

  viewer.callback_init = init;
  viewer.callback_post_draw = post_draw;
  
  viewer.launch();
}
