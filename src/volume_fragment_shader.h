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
constexpr const char* ContourTreeFragmentShader = R"(
#version 450
  in vec2 uv;
  out vec4 out_color;

  layout (std430, binding = 0) buffer Contour {
      uint nFeatures;
      uint values[];
  } contour;

  uniform sampler2D entry_texture;
  uniform sampler2D exit_texture;

  uniform sampler3D volume_texture;
  uniform sampler1D transfer_function;

  uniform usampler3D index_volume;
  uniform int color_by_identifier;

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

  uniform int id;

  // Early-ray termination
  const float ERT_THRESHOLD = 0.99;
  const float REF_SAMPLING_INTERVAL = 150.0;

  // Code from https://raw.githubusercontent.com/kbinani/glsl-colormap/master/shaders/transform_rainbow.frag
  // Under MIT license

  vec4 colormap(float x) {
    float r = 0.0, g = 0.0, b = 0.0;

    if (x < 0.0) {
      r = 127.0 / 255.0;
    } else if (x <= 1.0 / 9.0) {
      r = 1147.5 * (1.0 / 9.0 - x) / 255.0;
    } else if (x <= 5.0 / 9.0) {
      r = 0.0;
    } else if (x <= 7.0 / 9.0) {
      r = 1147.5 * (x - 5.0 / 9.0) / 255.0;
    } else {
      r = 1.0;
    }

    if (x <= 1.0 / 9.0) {
      g = 0.0;
    } else if (x <= 3.0 / 9.0) {
      g = 1147.5 * (x - 1.0 / 9.0) / 255.0;
    } else if (x <= 7.0 / 9.0) {
      g = 1.0;
    } else if (x <= 1.0) {
      g = 1.0 - 1147.5 * (x - 7.0 / 9.0) / 255.0;
    } else {
      g = 0.0;
    }

    if (x <= 3.0 / 9.0) {
      b = 1.0;
    } else if (x <= 5.0 / 9.0) {
      b = 1.0 - 1147.5 * (x - 3.0 / 9.0) / 255.0;
    } else {
      b = 0.0;
    }

    return vec4(r, g, b, 1.0);
  }


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

      const uint segVoxel = texture(index_volume, sample_pos).r;
      const uint feature = contour.values[segVoxel];

      if (feature != -1) {
      //if (feature == id) {
        float value = texture(volume_texture, sample_pos).r;
        vec4 color;
        if (color_by_identifier == 1) {
            const float normFeature = float(feature + 1) / float(contour.nFeatures);
            color.rgb = colormap(normFeature).rgb;
            color.a = 1.0;
        }
        else {
          color = texture(transfer_function, value);
        }
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
