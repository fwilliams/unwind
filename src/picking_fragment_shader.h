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
constexpr const char* ContourTreePickingFragmentShader = R"(
#version 450
  in vec2 uv;
  out vec4 out_color;

  layout (std430, binding = 0) buffer Contour {
      uint nFeatures;
      uint values[];
  } contour;


  uniform sampler2D entry_texture;
  uniform sampler2D exit_texture;

  uniform usampler3D index_volume;

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
      const uint segVoxel = texture(index_volume, sample_pos).r;
      const uint feature = contour.values[segVoxel];
      if (feature != -1) {
        out_color = vec4(vec3(float(feature + 1)), 1.0);
        return;
      }
      else {
        t += t_incr;
      }
    }

    out_color = vec4(0.0);
  }
)";
