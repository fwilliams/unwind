#include "bounding_widget_3d.h"
#include "state.h"

#include <utils/colors.h>
#include <utils/glm_conversion.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/edges.h>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>

#include "volume_fragment_shader.h"
#include "picking_fragment_shader.h"


Bounding_Widget_3d::Bounding_Widget_3d(State& state) : _state(state) {}

void Bounding_Widget_3d::initialize(igl::opengl::glfw::Viewer* viewer) {
    _viewer = viewer;
    glm::ivec2 viewport_size = glm::ivec2(_viewer->core.viewport[2], _viewer->core.viewport[3]);
    volume_renderer.init(viewport_size);
    volume_renderer.set_volume_data(_state.volume_rendering.parameters.volume_dimensions, _state.volume_data.data());

    // Fix the model view matrices
    Eigen::MatrixXd V(8, 3);
    V << -.5f, -.5f, -.5f,
         -.5f, -.5f,  .5f,
         -.5f,  .5f, -.5f,
         -.5f,  .5f,  .5f,
          .5f, -.5f, -.5f,
          .5f, -.5f,  .5f,
          .5f,  .5f, -.5f,
          .5f,  .5f,  .5f;
    glm::ivec3 voldims = _state.volume_rendering.parameters.volume_dimensions;
    Eigen::RowVector3d volume_dims(voldims[0], voldims[1], voldims[2]);
    Eigen::RowVector3d normalized_volume_dims = volume_dims / volume_dims.maxCoeff();
    for (int i = 0; i < V.rows(); i++) { V.row(i) *= normalized_volume_dims; }
    _viewer->core.align_camera_center(V);
}

bool Bounding_Widget_3d::post_draw() {
    glm::mat4 model_matrix = GM4f(_viewer->core.model);
    glm::mat4 view_matrix = GM4f(_viewer->core.view);
    glm::mat4 proj_matrix = GM4f(_viewer->core.proj);
    glm::vec3 light_position = G3f(_viewer->core.light_position);

    volume_renderer.render_bounding_box(model_matrix, view_matrix, proj_matrix);
    volume_renderer.render_volume(light_position);
    return false;
}

bool Bounding_Widget_3d::pre_draw(float current_cut_index) {
    return false;
}
