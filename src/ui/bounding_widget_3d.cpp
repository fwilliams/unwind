#include "bounding_widget_3d.h"
#include "state.h"

#include <iomanip>

#include <utils/colors.h>
#include <utils/glm_conversion.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/edges.h>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

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

//    update_bounding_geometry(_state.cage.mesh_vertices(), _state.cage.mesh_faces());

    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Multipass framebuffer");
//    for (int i = 0; i < 1; i++) {
    glGenTextures(1, &_gl_state.texture[0]);
    glBindTexture(GL_TEXTURE_2D, _gl_state.texture[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, &_gl_state.framebuffer[0]);
    glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[0]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _gl_state.texture[0], 0);

//    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glPopDebugGroup();
}

void Bounding_Widget_3d::update_bounding_geometry(const Eigen::MatrixXd& cage_V, const Eigen::MatrixXi& cage_F) {
//    Eigen::MatrixXd cage_V = _state.cage.mesh_vertices();
    Eigen::RowVector3d volume_size = E3d(_state.volume_rendering.parameters.volume_dimensions);
//    Eigen::MatrixXi cage_F = _state.cage.mesh_faces();
    std::size_t num_vertices = cage_V.rows();
    std::size_t num_faces = cage_F.rows();
    std::vector<glm::vec3> V(num_vertices);
    std::vector<glm::ivec3> F(num_faces);
    for (int i = 0; i < num_vertices; i++) {
        double x = cage_V(i, 0) / volume_size[0];
        double y = cage_V(i, 1) / volume_size[1];
        double z = cage_V(i, 2) / volume_size[2];
        V[i] = glm::vec3(x, y, z);
    }
    for (int i = 0; i < num_faces; i++) {
        F[i] = glm::ivec3(cage_F(i, 0), cage_F(i, 1), cage_F(i, 2));
    }

//    volume_renderer.set_bounding_geometry((GLfloat*)V.data(), num_vertices, (GLint*)F.data(), num_faces);
}
bool Bounding_Widget_3d::post_draw(const glm::vec4& viewport) {
//    std::cout << glm::to_string(viewport) << std::endl;
    if (glm::length(viewport - _last_viewport) > 1e-8) {
        _last_viewport = viewport;
        glm::ivec2 viewport_size = glm::ivec2(viewport[2], viewport[3]);
        volume_renderer.resize_framebuffer(viewport_size);
        _viewer->core.viewport = E4f(viewport);

//        for (int i = 0; i < 1; i++) {
        glBindTexture(GL_TEXTURE_2D, _gl_state.texture[0]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, viewport_size.x, viewport_size.y, 0, GL_RGB, GL_FLOAT, nullptr);
//        }
        std::cout << "RESIZE!" << std::endl;
        glBindTexture(GL_TEXTURE_2D, 0);
    }

//    update_bounding_geometry(_state.cage.mesh_vertices(), _state.cage.mesh_faces());

    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

    glm::mat4 model_matrix = GM4f(_viewer->core.model);
    glm::mat4 view_matrix = GM4f(_viewer->core.view);
    glm::mat4 proj_matrix = GM4f(_viewer->core.proj);
    glm::vec3 light_position = G3f(_viewer->core.light_position);

    std::vector<BoundingCage::CellIterator> sorted_cells;
    for (auto it = _state.cage.cells.begin(); it != _state.cage.cells.end(); it++) {
        sorted_cells.push_back(it);
    }
    auto comp_cells = [&](BoundingCage::CellIterator c1, BoundingCage::CellIterator c2) -> bool {
        glm::vec4 volume_dims = glm::vec4(_state.volume_rendering.parameters.volume_dimensions, 1.0);
        glm::vec4 c1l = glm::vec4(G3f(c1->left_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c1r = glm::vec4(G3f(c1->right_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c2l = glm::vec4(G3f(c2->left_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c2r = glm::vec4(G3f(c2->right_keyframe()->centroid_3d()), 1.0) / volume_dims;

        c1l = view_matrix*model_matrix*c1l;
        c1r = view_matrix*model_matrix*c1r;
        c2l = view_matrix*model_matrix*c2l;
        c2r = view_matrix*model_matrix*c2r;
        float d1 = glm::max(glm::length2(glm::vec3(c1l)), glm::length2(glm::vec3(c1r)));
        float d2 = glm::max(glm::length2(glm::vec3(c2l)), glm::length2(glm::vec3(c2r)));
        return d1 < d2;
    };
    std::sort(sorted_cells.begin(), sorted_cells.end(), comp_cells);




    /*
//    {
//        auto cell = sorted_cells[0];
//        Eigen::MatrixXd cV = cell->mesh_vertices();
//        Eigen::MatrixXi cF = cell->mesh_faces();
//        update_bounding_geometry(cV, cF);
//        volume_renderer.render_bounding_box(model_matrix, view_matrix, proj_matrix);

//        glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[current_buf]);
//        volume_renderer.render_volume(light_position);

//        last_buf = current_buf;
//        current_buf = (current_buf + 1) % 2;
//    }

//    for (int i = 1; i < sorted_cells.size() - 1; i++) {
////        glClearColor(0.0, 0.0, 0.0, 0.0);
////        glClear(GL_COLOR_BUFFER_BIT);
//        auto cell = sorted_cells[i];
//        Eigen::MatrixXd cV = cell->mesh_vertices();
//        Eigen::MatrixXi cF = cell->mesh_faces();
//        update_bounding_geometry(cV, cF);
//        volume_renderer.render_bounding_box(model_matrix, view_matrix, proj_matrix);

//        glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[current_buf]);
//        volume_renderer.render_volume(light_position, _gl_state.texture[last_buf]);

//        last_buf = current_buf;
//        current_buf = (current_buf + 1) % 2;
//    }
    */


    int current_buf = 0;
    int last_buf = 0;


//    glDisable(GL_CULL_FACE);
    glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Multipass render");
    glm::ivec2 viewport_size = glm::ivec2(viewport[2], viewport[3]);
    glViewport(0, 0, viewport_size.x, viewport_size.y);
    glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[0]);
//    glClearColor(0.5, 0.6, 0.7, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "INCOMPLETE FRAMEBUFFER!!!!" << std::endl;
    }
    volume_renderer.render_volume(light_position, 0, 0.f);

    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    volume_renderer.render_volume(light_position, _gl_state.texture[0], 1.f);

//    glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[1]);
//    glClearColor(0.0, 0.0, 0.0, 0.0);
//    glClear(GL_COLOR_BUFFER_BIT);
//    glBindFramebuffer(GL_FRAMEBUFFER, 0);

//    {
//        auto cell = sorted_cells[sorted_cells.size()-2];
//        Eigen::MatrixXd cV = cell->mesh_vertices();
//        Eigen::MatrixXi cF = cell->mesh_faces();
//        update_bounding_geometry(cV, cF);
//        volume_renderer.render_bounding_box(model_matrix, view_matrix, proj_matrix);

//        glBindFramebuffer(GL_FRAMEBUFFER, _gl_state.framebuffer[0]);
//        volume_renderer.render_volume(light_position, _gl_state.texture[1], 0.f);
//        glBindFramebuffer(GL_FRAMEBUFFER, 0);
//        last_buf = current_buf;
//        current_buf = (current_buf + 1) % 2;
//    }

//    {
//        auto cell = sorted_cells[sorted_cells.size()-1];
//        Eigen::MatrixXd cV = cell->mesh_vertices();
//        Eigen::MatrixXi cF = cell->mesh_faces();
//        update_bounding_geometry(cV, cF);
//        volume_renderer.render_bounding_box(model_matrix, view_matrix, proj_matrix);

//        glBindFramebuffer(GL_FRAMEBUFFER, 0);
//        volume_renderer.render_volume(light_position, _gl_state.texture[0], 1.f);
//    }

    glPopDebugGroup();
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
    return false;
}

bool Bounding_Widget_3d::pre_draw(float current_cut_index) {
    return false;
}
