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
#include <glm/gtc/matrix_transform.hpp>

#include "bounding_polygon_plugin.h"

Bounding_Widget_3d::Bounding_Widget_3d(State& state) : _state(state) {}

void Bounding_Widget_3d::initialize(igl::opengl::glfw::Viewer* viewer, Bounding_Polygon_Menu* parent) {
    _viewer = viewer;
    _parent = parent;
    glm::ivec2 viewport_size = glm::ivec2(_viewer->core.viewport[2], _viewer->core.viewport[3]);

    volume_renderer.init(viewport_size);

    renderer_2d.init();
    cage_polyline_id = renderer_2d.add_polyline_3d(nullptr, nullptr, 0, PointLineRenderer::PolylineStyle());
    current_kf_polyline_id = renderer_2d.add_polyline_3d(nullptr, nullptr, 0, PointLineRenderer::PolylineStyle());
    skeleton_polyline_id = renderer_2d.add_polyline_3d(nullptr, nullptr, 0, PointLineRenderer::PolylineStyle());

    // Fix the model view matrices so the camera is centered on the volume
    {
        Eigen::MatrixXd V(8, 3);
        V << -.5f, -.5f, -.5f,
             -.5f, -.5f,  .5f,
             -.5f,  .5f, -.5f,
             -.5f,  .5f,  .5f,
              .5f, -.5f, -.5f,
              .5f, -.5f,  .5f,
              .5f,  .5f, -.5f,
              .5f,  .5f,  .5f;
        glm::ivec3 voldims = G3i(_state.low_res_volume.dims());
        Eigen::RowVector3d volume_dims(voldims[0], voldims[1], voldims[2]);
        Eigen::RowVector3d normalized_volume_dims = volume_dims / volume_dims.maxCoeff();
        for (int i = 0; i < V.rows(); i++) { V.row(i) *= normalized_volume_dims; }
        _viewer->core.align_camera_center(V);
    }
}

void Bounding_Widget_3d::update_volume_geometry(const Eigen::RowVector3d& volume_size, const Eigen::MatrixXd& cage_V, const Eigen::MatrixXi& cage_F) {
//    Eigen::RowVector3d volume_size = _state.low_res_volume.dims().cast<double>();
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

    volume_renderer.set_bounding_geometry((GLfloat*)V.data(), num_vertices, (GLint*)F.data(), num_faces);
}

void Bounding_Widget_3d::update_2d_geometry_curved(BoundingCage::KeyFrameIterator current_kf) {
    typedef Eigen::Matrix<GLfloat, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRm;
    const Eigen::RowVector3f volume_size = _state.low_res_volume.dims().cast<float>();

    std::vector<GLfloat> cageV;
    int num_vertices = 0;
    for (BoundingCage::Cell& cell : _state.cage.cells) {
        BoundingCage::KeyFrameIterator lkf = cell.left_keyframe(), rkf = cell.right_keyframe();
        Eigen::MatrixXd lkfV = lkf->bounding_box_vertices_3d();
        Eigen::MatrixXd rkfV = rkf->bounding_box_vertices_3d();

        for (int i = 0; i < lkfV.rows(); i++) {
            int next_i = (i + 1) % lkfV.rows();
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(i, j) / volume_size[j]); }
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(next_i, j) / volume_size[j]); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(i, j) / volume_size[j]); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(next_i, j) / volume_size[j]); }
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(i, j) / volume_size[j]); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(i, j) / volume_size[j]); }

            num_vertices += 6;
        }
    }
    glm::vec4 cage_color(0.2, 0.2, 0.8, 0.5);
    PointLineRenderer::PolylineStyle cage_style;
    cage_style.primitive = PointLineRenderer::LINES;
    cage_style.render_points = true;
    cage_style.line_width = 1.0f;
    cage_style.point_size = 4.0f;
    renderer_2d.update_polyline_3d(cage_polyline_id, cageV.data(),cage_color, num_vertices, cage_style);

    MatrixXfRm kfV = current_kf->bounding_box_vertices_3d().cast<GLfloat>();
    kfV.array().rowwise() /= volume_size.array();
    glm::vec4 kf_color(0.2, 0.8, 0.2, 0.3);
    PointLineRenderer::PolylineStyle kf_style;
    kf_style.primitive = PointLineRenderer::LINE_LOOP;
    kf_style.render_points = false;
    kf_style.line_width = 1.0f;
    kf_style.point_size = 4.0f;
    renderer_2d.update_polyline_3d(current_kf_polyline_id, kfV.data(), kf_color, kfV.rows(), kf_style);

    MatrixXfRm skV(_state.cage.num_keyframes(), 3);
    int count = 0;
    for (const BoundingCage::KeyFrame& kf : _state.cage.keyframes) {
        skV.row(count++) = kf.centroid_3d().cast<GLfloat>().array() / volume_size.array();
    }
    PointLineRenderer::PolylineStyle sk_style;
    sk_style.primitive = PointLineRenderer::LINE_STRIP;
    sk_style.render_points = false;
    glm::vec4 sk_color(0.8, 0.2, 0.2, 0.5);
    sk_style.line_width = 1.0f;
    renderer_2d.update_polyline_3d(skeleton_polyline_id, skV.data(), sk_color, skV.rows(), sk_style);
}

void Bounding_Widget_3d::update_2d_geometry_straight(BoundingCage::KeyFrameIterator current_kf) {
    typedef Eigen::Matrix<GLfloat, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRm;
    const Eigen::RowVector3f volume_size = _state.low_res_volume.dims().cast<float>();

    std::vector<GLfloat> cageV;
    int num_vertices = 0;

    std::vector<double> kf_lengths;
    _state.cage.keyframe_depths(kf_lengths);

    for (int count = 0; count < _state.cage.num_cells(); count++) {
        double max_length = kf_lengths.back();
        double z_left = kf_lengths[count] / max_length, z_right = kf_lengths[count+1] / max_length;


        Eigen::MatrixXd lkfV(4, 3); // = lkf->bounding_box_vertices_3d();
        lkfV << 0.0, 0.0, z_left,
                1.0, 0.0, z_left,
                1.0, 1.0, z_left,
                0.0, 1.0, z_left;
        Eigen::MatrixXd rkfV(4, 3); // = rkf->bounding_box_vertices_3d();
        rkfV << 0.0, 0.0, z_right,
                1.0, 0.0, z_right,
                1.0, 1.0, z_right,
                0.0, 1.0, z_right;

        for (int i = 0; i < lkfV.rows(); i++) {
            int next_i = (i + 1) % lkfV.rows();
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(i, j)); }
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(next_i, j)); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(i, j)); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(next_i, j)); }
            for (int j = 0; j < 3; j++) { cageV.push_back(lkfV(i, j)); }
            for (int j = 0; j < 3; j++) { cageV.push_back(rkfV(i, j)); }

            num_vertices += 6;
        }
    }
    glm::vec4 cage_color(0.2, 0.2, 0.8, 0.5);
    PointLineRenderer::PolylineStyle cage_style;
    cage_style.primitive = PointLineRenderer::LINES;
    cage_style.render_points = true;
    cage_style.line_width = 1.0f;
    cage_style.point_size = 4.0f;
    renderer_2d.update_polyline_3d(cage_polyline_id, cageV.data(),cage_color, num_vertices, cage_style);


    MatrixXfRm kfV(0, 3);
    glm::vec4 kf_color(0.2, 0.8, 0.2, 0.3);
    PointLineRenderer::PolylineStyle kf_style;
    kf_style.primitive = PointLineRenderer::LINE_LOOP;
    kf_style.render_points = false;
    kf_style.line_width = 1.0f;
    kf_style.point_size = 4.0f;
    renderer_2d.update_polyline_3d(current_kf_polyline_id, kfV.data(), kf_color, kfV.rows(), kf_style);

    MatrixXfRm skV(2, 3);
    Eigen::RowVector4d bbox = _state.cage.keyframe_bounding_box();
    double min_u = bbox[0], max_u = bbox[1], min_v = bbox[2], max_v = bbox[3];
    double bbox_w = max_u - min_u, bbox_h = max_v - min_v;
    double x = fabs(min_u) / bbox_w;
    double y = fabs(min_v) / bbox_h;
    skV << x, y, 0.0,
           x, y, 1.0;
    PointLineRenderer::PolylineStyle sk_style;
    sk_style.primitive = PointLineRenderer::LINE_STRIP;
    sk_style.render_points = false;
    glm::vec4 sk_color(0.8, 0.2, 0.2, 0.5);
    sk_style.line_width = 1.0f;
    renderer_2d.update_polyline_3d(skeleton_polyline_id, skV.data(), sk_color, skV.rows(), sk_style);
}

bool Bounding_Widget_3d::post_draw_straight(const glm::vec4 &viewport, BoundingCage::KeyFrameIterator current_kf) {
    // Back up the old viewport so we can restore it
    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    update_2d_geometry_straight(current_kf);

    glm::ivec2 viewport_size = glm::ivec2(viewport[2], viewport[3]);
    glm::ivec2 viewport_pos = glm::ivec2(viewport[0], viewport[1]);

    // Resize framebuffer textures to match the viewport size if it has changed since the last draw call
    if (glm::length(viewport - _last_viewport) > 1e-8) {
        _last_viewport = viewport;
        volume_renderer.resize_framebuffer(viewport_size);
        _state.logger->debug("Widget 3d resizing framebuffer textures");
    }

    /*
    glm::ivec3 volume_dims = _parent->exporter.export_dims();
    glm::vec3 normalized_straight_tex_size =
            glm::vec3(volume_dims) / static_cast<float>(glm::compMax(volume_dims));
    glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
    glm::mat4 scaling = glm::scale(glm::mat4(1.f), normalized_straight_tex_size);
    glm::mat4 model_matrix = GM4f(_viewer->core.model) * scaling * translate;
    glm::mat4 view_matrix = GM4f(_viewer->core.view);
    glm::mat4 proj_matrix = GM4f(_viewer->core.proj);
    glm::vec3 light_position = G3f(_viewer->core.light_position);

    glViewport(viewport_pos.x, viewport_pos.y, viewport_size.x, viewport_size.y);
    GLuint straight_tex = _parent->exporter.export_texture();

    std::vector<double> kf_lengths;
    _state.cage.keyframe_depths(kf_lengths);

    std::vector<int> sorted_cell_indexes;
    for (int i = 0; i < _state.cage.num_cells(); i++) { sorted_cell_indexes.push_back(i); }
    auto comp_cells = [&](int c1, int c2) -> bool {
        glm::vec4 voldims = glm::vec4(volume_dims, 1.0);
        glm::vec4 c1l = glm::vec4(0.0, 0.0, kf_lengths[c1], 1.0) / voldims;
        glm::vec4 c1r = glm::vec4(0.0, 0.0, kf_lengths[c1+1], 1.0) / voldims;
        glm::vec4 c2l = glm::vec4(0.0, 0.0, kf_lengths[c2], 1.0) / voldims;
        glm::vec4 c2r = glm::vec4(0.0, 0.0, kf_lengths[c2+1], 1.0) / voldims;

        glm::vec4 cam_ctr = glm::vec4(G3f(_viewer->core.camera_center), 1.0);

        c1l = model_matrix*c1l;
        c1r = model_matrix*c1r;
        c2l = model_matrix*c2l;
        c2r = model_matrix*c2r;

        float d1 = glm::max(glm::distance(c1l, cam_ctr), glm::distance(c1r, cam_ctr));
        float d2 = glm::max(glm::distance(c2l, cam_ctr), glm::distance(c2r, cam_ctr));

        return d1 < d2;
    };
    std::sort(sorted_cell_indexes.begin(), sorted_cell_indexes.end(), comp_cells);

    volume_renderer.set_step_size(1.0 / glm::length(G3f(_state.low_res_volume.dims())));
    volume_renderer.begin(volume_dims, straight_tex);
    for (int i = 0; i < _state.cage.num_cells(); i++) {
        double z_left = kf_lengths[sorted_cell_indexes[i]];
        double z_right = kf_lengths[sorted_cell_indexes[i]+1];
        Eigen::RowVector4d bbox = _state.cage.keyframe_bounding_box();
        double min_u = bbox[0], max_u = bbox[1], min_v = bbox[2], max_v = bbox[3];
        double bbox_w = max_u - min_u;
        double bbox_h = max_v - min_v;

        Eigen::MatrixXd cV(8, 3);
//        cV << 0.0,    0.0,    z_left,
//              bbox_w, 0.0,    z_left,
//              bbox_w, bbox_h, z_left,
//              0.0,    bbox_h, z_left,
//              0.0,    0.0,    z_right,
//              bbox_w, 0.0,    z_right,
//              bbox_w, bbox_h, z_right,
//              0.0,    bbox_h, z_right;
        cV << min_u, min_v, z_left,
              max_u, min_v, z_left,
              max_u, max_v, z_left,
              min_u, max_v, z_left,
              min_u, min_v, z_right,
              max_u, min_v, z_right,
              max_u, max_v, z_right,
              min_u, max_v, z_right;


        Eigen::MatrixXi cF(12, 3);
        cF << 3, 0, 1,
              3, 1, 2,
              5, 4, 7,
              5, 7, 6,
              0, 3, 7,
              0, 7, 4,
              3, 6, 7,
              6, 3, 2,
              2, 5, 6,
              5, 2, 1,
              1, 0, 5,
              5, 0, 4;

        update_volume_geometry(E3d(volume_dims), cV, cF);
        bool final = (i == _state.cage.num_cells()-1);
        std::cout << "final ? " << final << std::endl;
        volume_renderer.render_pass(model_matrix, view_matrix, proj_matrix, light_position, final);
    }
    */


    glViewport(viewport_pos.x, viewport_pos.y, viewport_size.x, viewport_size.y);
    GLuint straight_tex = _parent->exporter.export_texture();

    glm::vec3 volume_dims = glm::vec3(_parent->exporter.export_dims())/glm::vec3(2.0);
    glm::vec3 normalized_straight_tex_size =
            glm::vec3(volume_dims) / static_cast<float>(glm::compMax(volume_dims));
    glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
    glm::mat4 scaling = glm::scale(glm::mat4(1.f), normalized_straight_tex_size);
    glm::mat4 model_matrix = GM4f(_viewer->core.model) * scaling * translate;
    glm::mat4 view_matrix = GM4f(_viewer->core.view);
    glm::mat4 proj_matrix = GM4f(_viewer->core.proj);
    glm::vec3 light_position = G3f(_viewer->core.light_position);

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
    std::array<GLint, NUM_FACES*3> index_data = {
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

//    volume_renderer.set_step_size(1.0 / glm::length(G3f(_state.low_res_volume.dims())));
    volume_renderer.set_step_size(1.0 / glm::length(glm::vec3(volume_dims)));
    volume_renderer.begin(volume_dims, straight_tex);
    volume_renderer.set_bounding_geometry(vertex_data.data(), NUM_VERTICES, index_data.data(), NUM_FACES);
    volume_renderer.render_pass(model_matrix, view_matrix, proj_matrix, light_position, true /* final */);

    renderer_2d.draw(model_matrix, view_matrix, proj_matrix);
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
    return false;
}

bool Bounding_Widget_3d::post_draw_curved(const glm::vec4& viewport, BoundingCage::KeyFrameIterator current_kf) {
    // Back up the old viewport so we can restore it
    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    update_2d_geometry_curved(current_kf);

    glm::ivec2 viewport_size = glm::ivec2(viewport[2], viewport[3]);
    glm::ivec2 viewport_pos = glm::ivec2(viewport[0], viewport[1]);

    // Resize framebuffer textures to match the viewport size if it has changed since the last draw call
    if (glm::length(viewport - _last_viewport) > 1e-8) {
        _last_viewport = viewport;
        volume_renderer.resize_framebuffer(viewport_size);
        _state.logger->debug("Widget 3d resizing framebuffer textures");
    }

    // Geometry is in [0, 1]^3, rescale it to be centered and proportionally sized to the volume
    glm::ivec3 volume_dims = G3i(_state.low_res_volume.dims());
    glm::vec3 normalized_volume_dimensions = glm::vec3(volume_dims) / static_cast<float>(glm::compMax(volume_dims));
    glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
    glm::mat4 scaling = glm::scale(glm::mat4(1.f), normalized_volume_dimensions);

    glm::mat4 model_matrix = GM4f(_viewer->core.model) * scaling * translate;
    glm::mat4 view_matrix = GM4f(_viewer->core.view);
    glm::mat4 proj_matrix = GM4f(_viewer->core.proj);
    glm::vec3 light_position = G3f(_viewer->core.light_position);


    // Sort the cells of the bounding cage front to back
    std::vector<BoundingCage::CellIterator> sorted_cells;
    for (auto it = _state.cage.cells.begin(); it != _state.cage.cells.end(); it++) {
        sorted_cells.push_back(it);
    }
    auto comp_cells = [&](BoundingCage::CellIterator c1, BoundingCage::CellIterator c2) -> bool {
        glm::vec4 volume_dims = glm::vec4(G3i(_state.low_res_volume.dims()), 1.0);
        glm::vec4 c1l = glm::vec4(G3f(c1->left_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c1r = glm::vec4(G3f(c1->right_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c2l = glm::vec4(G3f(c2->left_keyframe()->centroid_3d()), 1.0) / volume_dims;
        glm::vec4 c2r = glm::vec4(G3f(c2->right_keyframe()->centroid_3d()), 1.0) / volume_dims;

        glm::vec4 cam_ctr = glm::vec4(G3f(_viewer->core.camera_center), 1.0);

        c1l = model_matrix*c1l;
        c1r = model_matrix*c1r;
        c2l = model_matrix*c2l;
        c2r = model_matrix*c2r;

        float d1 = glm::max(glm::distance(c1l, cam_ctr), glm::distance(c1r, cam_ctr));
        float d2 = glm::max(glm::distance(c2l, cam_ctr), glm::distance(c2r, cam_ctr));

        return d1 < d2;
    };
    std::sort(sorted_cells.begin(), sorted_cells.end(), comp_cells);


    glViewport(viewport_pos.x, viewport_pos.y, viewport_size.x, viewport_size.y);

    volume_renderer.set_step_size(1.0 / glm::length(glm::vec3(volume_dims)));
    volume_renderer.begin(volume_dims, _state.low_res_volume.volume_texture);
    for (int i = 0; i < sorted_cells.size(); i++) {
        auto cell = sorted_cells[i];
        Eigen::MatrixXd cV = cell->mesh_vertices();
        Eigen::MatrixXi cF = cell->mesh_faces();
        update_volume_geometry(_state.low_res_volume.dims().cast<double>(), cV, cF);

        bool final = (i == sorted_cells.size()-1);
        volume_renderer.render_pass(model_matrix, view_matrix, proj_matrix, light_position, final);
    }

    renderer_2d.draw(model_matrix, view_matrix, proj_matrix);

    // Restore the previous viewport
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
    return false;
}

bool Bounding_Widget_3d::pre_draw(float current_cut_index) {
    return false;
}
