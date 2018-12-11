#include "bounding_polygon_plugin.h"

#include "state.h"
#include "utils/colors.h"
#include "utils/utils.h"
#include <igl/edges.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#pragma optimize ("", off)

Bounding_Polygon_Menu::Bounding_Polygon_Menu(State& state)
    : state(state)
    , widget_2d(Bounding_Polygon_Widget(state))
{}


void Bounding_Polygon_Menu::initialize() {
    // Store a backup copy of the viewer viewport and then set the viewport to the size specified by the
    // layout constraints of the widget
    old_viewport = viewer->core.viewport;
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    viewer_viewport = Eigen::RowVector4f(view_hsplit*window_width, view_vsplit*window_height,
                                         (1.0-view_hsplit)*window_width, (1.0-view_vsplit)*window_height);
    viewer->core.viewport = viewer_viewport;

    // Clear all meshes previously stored in the viewer so we have a clean slate
    for (size_t i = viewer->data_list.size() - 1; i > 0; i--) {
        viewer->erase_mesh(i);
    }
    viewer->selected_data_index = mesh_overlay_id;

    const Eigen::MatrixXd& TV = state.extracted_volume.TV;
    const Eigen::MatrixXi& TF = state.extracted_volume.TF;
    viewer->data().set_mesh(TV, TF);
    viewer->core.align_camera_center(TV, TF);

    viewer->append_mesh();
    points_overlay_id = viewer->selected_data_index;

    int push_mesh_id = mesh_overlay_id;
    viewer->selected_data_index = mesh_overlay_id;
    {
        viewer->data().clear();
        Eigen::MatrixXi E;
        igl::edges(state.extracted_volume.TF, E);
        const Eigen::RowVector3d color(0.75, 0.75, 0.75);
        viewer->data().set_edges(state.extracted_volume.TV, E, color);

        Eigen::MatrixXd P1(state.cage.skeleton_vertices().rows() - 1, 3),
                P2(state.cage.skeleton_vertices().rows() - 1, 3);
        for (int i = 0; i < state.cage.skeleton_vertices().rows() - 1; i++) {
            P1.row(i) = state.cage.skeleton_vertices().row(i);
            P2.row(i) = state.cage.skeleton_vertices().row(i + 1);
        }
        viewer->data().add_edges(P1, P2, ColorRGB::LIGHT_GREEN);
        viewer->data().point_size = 10.f;

        for (int i = 0; i < state.cage.smooth_skeleton_vertices().rows() - 1; i++) {
            P1.row(i) = state.cage.smooth_skeleton_vertices().row(i);
            P2.row(i) = state.cage.smooth_skeleton_vertices().row(i + 1);
        }
        viewer->data().add_edges(P1, P2, ColorRGB::RED);
        viewer->data().point_size = 20.f;

        viewer->selected_data_index = push_mesh_id;
    }

    // Draw the bounding cage mesh
    viewer->append_mesh();
    cage_mesh_overlay_id = viewer->selected_data_index;
    {
        viewer->data().clear();
        viewer->data().set_face_based(true);
        Eigen::MatrixXi faces(0, 3);
        for (const BoundingCage::Cell& cell : state.cage.cells) {
            int idx = faces.rows();
            faces.conservativeResize(faces.rows() + cell.mesh_faces().rows(), 3);
            faces.block(idx, 0, cell.mesh_faces().rows(), 3) = cell.mesh_faces();
        }
        viewer->data().set_mesh(state.cage.mesh_vertices(), faces);
    }

    // Initialize the 2d cross section widget
    widget_2d.initialize(viewer);

    state.logger->trace("Done initializing bounding polygon plugin!");
}

void Bounding_Polygon_Menu::deinitialize() {
    viewer->core.viewport = old_viewport;
}

bool Bounding_Polygon_Menu::mouse_move(int mouse_x, int mouse_y) {
    return widget_2d.mouse_move(mouse_x, mouse_y) || FishUIViewerPlugin::mouse_move(mouse_x, mouse_y);;
}


bool Bounding_Polygon_Menu::mouse_down(int button, int modifier) {
    return widget_2d.mouse_down(button, modifier) || FishUIViewerPlugin::mouse_down(button, modifier);
}


bool Bounding_Polygon_Menu::mouse_up(int button, int modifier) {
    return widget_2d.mouse_up(button, modifier) || FishUIViewerPlugin::mouse_up(button, modifier);
}

bool Bounding_Polygon_Menu::mouse_scroll(float delta_y) {
    return widget_2d.mouse_scroll(delta_y) || FishUIViewerPlugin::mouse_scroll(delta_y);
}


bool Bounding_Polygon_Menu::pre_draw() {
    bool ret = FishUIViewerPlugin::pre_draw();

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);

    glDisable(GL_CULL_FACE);

    size_t push_overlay_id = viewer->selected_data_index;

    viewer->selected_data_index = points_overlay_id;
    {
        viewer->data().clear();
        viewer->data().line_width = 2.4f;

        for (BoundingCage::KeyFrame& kf : state.cage.keyframes) {
            viewer->data().add_points(kf.center(), ColorRGB::GREEN);
            Eigen::MatrixXd V3d = kf.vertices_3d();
            Eigen::RowVector3d clr = kf.index() == current_cut_index ?
                        ColorRGB::ORANGERED : ColorRGB::RED;
            for (int i = 0; i < V3d.rows(); i++) {
                Eigen::RowVector3d c = (i == current_vertex ? ColorRGB::CYAN : clr);
                viewer->data().add_points(V3d.row(i), c);
            }
        }

        for (auto kf = state.cage.keyframes.rbegin(); kf != state.cage.keyframes.rend(); --kf) {
            Eigen::Matrix3d cf = kf->orientation();
            if (kf->index() != current_cut_index) {
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(0), ColorRGB::RED);
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(1), ColorRGB::GREEN);
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(2), ColorRGB::BLUE);
            }
            else {
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(0), ColorRGB::CYAN);
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(1), ColorRGB::YELLOW);
                viewer->data().add_edges(kf->center(), kf->center() + 100.0 * cf.row(2), ColorRGB::MAGENTA);
            }
        }

        viewer->data().point_size = 10.f;
        BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);

        for (const BoundingCage::Cell& cell : state.cage.cells) {
            Eigen::MatrixXd P1;
            Eigen::MatrixXd P2;
            edge_endpoints(state.cage.mesh_vertices(), cell.mesh_faces(), P1, P2);
            viewer->data().add_edges(P1, P2, ColorRGB::GREEN);
        }

        viewer->data().add_points(kf->center(), ColorRGB::RED);
        viewer->data().add_points(kf->vertices_3d(), ColorRGB::ORANGERED);
        viewer->data().add_edges(kf->center(), kf->center() + 100 * kf->orientation().row(0), ColorRGB::CYAN);
        viewer->data().add_edges(kf->center(), kf->center() + 100 * kf->orientation().row(1), ColorRGB::YELLOW);
        viewer->data().add_edges(kf->center(), kf->center() + 100 * kf->orientation().row(2), ColorRGB::MAGENTA);
        viewer->selected_data_index = push_overlay_id;
    }


    viewer->selected_data_index = cage_mesh_overlay_id;
    {
        viewer->data().clear();
        viewer->data().set_face_based(true);
        Eigen::MatrixXi faces(0, 3);
        for (const BoundingCage::Cell& cell : state.cage.cells) {
            int idx = faces.rows();
            faces.conservativeResize(faces.rows() + cell.mesh_faces().rows(), 3);
            faces.block(idx, 0, cell.mesh_faces().rows(), 3) = cell.mesh_faces();
        }
        viewer->data().set_mesh(state.cage.mesh_vertices(), faces);
        viewer->selected_data_index = push_overlay_id;
    }

    return ret;
}


bool Bounding_Polygon_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);
    widget_2d.position = glm::vec2(0.f, view_vsplit*window_height);
    widget_2d.size = glm::vec2(window_width*view_hsplit, (1.0-view_vsplit)*window_height);
    ret = widget_2d.post_draw(state.cage.keyframe_for_index(current_cut_index), static_cast<int>(current_cut_index));

    ImGui::SetNextWindowBgAlpha(0.0f);
    float window_height_float = static_cast<float>(window_height);
    float window_width_float = static_cast<float>(window_width);
    ImGui::SetNextWindowPos(ImVec2(0.f, (1.0-view_vsplit)*window_height_float), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImVec2(window_width_float, window_height_float*view_vsplit), ImGuiSetCond_Always);
    ImGui::Begin("Select Boundary", nullptr,
                 ImGuiWindowFlags_NoSavedSettings |
                 ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoTitleBar);

    if (ImGui::Button("< Prev")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it--;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.begin();
        }
        current_cut_index = static_cast<float>(it->index());
    }
    ImGui::SameLine();
    bool changed_slider = ImGui::SliderFloat("#vertexid", &current_cut_index,
                                             static_cast<float>(state.cage.min_index()),
                                             static_cast<float>(state.cage.max_index()));
    if (changed_slider) {
        const float min = static_cast<float>(state.cage.min_index());
        const float max = static_cast<float>(state.cage.max_index());

        current_cut_index = std::max(min, std::min(current_cut_index, max));
    }
    ImGui::SameLine();
    if (ImGui::Button("Next >")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it++;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.rbegin();
        }
        current_cut_index = static_cast<float>(it->index());
    }

    if (ImGui::Button("Insert KF")) {
        state.cage.insert_keyframe(current_cut_index);
        glfwPostEmptyEvent();
    }
    if (ImGui::Button("Remove KF")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        state.cage.delete_keyframe(it);
        glfwPostEmptyEvent();
    }
    BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
    if (ImGui::InputInt("vertex: ", &current_vertex, 1, 1)) {
        current_vertex = std::max(0, current_vertex);
        current_vertex = std::min(current_vertex, static_cast<int>(it->vertices_2d().rows() - 1));
    }
    ImGui::SameLine();
    if (ImGui::Button("+x")) {
        Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(1, 0);
        const bool validate_2d = true;
        const bool validate_3d = false;
        it->move_point_2d(current_vertex, pt, validate_2d, validate_3d);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("-x")) {
        Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(-1, 0);
        const bool validate_2d = true;
        const bool validate_3d = false;
        it->move_point_2d(current_vertex, pt, validate_2d, validate_3d);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("+y")) {
        Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(0, 1);
        const bool validate_2d = true;
        const bool validate_3d = false;
        it->move_point_2d(current_vertex, pt, validate_2d, validate_3d);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("-y")) {
        Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(0, -1);
        const bool validate_2d = true;
        const bool validate_3d = false;
        it->move_point_2d(current_vertex, pt, validate_2d, validate_3d);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Vertex")) {
        state.cage.insert_boundary_vertex(current_vertex, 0.5);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("Rmv Vertex")) {
        state.cage.delete_boundary_vertex(current_vertex);
        glfwPostEmptyEvent();
    }

    ImGui::End();
    ImGui::Render();

    return ret;
}

