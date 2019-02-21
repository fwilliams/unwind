#include "endpoint_selection_plugin.h"

#include "state.h"
#include "utils/colors.h"
#include "utils/utils.h"

#include <igl/boundary_facets.h>
#include <igl/marching_tets.h>
#include <igl/unproject_onto_mesh.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <unordered_set>

namespace {

bool validate_endpoint_pairs(const std::vector<std::pair<int, int>>& endpoints,
                             const Eigen::VectorXi& components) {
    bool success = true;
    std::unordered_set<int> computed_components;

    for (int i = 0; i < endpoints.size(); i++) {
        const int c1 = components[endpoints[i].first];

        // abock(2018-09-10): c2 was the same as c1 before, I think in error
        const int c2 = components[endpoints[i].second];
        if (c1 != c2) {
            success = false;
            break;
        }
        if (computed_components.find(c1) != computed_components.end()) {
            success = false;
            break;
        }
        else {
            computed_components.insert(c1);
        }
    }

    return success;
}


void compute_skeleton(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                      const Eigen::VectorXd normalized_distances,
                      const std::vector<std::pair<int, int>>& endpoint_pairs,
                      const Eigen::VectorXi& connected_components,
                      int num_skeleton_vertices,
                      Eigen::MatrixXd& skeleton_vertices)
{
    std::vector<Eigen::MatrixXi> TT_comps;
    split_mesh_components(TT, connected_components, TT_comps);

    Eigen::MatrixXd LV;
    Eigen::MatrixXi LF;

    int vertex_count = 0;
    skeleton_vertices.resize(num_skeleton_vertices, 3);

    for (int ep_i = 0; ep_i < endpoint_pairs.size(); ep_i++) {
        const int component = connected_components[endpoint_pairs[ep_i].first];
        skeleton_vertices.row(vertex_count) = TV.row(endpoint_pairs[ep_i].first);
        vertex_count++;

        const double nd_ep0 = normalized_distances[endpoint_pairs[ep_i].first];
        const double nd_ep1 = normalized_distances[endpoint_pairs[ep_i].second];
        const double isoval_incr = (nd_ep1 - nd_ep0) / num_skeleton_vertices;

        double isovalue = normalized_distances[endpoint_pairs[ep_i].first] + isoval_incr;
        for (int i = 0; i < num_skeleton_vertices - 2; i++) {
            igl::marching_tets(TV, TT_comps[component], normalized_distances, isovalue, LV, LF);
            if (LV.rows() == 0) {
                isovalue += isoval_incr;
                continue;
            }
            Eigen::RowVector3d c = LV.colwise().sum() / LV.rows();
            skeleton_vertices.row(vertex_count) = c;
            vertex_count += 1;
            isovalue += isoval_incr;
        }

        skeleton_vertices.row(vertex_count) = TV.row(endpoint_pairs[ep_i].second);
        vertex_count += 1;
    }

    skeleton_vertices.conservativeResize(vertex_count, 3);
}


void smooth_skeleton(const Eigen::MatrixXd& skeleton_vertices,
                     Eigen::MatrixXd& smoothed_vertices)
{
    smoothed_vertices.resize(skeleton_vertices.rows(), 3);
    smoothed_vertices.row(0) = skeleton_vertices.row(0);
    for (int i = 1; i < skeleton_vertices.rows() - 1; i++) {
        smoothed_vertices.row(i) = 0.5 * (skeleton_vertices.row(i - 1) +
                                   skeleton_vertices.row(i + 1));
    }
    smoothed_vertices.row(skeleton_vertices.rows() - 1) = skeleton_vertices.row(skeleton_vertices.rows() - 1);
}

} // namespace

EndPoint_Selection_Menu::EndPoint_Selection_Menu(State& state) : state(state) {
  extracting_skeleton = false;
  done_extracting_skeleton = false;
}


void EndPoint_Selection_Menu::initialize() {
    for (size_t i = viewer->data_list.size() - 1; i > 0; i--) {
        viewer->erase_mesh(i);
    }
    viewer->data().clear();
    viewer->append_mesh();

    mesh_overlay_id = static_cast<int>(viewer->selected_data_index);
    viewer->selected_data_index = mesh_overlay_id;

    const Eigen::MatrixXd& TV = state.dilated_tet_mesh.TV;
    const Eigen::MatrixXi& TF = state.dilated_tet_mesh.TF;
    viewer->data().set_mesh(TV, TF);
    viewer->core.align_camera_center(TV, TF);

    viewer->append_mesh();
    points_overlay_id = static_cast<int>(viewer->selected_data_index);

    viewer->selected_data_index = mesh_overlay_id;

    old_viewport = viewer->core.viewport;

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    viewer->core.viewport = Eigen::RowVector4f(view_hsplit*window_width, 0, (1.0-view_hsplit)*window_width, window_height);

    state.endpoint_pairs.clear();
}

void EndPoint_Selection_Menu::deinitialize() {
    for (size_t i = viewer->data_list.size() - 1; i > 0; i--) {
        viewer->erase_mesh(i);
    }
    viewer->data().clear();
    viewer->core.viewport = old_viewport;
}

bool EndPoint_Selection_Menu::pre_draw() {
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    viewer->core.viewport = Eigen::RowVector4f(view_hsplit*window_width, 0, (1.0-view_hsplit)*window_width, window_height);

    bool ret = FishUIViewerPlugin::pre_draw();
    const Eigen::MatrixXd& TV = state.dilated_tet_mesh.TV;

    int push_mesh_id = static_cast<int>(viewer->selected_data_index);
    viewer->selected_data_index = points_overlay_id;
    viewer->data().clear();

    if (selecting_endpoints) {
        for (unsigned int i = 0; i < current_endpoint_idx; i++) {
            const int vid = current_endpoints[i];
            viewer->data().add_points(TV.row(vid), i == 0 ? ColorRGB::GREEN : ColorRGB::RED);
        }
    }

    for (int i = 0; i < state.endpoint_pairs.size(); i++) {
        std::pair<int, int> ep = state.endpoint_pairs[i];
        viewer->data().add_points(TV.row(ep.first), ColorRGB::GREEN);
        viewer->data().add_points(TV.row(ep.second), ColorRGB::RED);
    }

    viewer->selected_data_index = push_mesh_id;

    return ret;
}

void EndPoint_Selection_Menu::debug_draw_intermediate_state() {
    if (debug.drew_debug_state) {
        return;
    }

    viewer->data().clear();
    const Eigen::MatrixXd& TV = state.dilated_tet_mesh.TV;
    const Eigen::MatrixXi& TF = state.dilated_tet_mesh.TF;
    Eigen::MatrixXd V1, V2;
    edge_endpoints(TV, TF, V1, V2);
    viewer->data().add_edges(V1, V2, ColorRGB::SILVER);

    size_t skel_mesh = viewer->append_mesh() - 1;
    viewer->selected_data_index = skel_mesh;
    {
        const Eigen::MatrixXd& skV = state.cage.skeleton_vertices();
        V1.resize(skV.rows()-1, 3);
        V2.resize(skV.rows()-1, 3);
        for (int i = 0; i < skV.rows()-1; i++) {
            V1.row(i) = skV.row(i);
            V2.row(i) = skV.row(i+1);
        }
        viewer->data().point_size = 5.0;
        viewer->data().line_width = 2.0;
        viewer->data().add_edges(V1, V2, ColorRGB::GREEN);
        viewer->data().add_points(skV, ColorRGB::GREEN);
    }

    size_t skel_mesh2 = viewer->append_mesh() - 1;
    viewer->selected_data_index = skel_mesh2;
    {
        const Eigen::MatrixXd& skV = state.cage.smooth_skeleton_vertices();
        V1.resize(skV.rows()-1, 3);
        V2.resize(skV.rows()-1, 3);
        for (int i = 0; i < skV.rows()-1; i++) {
            V1.row(i) = skV.row(i);
            V2.row(i) = skV.row(i+1);
        }
        viewer->data().point_size = 5.0;
        viewer->data().line_width = 2.0;
        viewer->data().add_edges(V1, V2, ColorRGB::DARK_MAGENTA);
        viewer->data().add_points(skV, ColorRGB::DARK_MAGENTA);
    }

    size_t kf_mesh = viewer->append_mesh() - 1;
    viewer->selected_data_index = kf_mesh;
    {
        Eigen::MatrixXd kf_centers(state.cage.num_keyframes(), 3);
        Eigen::MatrixXd kf_centroids(state.cage.num_keyframes(), 3);
        int count = 0;
        for (BoundingCage::KeyFrame& kf : state.cage.keyframes) {
            kf_centers.row(count) = kf.origin();
            kf_centroids.row(count) = kf.centroid_3d();

            Eigen::MatrixXd p1(2, 3), p2(2, 3);
            Eigen::MatrixXd c(2, 3);
            c.row(0) = ColorRGB::RED;
            c.row(0) = ColorRGB::GREEN;
            p1.row(0) = kf.origin(); p2.row(0) = kf.origin() + 10.0*kf.up_rotated_3d();
            p1.row(1) = kf.origin(); p2.row(1) = kf.origin() + 10.0*kf.right_rotated_3d();

            viewer->data().add_edges(p1, p2, c);
            Eigen::MatrixXd bboxv = kf.bounding_box_vertices_3d();
            viewer->data().add_points(bboxv, ColorRGB::CYAN);
            count += 1;
        }

        viewer->data().point_size = 10.0;
        viewer->data().line_width = 2.0;
        viewer->data().add_points(kf_centroids, ColorRGB::MAGENTA);
        viewer->data().add_points(kf_centers, ColorRGB::STEEL_BLUE);
    }

    size_t cell_mesh = viewer->append_mesh() - 1;
    viewer->selected_data_index = cell_mesh;
    viewer->data().set_mesh(state.cage.mesh_vertices(), state.cage.mesh_faces());
    viewer->data().show_faces = false;
    viewer->data().line_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    viewer->data().line_width = 2.0;

    size_t pts_mesh = viewer->append_mesh() - 1;
    viewer->selected_data_index = pts_mesh;
    {
        Eigen::MatrixXd C;
        double min_z = state.dilated_tet_mesh.geodesic_dists.minCoeff();
        double max_z = state.dilated_tet_mesh.geodesic_dists.maxCoeff();
        igl::colormap(igl::COLOR_MAP_TYPE_PARULA, state.dilated_tet_mesh.geodesic_dists, min_z, max_z, C);

        viewer->data().add_points(TV, C);
        viewer->data().point_size = 4.0;
    }

    debug.drew_debug_state = true;
}


bool EndPoint_Selection_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    viewer->core.viewport = Eigen::RowVector4f(view_hsplit*window_width, 0, (1.0-view_hsplit)*window_width, window_height);

    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiSetCond_Always);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);
    ImGui::SetNextWindowSize(ImVec2(w * view_hsplit, h), ImGuiSetCond_Always);
    ImGui::Begin("Select Endpoints", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);

    if (done_extracting_skeleton) {
        if (debug.enabled) {
            debug_draw_intermediate_state();
            if (ImGui::Button("COOL?")) {
                state.set_application_state(Application_State::BoundingPolygon);
            }
        }
    }

    if (extracting_skeleton) {
        ImGui::OpenPopup("Extracting Skeleton");
        ImGui::BeginPopupModal("Extracting Skeleton");
        ImGui::Text("Extracting Fish Skeleton. Please wait, this may take a few seconds.");
        ImGui::NewLine();
        ImGui::EndPopup();
    }

    if (bad_selection) {
        ImGui::OpenPopup("Invalid Endpoint Selection");
        ImGui::BeginPopupModal("Invalid Endpoint Selection");
        ImGui::Text("%s", bad_selection_error_message.c_str());
        ImGui::NewLine();
        ImGui::Separator();
        if (ImGui::Button("OK")) {
            bad_selection = false;
        }
        ImGui::EndPopup();
    }

    std::string button_text("New Endpoint Pair");
    if (selecting_endpoints) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

        if (current_endpoint_idx == 0) {
            button_text = std::string("Select Front");
        }
        else if (current_endpoint_idx == 1) {
            button_text = std::string("Select Back");
        }
        else {
            assert(false);
        }
    }

    int num_digits_ep = !state.endpoint_pairs.empty() ?
        static_cast<int>(log10(static_cast<double>(state.endpoint_pairs.size())) + 1) :
        1;

    if (!state.endpoint_pairs.empty()) {
        ImGui::Text("Endpoint Pairs:");
        for (int i = 0; i < state.endpoint_pairs.size(); i++) {
            int num_digits_i = (i + 1) > 0 ?
                static_cast<int>(log10(static_cast<double>(i + 1)) + 1) :
                1;

            std::string label_text = "Endpoint ";
            for (int zi = 0; zi < num_digits_ep - num_digits_i; zi++) {
                label_text += std::string("0");
            }
            label_text += std::to_string(i);
            label_text += std::string(": ");
            std::string rm_button_text = std::string("Remove##") + std::to_string(i + 1);
            ImGui::BulletText("%s", label_text.c_str());
            ImGui::SameLine();
            if (ImGui::Button(rm_button_text.c_str(), ImVec2(-1, 0))) {
                assert(i < state.endpoint_pairs.size());
                state.endpoint_pairs.erase(state.endpoint_pairs.begin() + i);
            }
        }
        ImGui::NewLine();
        ImGui::Separator();
    }

    if (ImGui::Button(button_text.c_str(), ImVec2(-1, 0))) {
        selecting_endpoints = true;
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }

    if (selecting_endpoints) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();

        if (ImGui::Button("Cancel", ImVec2(-1, 0))) {
            current_endpoint_idx = 0;
            current_endpoints = { -1, -1 };
            selecting_endpoints = false;
        }
    }
    ImGui::NewLine();
    ImGui::Separator();
    if (ImGui::Button("Back")) {
        state.set_application_state(Application_State::Segmentation);
    }
    ImGui::SameLine();
    if (state.endpoint_pairs.empty()) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    if (ImGui::Button("Next")) {
        extract_skeleton();
    }
    if (state.endpoint_pairs.empty()) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }
    ImGui::End();

    ImGui::Render();
    return ret;
}

bool EndPoint_Selection_Menu::mouse_down(int button, int modifier) {
    bool ret = FishUIViewerPlugin::mouse_down(button, modifier);
    if (!selecting_endpoints) {
        return ret;
    }

    int fid;            // ID of the clicked face
    Eigen::Vector3f bc; // Barycentric coords of the click point on the face
    double x = viewer->current_mouse_x;
    double y = viewer->core.viewport(3) - viewer->current_mouse_y;

    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
            viewer->core.view * viewer->core.model,
            viewer->core.proj, viewer->core.viewport,
            state.dilated_tet_mesh.TV, state.dilated_tet_mesh.TF, fid, bc))
    {
        int max;
        bc.maxCoeff(&max);
        int vid = state.dilated_tet_mesh.TF(fid, max);
        current_endpoints[current_endpoint_idx] = vid;

        current_endpoint_idx += 1;

        if (current_endpoint_idx >= 2) { // We've selected 2 endpoints
            state.endpoint_pairs.push_back(
                std::make_pair(current_endpoints[0], current_endpoints[1]));

            if (current_endpoints[0] == current_endpoints[1]) {
                bad_selection = true;
                bad_selection_error_message = "Invalid Endpoints: Selected endpoints are the same.";
                state.endpoint_pairs.pop_back();
            }
            else if (!validate_endpoint_pairs(state.endpoint_pairs, state.dilated_tet_mesh.connected_components)) {
                bad_selection = true;
                bad_selection_error_message = "Invalid Endpoints: You can only have one endpoint pair per connected component.";
                state.endpoint_pairs.pop_back();
            }

            current_endpoints = { -1, -1 };
            current_endpoint_idx = 0;
            selecting_endpoints = false;
        }
    }

    return ret;
}


void EndPoint_Selection_Menu::extract_skeleton() {
    auto thread_fun = [&]() {
        extracting_skeleton = true;
        glfwPostEmptyEvent();

        const Eigen::MatrixXd& TV = state.dilated_tet_mesh.TV;
        const Eigen::MatrixXi& TT = state.dilated_tet_mesh.TT;
        const Eigen::VectorXi& C = state.dilated_tet_mesh.connected_components;
        const int comp = C[state.endpoint_pairs[0].first];

        Eigen::MatrixXd TV2;
        Eigen::MatrixXi TT2;
        remesh_connected_components(comp, C, TV, TT, TV2, TT2);
        Eigen::MatrixXd skeleton_vertices;

        const bool normalized = true;
        //heat_diffusion_distances(TV, TT, state.endpoint_pairs, state.dilated_tet_mesh.geodesic_dists, normalized);
        geodesic_distances(TV2, TT2, state.endpoint_pairs, state.dilated_tet_mesh.geodesic_dists, normalized);
        compute_skeleton(TV, TT, state.dilated_tet_mesh.geodesic_dists,
            state.endpoint_pairs, state.dilated_tet_mesh.connected_components,
            100, skeleton_vertices);

        const double rad = 7.5;
        Eigen::Vector4d bbox(-rad, rad, -rad, rad);
        state.cage.set_skeleton_vertices(skeleton_vertices, 50 /* smoothing iterations */, bbox);

        extracting_skeleton = false;
        done_extracting_skeleton = true;
        glfwPostEmptyEvent();
    };

    extract_skeleton_thread = std::thread(thread_fun);
    extract_skeleton_thread.detach();
}
