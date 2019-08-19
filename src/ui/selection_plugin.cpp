#include "selection_plugin.h"

#include "state.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include "utils/glm_conversion.h"

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>

#pragma optimize ("", off)

Selection_Menu::Selection_Menu(State& state) : _state(state) {}

void Selection_Menu::deinitialize() {
    double elapsed = _timer.elapsed();
    _state.timing_logger->info("END SEGMENTATION {} {} {}", elapsed, _num_key_presses, _num_mouse_clicks);
    selection_renderer.destroy();
    viewer->core.viewport = old_viewport;
}

void Selection_Menu::initialize() {
    _timer.reset();
    _num_key_presses = 0;
    _num_mouse_clicks = 0;
    _state.timing_logger->info("BEGIN SEGMENTATION");
    _state.timing_logger->info("BEGIN_INTERACT SEGMENTATION");

    selection_renderer.initialize(glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]));

    const glm::ivec3 volume_dims = G3i(_state.low_res_volume.dims());
    rendering_params.volume_dimensions = volume_dims;

    number_features_is_dirty = false;
    selection_list_is_dirty = false;
    {
        uint32_t* buffer_data = _state.segmented_features.buffer_data.data();
        size_t num_features =_state.segmented_features.buffer_data.size();
        selection_renderer.set_contour_data(buffer_data, num_features);

        std::vector<uint32_t> selected = _state.segmented_features.selected_features;
        selected.insert(selected.begin(), static_cast<uint32_t>(selected.size()));
        selection_renderer.set_selection_data(selected.data(), selected.size());
    }

    target_viewport_size = { -1.f, -1.f, -1.f, -1.f };

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);

    const int maxDim = glm::compMax(rendering_params.volume_dimensions);
    const float md = static_cast<float>(maxDim);
    const glm::vec3 normalized_volume_dims = {
      rendering_params.volume_dimensions[0] / md,
      rendering_params.volume_dimensions[1] / md,
      rendering_params.volume_dimensions[2] / md
    };
    double w = normalized_volume_dims[0]/2.0, h = normalized_volume_dims[1]/2.0, d = normalized_volume_dims[2]/2.0;
    Eigen::MatrixXd volume_bbox_v(8, 3);
    volume_bbox_v <<
        -w, -h, -d,
        -w, -h,  d,
        -w,  h, -d,
        -w,  h,  d,
         w, -h, -d,
         w, -h,  d,
         w,  h, -d,
         w,  h,  d;
    Eigen::MatrixXi volume_bbox_i(12, 3);
    volume_bbox_i <<
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
        1, 7, 3;
    viewer->core.align_camera_center(volume_bbox_v, volume_bbox_i);

    if (transfer_function.empty()) {
        // Create the initial nodes
        TfNode first = { 0.f, { 0.f, 0.f, 0.f, 0.f } };
        TfNode last = { 1.f, { 1.f, 1.f, 1.f, 1.f } };
        transfer_function.push_back(std::move(first));
        transfer_function.push_back(std::move(last));
        transfer_function_dirty = true;
    }

    old_viewport = viewer->core.viewport;

    Eigen::RowVector4f viewport(view_hsplit*window_width, 0, (1.0-view_hsplit)*window_width, window_height);
    viewer->core.viewport = viewport;
}

void Selection_Menu::draw_selection_volume() {
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    Eigen::RowVector4f viewport(view_hsplit*window_width, 0, (1.0-view_hsplit)*window_width, window_height);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    viewer->core.viewport = viewport;

    if (viewer->core.viewport != target_viewport_size) {
        selection_renderer.resize_framebuffer(
                    glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]));
        target_viewport_size = G4f(viewer->core.viewport);
    }

    if (number_features_is_dirty) {
        selection_list_is_dirty = true;
        _state.segmented_features.recompute_feature_map();

        uint32_t* buffer_data = _state.segmented_features.buffer_data.data();
        size_t num_features =_state.segmented_features.buffer_data.size();
        selection_renderer.set_contour_data(buffer_data, num_features);
        number_features_is_dirty = false;
        _state.dirty_flags.mesh_dirty = true;
    }

    if (selection_list_is_dirty) {
        std::vector<uint32_t> selected = _state.segmented_features.selected_features;
        selected.insert(selected.begin(), static_cast<uint32_t>(selected.size()));
        selection_renderer.set_selection_data(selected.data(), selected.size());
        selection_list_is_dirty = false;
        _state.dirty_flags.mesh_dirty = true;
    }

    if (transfer_function_dirty) {
        selection_renderer.set_transfer_function(transfer_function);
        transfer_function_dirty = false;
    }

    rendering_params.sampling_rate = 1.0 / glm::length(glm::vec3(rendering_params.volume_dimensions));
    rendering_params.light_position = G3f(viewer->core.light_position);
    rendering_params.highlight_factor = highlight_factor;
    rendering_params.emphasize_by_selection = static_cast<int>(emphasize_by_selection);
    rendering_params.color_by_id = color_by_id;

    const int maxDim = glm::compMax(rendering_params.volume_dimensions);
    const float md = static_cast<float>(maxDim);
    const glm::vec3 normalized_volume_dims = {
      rendering_params.volume_dimensions[0] / md,
      rendering_params.volume_dimensions[1] / md,
      rendering_params.volume_dimensions[2] / md
    };
    glm::mat4 scaling = glm::scale(glm::mat4(1.f), normalized_volume_dims);
    glm::mat4 translate = glm::translate(glm::mat4(1.f), glm::vec3(-0.5f));
    glm::mat4 model = GM4f(viewer->core.model) * scaling * translate;
    glm::mat4 view = GM4f(viewer->core.view);
    glm::mat4 proj = GM4f(viewer->core.proj);
    selection_renderer.geometry_pass(model, view, proj);
    selection_renderer.volume_pass(
                rendering_params,
                _state.low_res_volume.index_texture,
                _state.low_res_volume.volume_texture);

    glm::ivec2 inv_mouse_coords { viewer->current_mouse_x, viewer->core.viewport[3] - viewer->current_mouse_y };
    glm::vec3 picking = selection_renderer.picking_pass(
                rendering_params,
                inv_mouse_coords,
                _state.low_res_volume.index_texture,
                _state.low_res_volume.volume_texture);
    current_selected_feature = static_cast<int>(picking.x);

    if (should_select) {
        if (current_selected_feature != 0) {
            auto update = [&](std::vector<uint32_t>& indices) {
                assert(std::is_sorted(indices.begin(), indices.end()));
                auto it = std::lower_bound(indices.begin(), indices.end(),
                    static_cast<uint32_t>(current_selected_feature));

                if (it == indices.end()) {
                    // The index was not found
                    indices.push_back(current_selected_feature);
                }
                else if (*it == current_selected_feature) {
                    // We found the feature
                    indices.erase(it);
                }
                else {
                    // We did not find the feature
                    indices.insert(it, current_selected_feature);
                }
                assert(std::is_sorted(indices.begin(), indices.end()));
            };

            update(_state.segmented_features.selected_features);

            selection_list_is_dirty = true;
        }

        should_select = false;
    }

    glViewport(0, 0, window_width, window_height);
}

bool Selection_Menu::key_down(int key, int modifiers) {
    _num_key_presses += 1;
    if (key == 32) { // SPACE
        should_select = true;
        return true;
    }
    return false;
}

bool Selection_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

    draw_selection_volume();

    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImVec2(w * view_hsplit, h), ImGuiSetCond_Always);
    ImGui::Begin("Select Segments", nullptr,
        //ImGuiWindowFlags_NoSavedSettings |
        //ImGuiWindowFlags_AlwaysAutoResize);
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    if (show_error_popup) {
        ImGui::OpenPopup(error_title.c_str());
        ImGui::BeginPopupModal(error_title.c_str());
        ImGui::Text("%s", error_message.c_str());
        ImGui::NewLine();
        ImGui::Separator();
        if (ImGui::Button("OK")) {
            show_error_popup = false;
        }
        ImGui::EndPopup();
    }

    ImGui::Text("Number of features:");
    ImGui::PushItemWidth(-1);
    if (ImGui::SliderInt("##numfeatures", &_state.segmented_features.num_selected_features, 1, 100)) {
        number_features_is_dirty = true;
    }
    ImGui::PopItemWidth();
    ImGui::NewLine();
    ImGui::Separator();

    std::string list = std::accumulate(
        _state.segmented_features.selected_features.begin(),
        _state.segmented_features.selected_features.end(), std::string(),
        [](std::string s, int i) { return s + std::to_string(i) + ", "; });
    // Remove the last ", "
    list = list.substr(0, list.size() - 2);

    ImGui::Text("Selected features: %s", list.c_str());
    ImGui::PushItemWidth(-1);
    if (ImGui::Button("Clear Selected Features", ImVec2(-1, 0))) {
        _state.segmented_features.selected_features.clear();
        selection_list_is_dirty = true;
    }
    ImGui::PopItemWidth();

    ImGui::NewLine();
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Advanced", nullptr, ImGuiTreeNodeFlags(0))) {
        float dilation_amt = (float)_state.dilated_tet_mesh.dilation_radius;
        float voxel_width = (float)_state.dilated_tet_mesh.meshing_voxel_radius;
        ImGui::Spacing();
        ImGui::Text("Meshing Dilation Amount:");
        ImGui::PushItemWidth(-1);
        if (ImGui::InputFloat("##dilationamt", &dilation_amt, 0.5, 1.0)) {
            _state.dilated_tet_mesh.dilation_radius = std::max((double)dilation_amt, 1.0);
            _state.dirty_flags.mesh_dirty = true;
        }
        ImGui::PopItemWidth();

        ImGui::Spacing();
        ImGui::Text("Meshing Voxel Width:");
        ImGui::PushItemWidth(-1);
        if (ImGui::InputFloat("##voxelwidth", &voxel_width, 0.1, 0.2)) {
            _state.dilated_tet_mesh.meshing_voxel_radius = std::max((double)voxel_width, 0.1);
            _state.dirty_flags.mesh_dirty = true;
        }
        ImGui::PopItemWidth();
    }
    ImGui::NewLine();
    ImGui::Separator();

    if (ImGui::Button("Back")) {
        _state.timing_logger->info("END_INTERACT SEGMENTATION {}", _timer.elapsed());
        _state.set_application_state(Application_State::Initial_File_Selection);
    }
    ImGui::SameLine();
    if (list.empty()) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    if (ImGui::Button("Next")) {
        _state.timing_logger->info("END_INTERACT SEGMENTATION {}", _timer.elapsed());
        _state.set_application_state(Application_State::Meshing);
    }
    if (list.empty()) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    ImGui::End();
    ImGui::Render();
    return ret;
}
