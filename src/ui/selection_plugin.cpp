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
    selection_renderer.destroy();
    viewer->core.viewport = old_viewport;
}

void Selection_Menu::initialize() {
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
    if (ImGui::SliderInt("Number of features", &_state.segmented_features.num_selected_features, 1, 100)) {
        number_features_is_dirty = true;
    }
    ImGui::NewLine();
    ImGui::Separator();

    std::string list = std::accumulate(
        _state.segmented_features.selected_features.begin(),
        _state.segmented_features.selected_features.end(), std::string(),
        [](std::string s, int i) { return s + std::to_string(i) + ", "; });
    // Remove the last ", "
    list = list.substr(0, list.size() - 2);

    ImGui::Text("Selected features: %s", list.c_str());

    if (ImGui::Button("Clear Selected Features", ImVec2(-1, 0))) {
        _state.segmented_features.selected_features.clear();
        selection_list_is_dirty = true;
    }

    ImGui::NewLine();
    ImGui::Separator();
    if (ImGui::Button("Back")) {
        _state.set_application_state(Application_State::Initial_File_Selection);
    }
    ImGui::SameLine();
    if (list.empty()) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    if (ImGui::Button("Next")) {
        _state.set_application_state(Application_State::Meshing);
    }
    if (list.empty()) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    /*
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Advanced", nullptr, ImGuiTreeNodeFlags(0))) {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 15.f);
        ImGui::Checkbox("Color by feature id", &color_by_id);

        ImGui::Text("%s", "Highlight Factor: ");
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 15.f);
        ImGui::SliderFloat("Highlight Factor", &highlight_factor, 0.f, 1.f);

        int selection_emphasis = static_cast<int>(emphasize_by_selection);
        const char* const items[] = {
          "None",
          "Highlight Selected",
          "Highlight Deselected"
        };

        bool changed = ImGui::Combo("Selection", &selection_emphasis, items, 3);
        if (changed) {
            emphasize_by_selection = static_cast<Emphasis>(selection_emphasis);
        }

        if (_state.Debugging) {
            ImGui::Text("%s", "Debugging Status");

            if (current_selected_feature == 0) {
                ImGui::Text("Current highlighted id: %s", "none");
            }
            else {
                ImGui::Text("Current highlighted id: %i", current_selected_feature);
            }
        }

        ImGui::Text("%s", "Transfer Function");

        constexpr const float Radius = 10.f;

        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 1.5f);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        //ImVec2 canvas_size = { 640.f, 150.f };

        float aspect_ratio = 150.f / 200.f; // height / width
        float canvas_width = 0.9f * ImGui::GetContentRegionAvailWidth();
        float canvas_height = aspect_ratio * canvas_width;
        ImVec2 canvas_size = { canvas_width, canvas_height };

        draw_list->AddRectFilledMultiColor(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255),
            IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
        draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x,
            canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));
        ImGui::InvisibleButton("canvas", canvas_size);

        // First render the lines
        for (size_t i = 0; i < transfer_function.size(); ++i) {
            TfNode& node = transfer_function[i];

            const float x = canvas_pos.x + canvas_size.x * node.t;
            const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

            if (i > 0) {
                TfNode& prev_node = transfer_function[i - 1];

                const float prev_x = canvas_pos.x + canvas_size.x * prev_node.t;
                const float prev_y = canvas_pos.y + canvas_size.y * (1.f - prev_node.rgba[3]);
                draw_list->AddLine(ImVec2(prev_x, prev_y), ImVec2(x, y),
                    IM_COL32(255, 255, 255, 255));
            }
        }

        for (size_t i = 0; i < transfer_function.size(); ++i) {
            TfNode& node = transfer_function[i];

            const float x = canvas_pos.x + canvas_size.x * node.t;
            const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

            if (i == current_interaction_index) {
                draw_list->AddCircleFilled(ImVec2(x, y), Radius * 1.5f,
                    IM_COL32(255, 255, 255, 255));
            }
            else {
                draw_list->AddCircleFilled(ImVec2(x, y), Radius,
                    IM_COL32(255, 255, 255, 255));
            }

            draw_list->AddCircleFilled(ImVec2(x, y), Radius,
                IM_COL32(node.rgba[0] * 255, node.rgba[1] * 255, node.rgba[2] * 255, 255));
        }

        // If the mouse button is pressed, we either have to add a new node or move an
        // existing one
        const bool mouse_in_tf_editor = ImGui::GetIO().MousePos.x >= canvas_pos.x &&
            ImGui::GetIO().MousePos.x <= (canvas_pos.x + canvas_size.x) &&
            ImGui::GetIO().MousePos.y >= canvas_pos.y &&
            ImGui::GetIO().MousePos.y <= (canvas_pos.y + canvas_size.y);

        if (mouse_in_tf_editor) {
            if (ImGui::IsMouseDown(0)) {
                for (size_t i = 0; i < transfer_function.size(); ++i) {
                    TfNode& node = transfer_function[i];
                    const float x = canvas_pos.x + canvas_size.x * node.t;
                    const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

                    const float dx = ImGui::GetIO().MousePos.x - x;
                    const float dy = ImGui::GetIO().MousePos.y - y;

                    const float r = sqrt(dx * dx + dy * dy);

                    if (r <= Radius * 2.5) {
                        clicked_mouse_position[0] = ImGui::GetIO().MousePos.x;
                        clicked_mouse_position[1] = ImGui::GetIO().MousePos.y;
                        is_currently_interacting = true;
                        current_interaction_index = static_cast<int>(i);
                        break;
                    }
                }

                if (is_currently_interacting) {
                    const float dx = ImGui::GetIO().MousePos.x - clicked_mouse_position[0];
                    const float dy = ImGui::GetIO().MousePos.y - clicked_mouse_position[1];

                    const float r = sqrt(dx * dx + dy * dy);

                    float new_t = (ImGui::GetIO().MousePos.x - canvas_pos.x) / canvas_size.x;
                    if (new_t < 0.f) {
                        new_t = 0.f;
                    }
                    if (new_t > 1.f) {
                        new_t = 1.f;
                    }

                    float new_a = 1.f - (ImGui::GetIO().MousePos.y - canvas_pos.y) / canvas_size.y;
                    if (new_a < 0.f) {
                        new_a = 0.f;
                    }
                    if (new_a > 1.f) {
                        new_a = 1.f;
                    }
                    // We don't want to move the first or last value
                    const bool is_first = current_interaction_index == 0;
                    const bool is_last = (current_interaction_index == transfer_function.size() - 1);
                    if (!is_first && !is_last) {
                        transfer_function[current_interaction_index].t = new_t;
                    }
                    transfer_function[current_interaction_index].rgba[3] = new_a;

                    using N = TfNode;
                    std::sort(transfer_function.begin(), transfer_function.end(),
                        [](const N& lhs, const N& rhs) { return lhs.t < rhs.t; });
                    transfer_function_dirty = true;
                }
                else {
                    current_interaction_index = -1;
                    // We want to only add one node per mouse click
                    if (!has_added_node_since_initial_click) {
                        // Didn't hit an existing node
                        const float t = (ImGui::GetIO().MousePos.x - canvas_pos.x) /
                            canvas_size.x;
                        const float a = 1.f - ((ImGui::GetIO().MousePos.y - canvas_pos.y) /
                            canvas_size.y);

                        for (size_t i = 0; i < transfer_function.size(); ++i) {
                            TfNode& node = transfer_function[i];

                            if (node.t > t) {
                                TfNode& prev = transfer_function[i - 1];

                                const float t_prime = (t - prev.t) / (node.t - prev.t);

                                const float r = prev.rgba[0] * (1.f - t_prime) +
                                    node.rgba[0] * t_prime;
                                const float g = prev.rgba[1] * (1.f - t_prime) +
                                    node.rgba[1] * t_prime;
                                const float b = prev.rgba[2] * (1.f - t_prime) +
                                    node.rgba[2] * t_prime;
                                const float a = prev.rgba[3] * (1.f - t_prime) +
                                    node.rgba[3] * t_prime;

                                transfer_function.insert(
                                    transfer_function.begin() + i,
                                    { t, { r, g, b, a } }
                                );
                                has_added_node_since_initial_click = true;
                                transfer_function_dirty = true;
                                break;
                            }
                        }
                    }
                }
            }
            else {
                clicked_mouse_position[0] = 0.f;
                clicked_mouse_position[1] = 0.f;
                is_currently_interacting = false;

                has_added_node_since_initial_click = false;
            }
        }

        if (ImGui::Button("Remove node")) {
            const bool is_first = current_interaction_index == 0;
            const bool is_last = current_interaction_index == transfer_function.size() - 1;
            if (!is_first && !is_last) {
                transfer_function.erase(transfer_function.begin() + current_interaction_index);
                current_interaction_index = -1;
                transfer_function_dirty = true;
            }
        }

        ImGui::PushItemWidth(-1);
        if (current_interaction_index >= 1 &&
            current_interaction_index <= transfer_function.size() - 1)
        {
            float* rgba = glm::value_ptr(transfer_function[current_interaction_index].rgba);
            if (ImGui::ColorPicker4("Change Color", rgba)) {
                transfer_function_dirty = true;
            }
        }
        else {
            float rgba[4];
            ImGui::ColorPicker4("Change Color", rgba);
        }
    }
    */
    ImGui::End();
    ImGui::Render();
    return ret;
}
