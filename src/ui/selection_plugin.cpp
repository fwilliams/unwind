#include "selection_plugin.h"

#include "state.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include <utils/volume_rendering.h>

#include "volume_fragment_shader.h"
#include "picking_fragment_shader.h"

#include "utils/glm_conversion.h"

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>

#pragma optimize ("", off)

Selection_Menu::Selection_Menu(State& state) : _state(state) {}

void Selection_Menu::deinitialize() {
    volume_rendering.destroy();
}

void Selection_Menu::initialize() {
    volume_rendering.initialize(
        glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]),
        ContourTreeFragmentShader, ContourTreePickingFragmentShader);

    volume_rendering._gl_state.uniform_locations_rendering.index_volume = glGetUniformLocation(
        volume_rendering.program.program_object, "index_volume"
    );
    volume_rendering._gl_state.uniform_locations_rendering.color_by_identifier = glGetUniformLocation(
        volume_rendering.program.program_object, "color_by_identifier"
    );
    volume_rendering._gl_state.uniform_locations_rendering.selection_emphasis_type = glGetUniformLocation(
        volume_rendering.program.program_object, "selection_emphasis_type"
    );
    volume_rendering._gl_state.uniform_locations_rendering.highlight_factor = glGetUniformLocation(
        volume_rendering.program.program_object, "highlight_factor"
    );
    volume_rendering._gl_state.uniform_locations_picking.index_volume = glGetUniformLocation(
        volume_rendering.picking_program.program_object, "index_volume"
    );

    // SSBO
    glGenBuffers(1, &volume_rendering._gl_state.contour_information_ssbo);
    glGenBuffers(1, &volume_rendering._gl_state.selection_list_ssbo);

    const glm::ivec3 volume_dims = G3i(_state.low_res_volume.dims());
    volume_rendering.parameters.volume_dimensions = volume_dims;
    volume_rendering.parameters.volume_dimensions_rcp = glm::vec3(1.f) / glm::vec3(volume_dims);

    const int maxDim = glm::compMax(volume_rendering.parameters.volume_dimensions);
    const float md = static_cast<float>(maxDim);

    const glm::vec3 normalized_volume_dims = {
      volume_rendering.parameters.volume_dimensions[0] / md,
      volume_rendering.parameters.volume_dimensions[1] / md,
      volume_rendering.parameters.volume_dimensions[2] / md
    };
    volume_rendering.parameters.normalized_volume_dimensions = normalized_volume_dims;

    _state.selected_features.clear();
    number_features_is_dirty = true;
    selection_list_is_dirty = false;
    target_viewport_size = { -1.f, -1.f, -1.f, -1.f };

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

    volume_rendering.parameters.sampling_rate = 1.0 / glm::length(glm::vec3(volume_rendering.parameters.volume_dimensions));
}

void Selection_Menu::resize_framebuffer_textures(glm::ivec2 framebuffer_size) {
    // Entry point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, volume_rendering.bounding_box.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, framebuffer_size.x, framebuffer_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);

    // Exit point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, volume_rendering.bounding_box.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, framebuffer_size.x, framebuffer_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);
}

bool Selection_Menu::pre_draw() {
    bool ret = FishUIViewerPlugin::pre_draw();

    return ret;
}

void Selection_Menu::draw_selection_volume() {
    if (viewer->core.viewport != target_viewport_size) {
        resize_framebuffer_textures(
            glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]));
        target_viewport_size = G4f(viewer->core.viewport);
    }

    if (number_features_is_dirty) {
        _state.selected_features.clear();
        selection_list_is_dirty = true;

        std::vector<contourtree::Feature> features =
            _state.topological_features.getFeatures(_state.num_selected_features, 0.f);

        uint32_t size = _state.topological_features.ctdata.noArcs;

        // Buffer contents:
        // [0]: number of features
        // [...]: A linearized map from voxel identifier -> feature number
        std::vector<uint32_t> buffer_data(size + 1 + 1, static_cast<uint32_t>(-1));
        buffer_data[0] = static_cast<uint32_t>(features.size());
        for (size_t i = 0; i < features.size(); ++i) {
            for (uint32_t j : features[i].arcs) {
                // +1 since the first value of the vector contains the number of features
                buffer_data[j + 1] = static_cast<uint32_t>(i);
            }
        }
        volume_rendering.set_contour_data(buffer_data.data(), buffer_data.size());
        number_features_is_dirty = false;
    }

    if (selection_list_is_dirty) {
        std::vector<uint32_t> selected = _state.selected_features;
        selected.insert(selected.begin(), static_cast<int>(selected.size()));
        volume_rendering.set_selection_data(selected.data(), selected.size());
        selection_list_is_dirty = false;
    }

    if (volume_rendering.transfer_function.is_dirty) {
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Update Transfer Function");
        update_transfer_function(volume_rendering.transfer_function);
        volume_rendering.transfer_function.is_dirty = false;
        glPopDebugGroup();
    }

    volume_rendering.parameters.light_position = G3f(viewer->core.light_position);
    volume_rendering.parameters.highlight_factor = highlight_factor;
    volume_rendering.parameters.emphasize_by_selection = static_cast<int>(emphasize_by_selection);
    volume_rendering.parameters.color_by_id = color_by_id;

    volume_rendering.render_bounding_box(GM4f(viewer->core.model), GM4f(viewer->core.view), GM4f(viewer->core.proj));
    volume_rendering.render_volume(
                _state.low_res_volume.index_texture,
                _state.low_res_volume.volume_texture);

    glm::ivec2 inv_mouse_coords { viewer->current_mouse_x, viewer->core.viewport[3] - viewer->current_mouse_y };
    glm::vec3 picking = volume_rendering.pick_volume_location(
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

            update(_state.selected_features);

            selection_list_is_dirty = true;
        }

        should_select = false;
    }
}

bool Selection_Menu::key_down(int key, int modifiers) {
    if (key == 32) { // SPACE
        should_select = true;
    }
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
    ImGui::SetNextWindowSize(ImVec2(w * 0.2f, h), ImGuiSetCond_Always);
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
    if (ImGui::SliderInt("Number of features", &_state.num_selected_features, 1, 100)) {
        number_features_is_dirty = true;
    }
    ImGui::NewLine();
    ImGui::Separator();

    std::string list = std::accumulate(
        _state.selected_features.begin(),
        _state.selected_features.end(), std::string(),
        [](std::string s, int i) { return s + std::to_string(i) + ", "; });
    // Remove the last ", "
    list = list.substr(0, list.size() - 2);

    ImGui::Text("Selected features: %s", list.c_str());

    if (ImGui::Button("Clear Selected Features", ImVec2(-1, 0))) {
        _state.selected_features.clear();
        selection_list_is_dirty = true;
    }
//    ImGui::NewLine();

//    ImGui::Separator();
//    ImGui::PushItemWidth(-1);
//    ImGui::Text("Current fish: %ld / %ld", _state.current_fish + 1,
//        _state.fishes.size());
//    bool pressed_prev = ImGui::Button("< Prev Fish");
//    ImGui::SameLine();
//    bool pressed_next = ImGui::Button("Next Fish >");
//    ImGui::SameLine();
//    bool pressed_delete = ImGui::Button("Delete Fish");

//    if (pressed_prev) {
//        bool is_fish_empty = _state.selected_features.empty();
//        bool is_in_last_fish = _state.current_fish == _state.fishes.size() - 1;
//        if (is_fish_empty && is_in_last_fish) {
//            pressed_delete = true;
//        }
//        else {
//            _state.current_fish = std::max(size_t(0), _state.current_fish - 1);
//        }
//        selection_list_is_dirty = true;
//    }
//    if (pressed_next) {
//        _state.current_fish++;
//        // We reached the end
//        if (_state.current_fish == _state.fishes.size()) {
//            _state.fishes.push_back({});
//        }

//        selection_list_is_dirty = true;
//    }

//    // We don't want to delete the last fish
//    if (pressed_delete && _state.fishes.size() > 1) {
//        _state.fishes.erase(_state.fishes.begin() + _state.current_fish);
//        _state.current_fish = std::max(size_t(0), _state.current_fish - 1);

//        selection_list_is_dirty = true;
//    }

//    ImGui::NewLine();

    ImGui::Separator();

    ImGui::NewLine();
    if (ImGui::Button("Back")) {
        // TODO: Back button
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

        volumerendering::Transfer_Function& tf = volume_rendering.transfer_function;

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
        for (size_t i = 0; i < tf.nodes.size(); ++i) {
            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

            const float x = canvas_pos.x + canvas_size.x * node.t;
            const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

            if (i > 0) {
                volumerendering::Transfer_Function::Node& prev_node = tf.nodes[i - 1];

                const float prev_x = canvas_pos.x + canvas_size.x * prev_node.t;
                const float prev_y = canvas_pos.y + canvas_size.y * (1.f - prev_node.rgba[3]);
                draw_list->AddLine(ImVec2(prev_x, prev_y), ImVec2(x, y),
                    IM_COL32(255, 255, 255, 255));
            }
        }

        for (size_t i = 0; i < tf.nodes.size(); ++i) {
            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

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
                for (size_t i = 0; i < tf.nodes.size(); ++i) {
                    volumerendering::Transfer_Function::Node& node = tf.nodes[i];
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
                    const bool is_last = (current_interaction_index == tf.nodes.size() - 1);
                    if (!is_first && !is_last) {
                        tf.nodes[current_interaction_index].t = new_t;
                    }
                    tf.nodes[current_interaction_index].rgba[3] = new_a;

                    using N = volumerendering::Transfer_Function::Node;
                    std::sort(tf.nodes.begin(), tf.nodes.end(),
                        [](const N& lhs, const N& rhs) { return lhs.t < rhs.t; });
                    tf.is_dirty = true;
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

                        for (size_t i = 0; i < tf.nodes.size(); ++i) {
                            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

                            if (node.t > t) {
                                volumerendering::Transfer_Function::Node& prev = tf.nodes[i - 1];

                                const float t_prime = (t - prev.t) / (node.t - prev.t);

                                const float r = prev.rgba[0] * (1.f - t_prime) +
                                    node.rgba[0] * t_prime;
                                const float g = prev.rgba[1] * (1.f - t_prime) +
                                    node.rgba[1] * t_prime;
                                const float b = prev.rgba[2] * (1.f - t_prime) +
                                    node.rgba[2] * t_prime;
                                const float a = prev.rgba[3] * (1.f - t_prime) +
                                    node.rgba[3] * t_prime;

                                tf.nodes.insert(
                                    tf.nodes.begin() + i,
                                    { t, { r, g, b, a } }
                                );
                                has_added_node_since_initial_click = true;
                                tf.is_dirty = true;
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
            const bool is_last = current_interaction_index == tf.nodes.size() - 1;
            if (!is_first && !is_last) {
                tf.nodes.erase(tf.nodes.begin() + current_interaction_index);
                current_interaction_index = -1;
                tf.is_dirty = true;
            }
        }

        ImGui::PushItemWidth(-1);
        if (current_interaction_index >= 1 &&
            current_interaction_index <= tf.nodes.size() - 1)
        {
            float* rgba = glm::value_ptr(tf.nodes[current_interaction_index].rgba);
            if (ImGui::ColorPicker4("Change Color", rgba)) {
                tf.is_dirty = true;
            }
        }
        else {
            float rgba[4];
            ImGui::ColorPicker4("Change Color", rgba);
        }
    }
    ImGui::End();
    ImGui::Render();
    return ret;
}
