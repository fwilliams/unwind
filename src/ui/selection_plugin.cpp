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

void Selection_Menu::initialize() {
    volumerendering::initialize(_state.volume_rendering,
        glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]),
        ContourTreeFragmentShader, ContourTreePickingFragmentShader);

    _gl_state.uniform_locations_rendering.index_volume = glGetUniformLocation(
        _state.volume_rendering.program.program_object, "index_volume"
    );
    _gl_state.uniform_locations_rendering.color_by_identifier = glGetUniformLocation(
        _state.volume_rendering.program.program_object, "color_by_identifier"
    );
    _gl_state.uniform_locations_rendering.selection_emphasis_type = glGetUniformLocation(
        _state.volume_rendering.program.program_object, "selection_emphasis_type"
    );
    _gl_state.uniform_locations_rendering.highlight_factor = glGetUniformLocation(
        _state.volume_rendering.program.program_object, "highlight_factor"
    );
    _gl_state.uniform_locations_picking.index_volume = glGetUniformLocation(
        _state.volume_rendering.picking_program.program_object, "index_volume"
    );

    // SSBO
    glGenBuffers(1, &_gl_state.contour_information_ssbo);
    glGenBuffers(1, &_gl_state.selection_list_ssbo);

    _state.volume_rendering.parameters.volume_dimensions = {
        _state.volume_file.w, _state.volume_file.h, _state.volume_file.d
    };
    _state.volume_rendering.parameters.volume_dimensions_rcp =
        glm::vec3(1.f) / glm::vec3(_state.volume_rendering.parameters.volume_dimensions);

    int maxDim = glm::compMax(_state.volume_rendering.parameters.volume_dimensions);
    float md = static_cast<float>(maxDim);

    _state.volume_rendering.parameters.normalized_volume_dimensions = {
      _state.volume_rendering.parameters.volume_dimensions[0] / md,
      _state.volume_rendering.parameters.volume_dimensions[1] / md,
      _state.volume_rendering.parameters.volume_dimensions[2] / md
    };

    load_rawfile(_state.volume_base_name + ".raw",
        E3i(_state.volume_rendering.parameters.volume_dimensions), _state.volume_data, true);
    volumerendering::upload_volume_data(_state.volume_rendering.volume_texture,
        _state.volume_rendering.parameters.volume_dimensions,
        _state.volume_data.data(), _state.volume_data.size());

    // Index volume
    glGenTextures(1, &_state.index_volume);
    glBindTexture(GL_TEXTURE_3D, _state.index_volume);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    const int voxel = _state.volume_file.w * _state.volume_file.h * _state.volume_file.d;
    const int num_bytes = voxel * sizeof(uint32_t);

    //std::vector<char> data(num_bytes);

    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open(_state.volume_base_name + ".part.raw", std::ifstream::binary);

    _state.index_volume_data.resize(num_bytes / 4);
    file.read(reinterpret_cast<char*>(_state.index_volume_data.data()), num_bytes);

    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, _state.volume_file.w, _state.volume_file.h,
        _state.volume_file.d, 0, GL_RED_INTEGER, GL_UNSIGNED_INT,
        reinterpret_cast<char*>(_state.index_volume_data.data()));
    glBindTexture(GL_TEXTURE_3D, 0);

    _state.fishes.resize(1);
    _state.current_fish = 0;
}

void Selection_Menu::resize_framebuffer_textures(glm::ivec2 framebuffer_size) {
    // Entry point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, _state.volume_rendering.bounding_box.entry_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, framebuffer_size.x, framebuffer_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);

    // Exit point texture and frame buffer
    glBindTexture(GL_TEXTURE_2D, _state.volume_rendering.bounding_box.exit_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, framebuffer_size.x, framebuffer_size.y, 0,
        GL_RGBA, GL_FLOAT, nullptr);
}

void Selection_Menu::draw_setup() {
    if (viewer->core.viewport != _state.target_viewport_size) {
        resize_framebuffer_textures(
            glm::ivec2(viewer->core.viewport[2], viewer->core.viewport[3]));
        _state.target_viewport_size = G4f(viewer->core.viewport);
    }

    if (number_features_is_dirty) {
        total_selection_list.clear();
        for (State::Fish_Status& s : _state.fishes) {
            s.feature_list.clear();
        }
        total_selection_list.clear();
        selection_list_is_dirty = true;

        std::vector<contourtree::Feature> features =
            _state.topological_features.getFeatures(_state.num_features, 0.f);

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

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.contour_information_ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint32_t) * buffer_data.size(),
            buffer_data.data(), GL_DYNAMIC_READ);

        number_features_is_dirty = false;
    }

    if (selection_list_is_dirty) {
        std::vector<uint32_t> selected = _state.fishes[_state.current_fish].feature_list;
        selected.insert(selected.begin(), static_cast<int>(selected.size()));

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.selection_list_ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint32_t) * selected.size(),
            selected.data(), GL_DYNAMIC_READ);

        selection_list_is_dirty = false;
    }
}

void Selection_Menu::draw() {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (_state.volume_rendering.transfer_function.is_dirty) {
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, "Update Transfer Function");
        update_transfer_function(_state.volume_rendering.transfer_function);
        _state.volume_rendering.transfer_function.is_dirty = false;
        glPopDebugGroup();
    }


    render_bounding_box(_state.volume_rendering, GM4f(viewer->core.model),
        GM4f(viewer->core.view), GM4f(viewer->core.proj));

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(_state.volume_rendering.program.program_object);

    // The default volume renderer already uses GL_TEXTURE0 through GL_TEXTURE3
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, _state.index_volume);
    glUniform1i(_gl_state.uniform_locations_rendering.index_volume, 4);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.contour_information_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _gl_state.contour_information_ssbo);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _gl_state.selection_list_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _gl_state.selection_list_ssbo);

    glUniform1i(_gl_state.uniform_locations_rendering.color_by_identifier,
        color_by_id ? 1 : 0);

    glUniform1i(_gl_state.uniform_locations_rendering.selection_emphasis_type,
        static_cast<int>(emphasize_by_selection));

    glUniform1f(_gl_state.uniform_locations_rendering.highlight_factor, highlight_factor);

    render_volume(_state.volume_rendering, G3f(viewer->core.light_position));

    glUseProgram(_state.volume_rendering.picking_program.program_object);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, _state.index_volume);
    glUniform1i(_gl_state.uniform_locations_picking.index_volume, 4);

    glm::vec3 picking = pick_volume_location(_state.volume_rendering,
        { viewer->current_mouse_x, viewer->core.viewport[3] - viewer->current_mouse_y });

    glUseProgram(0);

    current_selected_feature = static_cast<int>(picking.x);

    if (should_select) {
        if (current_selected_feature != 0) {
            auto update = [f = current_selected_feature](std::vector<uint32_t>& indices) {
                assert(std::is_sorted(indices.begin(), indices.end()));
                auto it = std::lower_bound(indices.begin(), indices.end(),
                    static_cast<uint32_t>(f));

                if (it == indices.end()) {
                    // The index was not found
                    indices.push_back(f);
                }
                else if (*it == f) {
                    // We found the feature
                    indices.erase(it);
                }
                else {
                    // We did not find the feature
                    indices.insert(it, f);
                }
                assert(std::is_sorted(indices.begin(), indices.end()));
            };

            update(total_selection_list);
            update(_state.fishes[_state.current_fish].feature_list);

            selection_list_is_dirty = true;
        }

        should_select = false;
    }
}

void Selection_Menu::key_down(unsigned int key, int modifiers) {
    if (key == 32) { // SPACE
        should_select = true;
    }
}

bool Selection_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

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
    if (ImGui::SliderInt("Number of features", &_state.num_features, 1, 100)) {
        number_features_is_dirty = true;
    }
    ImGui::NewLine();
    ImGui::Separator();

    std::string list = std::accumulate(
        _state.fishes[_state.current_fish].feature_list.begin(),
        _state.fishes[_state.current_fish].feature_list.end(), std::string(),
        [](std::string s, int i) { return s + std::to_string(i) + ", "; });
    // Remove the last ", "
    list = list.substr(0, list.size() - 2);

    ImGui::Text("Selected features: %s", list.c_str());

    if (ImGui::Button("Clear Selected Features", ImVec2(-1, 0))) {
        total_selection_list.clear();
    }
    ImGui::NewLine();

    ImGui::Separator();
    ImGui::PushItemWidth(-1);
    ImGui::Text("Current fish: %ld / %ld", _state.current_fish + 1,
        _state.fishes.size());
    bool pressed_prev = ImGui::Button("< Prev Fish");
    ImGui::SameLine();
    bool pressed_next = ImGui::Button("Next Fish >");
    ImGui::SameLine();
    bool pressed_delete = ImGui::Button("Delete Fish");

    if (pressed_prev) {
        bool is_fish_empty = _state.fishes[_state.current_fish].feature_list.empty();
        bool is_in_last_fish = _state.current_fish == _state.fishes.size() - 1;
        if (is_fish_empty && is_in_last_fish) {
            pressed_delete = true;
        }
        else {
            _state.current_fish = std::max(size_t(0), _state.current_fish - 1);
        }
        selection_list_is_dirty = true;
    }
    if (pressed_next) {
        _state.current_fish++;
        // We reached the end
        if (_state.current_fish == _state.fishes.size()) {
            _state.fishes.push_back({});
        }

        selection_list_is_dirty = true;
    }

    // We don't want to delete the last fish
    if (pressed_delete && _state.fishes.size() > 1) {
        _state.fishes.erase(_state.fishes.begin() + _state.current_fish);
        _state.current_fish = std::max(size_t(0), _state.current_fish - 1);

        selection_list_is_dirty = true;
    }

    ImGui::NewLine();
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

            ImGui::Text("Frame Counter: %lu", _state.frame_counter);
        }

        ImGui::Text("%s", "Transfer Function");

        constexpr const float Radius = 10.f;

        volumerendering::Transfer_Function& tf = _state.volume_rendering.transfer_function;

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
