#include "transfer_function_edit_widget.h"
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

TransferFunctionEditWidget::TransferFunctionEditWidget() {
    TfNode n1 = {0.0, glm::vec4(0.0)};
    TfNode n2 = {1.0, glm::vec4(1.0)};
    _transfer_function.push_back(n1);
    _transfer_function.push_back(n2);
}

bool TransferFunctionEditWidget::post_draw() {
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 1.5f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    //ImVec2 canvas_size = { 640.f, 150.f };

    float aspect_ratio = _aspect_ratio;
    float canvas_width = (1.0f - 2.f*_padding_width) * ImGui::GetContentRegionAvailWidth();
    float centering_offset = 0.5 * (ImGui::GetContentRegionAvailWidth() - canvas_width);
    float canvas_height = aspect_ratio * canvas_width;
    ImVec2 canvas_size = { canvas_width, canvas_height };
    ImVec2 canvas_pos = { ImGui::GetCursorScreenPos()[0] + centering_offset, ImGui::GetCursorScreenPos()[1] };

    draw_list->AddRectFilledMultiColor(canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255),
        IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
    draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x,
        canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));
    ImGui::InvisibleButton("canvas", canvas_size);

    // First render the lines
    for (size_t i = 0; i < _transfer_function.size(); ++i) {
        TfNode& node = _transfer_function[i];

        const float x = canvas_pos.x + canvas_size.x * node.t;
        const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

        if (i > 0) {
            TfNode& prev_node = _transfer_function[i - 1];

            const float prev_x = canvas_pos.x + canvas_size.x * prev_node.t;
            const float prev_y = canvas_pos.y + canvas_size.y * (1.f - prev_node.rgba[3]);
            draw_list->AddLine(ImVec2(prev_x, prev_y), ImVec2(x, y),
                IM_COL32(255, 255, 255, 255));
        }
    }

    for (size_t i = 0; i < _transfer_function.size(); ++i) {
        TfNode& node = _transfer_function[i];

        const float x = canvas_pos.x + canvas_size.x * node.t;
        const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

        if (i == _current_interaction_index) {
            draw_list->AddCircleFilled(ImVec2(x, y), _node_radius * 1.5f,
                IM_COL32(255, 255, 255, 255));
        }
        else {
            draw_list->AddCircleFilled(ImVec2(x, y), _node_radius,
                IM_COL32(255, 255, 255, 255));
        }

        draw_list->AddCircleFilled(ImVec2(x, y), _node_radius,
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
            for (size_t i = 0; i < _transfer_function.size(); ++i) {
                TfNode& node = _transfer_function[i];
                const float x = canvas_pos.x + canvas_size.x * node.t;
                const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

                const float dx = ImGui::GetIO().MousePos.x - x;
                const float dy = ImGui::GetIO().MousePos.y - y;

                const float r = sqrt(dx * dx + dy * dy);

                if (r <= _node_radius * 2.5) {
                    _clicked_mouse_position[0] = ImGui::GetIO().MousePos.x;
                    _clicked_mouse_position[1] = ImGui::GetIO().MousePos.y;
                    _is_currently_interacting = true;
                    _current_interaction_index = static_cast<int>(i);
                    break;
                }
            }

            if (_is_currently_interacting) {
                const float dx = ImGui::GetIO().MousePos.x - _clicked_mouse_position[0];
                const float dy = ImGui::GetIO().MousePos.y - _clicked_mouse_position[1];

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
                const bool is_first = _current_interaction_index == 0;
                const bool is_last = (_current_interaction_index == _transfer_function.size() - 1);
                if (!is_first && !is_last) {
                    _transfer_function[_current_interaction_index].t = new_t;
                }
                _transfer_function[_current_interaction_index].rgba[3] = new_a;

                std::sort(_transfer_function.begin(), _transfer_function.end(), [](const TfNode& lhs, const TfNode& rhs) { return lhs.t < rhs.t; });
                _transfer_function_dirty = true;
            } else {
                _current_interaction_index = -1;
                // We want to only add one node per mouse click
                if (!_has_added_node_since_initial_click) {
                    // Didn't hit an existing node
                    const float t = (ImGui::GetIO().MousePos.x - canvas_pos.x) /
                        canvas_size.x;
                    const float a = 1.f - ((ImGui::GetIO().MousePos.y - canvas_pos.y) /
                        canvas_size.y);

                    for (size_t i = 0; i < _transfer_function.size(); ++i) {
                        TfNode& node = _transfer_function[i];

                        if (node.t > t) {
                            TfNode& prev = _transfer_function[i - 1];

                            const float t_prime = (t - prev.t) / (node.t - prev.t);

                            const float r = prev.rgba[0] * (1.f - t_prime) +
                                node.rgba[0] * t_prime;
                            const float g = prev.rgba[1] * (1.f - t_prime) +
                                node.rgba[1] * t_prime;
                            const float b = prev.rgba[2] * (1.f - t_prime) +
                                node.rgba[2] * t_prime;
                            const float a = prev.rgba[3] * (1.f - t_prime) +
                                node.rgba[3] * t_prime;

                            _transfer_function.insert(
                                _transfer_function.begin() + i,
                                { t, { r, g, b, a } }
                            );
                            _has_added_node_since_initial_click = true;
                            _transfer_function_dirty = true;
                            break;
                        }
                    }
                }
            }
        }
        else {
            _clicked_mouse_position[0] = 0.f;
            _clicked_mouse_position[1] = 0.f;
            _is_currently_interacting = false;

            _has_added_node_since_initial_click = false;
        }
    }


    ImVec2 rm_button_pos = {centering_offset, ImGui::GetCursorPosY() + 0.4f*ImGui::GetTextLineHeight()};
    ImGui::SetCursorPos(rm_button_pos);

    ImVec2 button_size = _color_edit_as_popup ? ImVec2{canvas_width*0.48f, 0.0f} : ImVec2{canvas_width, 0.0f};
    const bool is_first = _current_interaction_index == 0;
    const bool is_last = _current_interaction_index == _transfer_function.size() - 1;
    bool pushed_disabled_style = false;
    if (_current_interaction_index < 0 || is_first || is_last) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        pushed_disabled_style = true;
    }
    if (ImGui::Button("Remove Node", button_size)) {
        _transfer_function.erase(_transfer_function.begin() + _current_interaction_index);
        _current_interaction_index = -1;
        _transfer_function_dirty = true;
    }
    if (pushed_disabled_style) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
        pushed_disabled_style = false;
    }

    auto draw_color_picker = [&]() {
        pushed_disabled_style = false;
        if (_current_interaction_index < 0) {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            pushed_disabled_style = true;
        }
        if (_current_interaction_index >= 1 &&
            _current_interaction_index <= _transfer_function.size() - 1)
        {
            float* rgba = glm::value_ptr(_transfer_function[_current_interaction_index].rgba);
            if (ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel)) {
                _transfer_function_dirty = true;
            }
        }
        else {
            float rgba[4];
            ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel);
        }
        if (pushed_disabled_style) {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
            pushed_disabled_style = false;
        }
    };

    if (_color_edit_as_popup) {
        ImVec2 clr_button_pos = {rm_button_pos[0] + 0.52f*canvas_width, rm_button_pos[1]};
        ImGui::SetCursorPos(clr_button_pos);
        pushed_disabled_style = false;
        if (_current_interaction_index < 0 || _color_popup_open) {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            pushed_disabled_style = true;
        }
        if (ImGui::Button("Change Color", button_size)) {
            ImGui::OpenPopup("Change Color");
        }
        if (pushed_disabled_style) {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
            pushed_disabled_style = false;
        }

        if (ImGui::BeginPopup("Change Color")) {
            _color_popup_open = true;
            if (_current_interaction_index >= 1 &&
                    _current_interaction_index <= _transfer_function.size() - 1)
            {
                float* rgba = glm::value_ptr(_transfer_function[_current_interaction_index].rgba);
                if (ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel)) {
                    _transfer_function_dirty = true;
                }
            }
            else {
                float rgba[4];
                ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel);
            }
            ImGui::EndPopup();
        } else {
            _color_popup_open = false;
        }
    } else {
        ImGui::SetCursorPos(ImVec2{0.0, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
        ImGui::Separator();
        ImGui::SetCursorPos(ImVec2{centering_offset, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
        ImGui::PushItemWidth(canvas_width);
        draw_color_picker();
        ImGui::PopItemWidth();
    }

}
