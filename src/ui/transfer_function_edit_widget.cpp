#include "transfer_function_edit_widget.h"
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

TransferFunctionEditWidget::TransferFunctionEditWidget() {

}

bool TransferFunctionEditWidget::post_draw() {
    constexpr const float Radius = 10.f;

    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 1.5f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    //ImVec2 canvas_size = { 640.f, 150.f };

    float aspect_ratio = 150.f / 200.f; // height / width
    float canvas_width = 0.9f * ImGui::GetContentRegionAvailWidth();
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

                std::sort(transfer_function.begin(), transfer_function.end(), [](const TfNode& lhs, const TfNode& rhs) { return lhs.t < rhs.t; });
                transfer_function_dirty = true;
            } else {
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


    ImVec2 rm_button_pos = {centering_offset, ImGui::GetCursorPosY() + 0.4f*ImGui::GetTextLineHeight()};
    ImGui::SetCursorPos(rm_button_pos);

    ImVec2 button_size = color_edit_as_popup ? ImVec2{canvas_width*0.48f, 0.0f} : ImVec2{canvas_width, 0.0f};
    const bool is_first = current_interaction_index == 0;
    const bool is_last = current_interaction_index == transfer_function.size() - 1;
    bool pushed_disabled_style = false;
    if (current_interaction_index < 0 || is_first || is_last) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        pushed_disabled_style = true;
    }
    if (ImGui::Button("Remove Node", button_size)) {
        transfer_function.erase(transfer_function.begin() + current_interaction_index);
        current_interaction_index = -1;
        transfer_function_dirty = true;
    }
    if (pushed_disabled_style) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
        pushed_disabled_style = false;
    }

    auto draw_color_picker = [&]() {
        if (current_interaction_index >= 1 &&
            current_interaction_index <= transfer_function.size() - 1)
        {
            float* rgba = glm::value_ptr(transfer_function[current_interaction_index].rgba);
            if (ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel)) {
                transfer_function_dirty = true;
            }
        }
        else {
            float rgba[4];
            ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel);
        }
    };

    if (color_edit_as_popup) {
        ImVec2 clr_button_pos = {rm_button_pos[0] + 0.52f*canvas_width, rm_button_pos[1]};
        ImGui::SetCursorPos(clr_button_pos);
        pushed_disabled_style = false;
        if (current_interaction_index < 0 || color_popup_open) {
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
            color_popup_open = true;
            if (current_interaction_index >= 1 &&
                    current_interaction_index <= transfer_function.size() - 1)
            {
                float* rgba = glm::value_ptr(transfer_function[current_interaction_index].rgba);
                if (ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel)) {
                    transfer_function_dirty = true;
                }
            }
            else {
                float rgba[4];
                ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel);
            }
            ImGui::EndPopup();
        } else {
            color_popup_open = false;
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
