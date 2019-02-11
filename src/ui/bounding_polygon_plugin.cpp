#include "bounding_polygon_plugin.h"

#include "state.h"
#include "utils/colors.h"
#include "utils/utils.h"
#include <igl/edges.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#pragma optimize ("", off)

void bounding_cage_polygon(BoundingCage& cage, Eigen::MatrixXf& V, Eigen::MatrixXi& F) {
    V.resize(cage.num_keyframes(), 3);
    F.resize((cage.num_keyframes()-1)*2*4+4, 3);

    for (BoundingCage::KeyFrame& kf : cage.keyframes) {

    }
}

Bounding_Polygon_Menu::Bounding_Polygon_Menu(State& state)
    : state(state)
    , widget_2d(Bounding_Polygon_Widget(state))
    , widget_3d(Bounding_Widget_3d(state))
{}


void Bounding_Polygon_Menu::initialize() {
    // Store a backup copy of the viewer viewport and then set the viewport to the size specified by the
    // layout constraints of the widget
    old_viewport = viewer->core.viewport;
    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    viewer_viewport = Eigen::Vector4f(view_hsplit*window_width, view_vsplit*window_height,
                                      (1.0-view_hsplit)*window_width, (1.0-view_vsplit)*window_height);
    viewer->core.viewport = viewer_viewport;
    viewer->data().clear();
    for (size_t i = viewer->data_list.size() - 1; i > 0; i--) {
        viewer->erase_mesh(i);
    }
    viewer->append_mesh();
    viewer->selected_data_index = 0;

    // Initialize the 2d cross section widget
    widget_2d.initialize(viewer, this);

    // Initialize the 3d volume viewer
    widget_3d.initialize(viewer, this);

    exporter.init(128, 128, 1024);

    if (transfer_function.size() == 0) {
        TfNode n1 = {0.0, glm::vec4(0.0)};
        TfNode n2 = {1.0, glm::vec4(1.0)};
        transfer_function.push_back(n1);
        transfer_function.push_back(n2);
    }
    state.logger->trace("Done initializing bounding polygon plugin!");
}

void Bounding_Polygon_Menu::deinitialize() {
    viewer->core.viewport = old_viewport;
}

bool Bounding_Polygon_Menu::is_2d_widget_in_focus()  {
    double mouse_x, mouse_y;
    glfwGetCursorPos(viewer->window, &mouse_x, &mouse_y);
    glm::vec2 p(mouse_x, mouse_y);
    return widget_2d.is_point_in_widget(p) && !mouse_in_popup;
}

bool Bounding_Polygon_Menu::mouse_move(int mouse_x, int mouse_y) {
    bool ret = FishUIViewerPlugin::mouse_move(mouse_x, mouse_y);

    // The mouse position given by ImGui is one frame behind so the in focus event will be delayed here.
    // Because of this we compute focus based on the latest mouse position
    bool in_focus = widget_2d.is_point_in_widget(glm::ivec2(mouse_x, mouse_y)) && !mouse_in_popup;

    ret = ret || widget_2d.mouse_move(mouse_x, mouse_y, in_focus);
    return ret;
}

bool Bounding_Polygon_Menu::mouse_down(int button, int modifier) {
    bool ret = FishUIViewerPlugin::mouse_down(button, modifier);

    ret = ret || widget_2d.mouse_down(button, modifier, is_2d_widget_in_focus());
    return ret;
}

bool Bounding_Polygon_Menu::mouse_up(int button, int modifier) {
    bool ret = FishUIViewerPlugin::mouse_up(button, modifier);
    ret = ret || widget_2d.mouse_up(button, modifier, is_2d_widget_in_focus());
    return ret;
}

bool Bounding_Polygon_Menu::mouse_scroll(float delta_y) {
    bool ret = FishUIViewerPlugin::mouse_scroll(delta_y);

    ret = ret || widget_2d.mouse_scroll(delta_y, is_2d_widget_in_focus());
    return ret;
}

bool Bounding_Polygon_Menu::key_down(int button, int modifier) {
    bool ret = FishUIViewerPlugin::key_down(button, modifier);
    ret = ret || widget_2d.key_down(button, modifier, is_2d_widget_in_focus());
    return ret;
}

bool Bounding_Polygon_Menu::key_up(int button, int modifier) {
    bool ret = FishUIViewerPlugin::key_up(button, modifier);
    ret = ret || widget_2d.key_up(button, modifier, is_2d_widget_in_focus());
    return ret;
}

void Bounding_Polygon_Menu::post_draw_transfer_function() {
    ImGui::Text("%s", "Edit Transfer Function");
    ImGui::Separator();

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
    ImVec2 button_size {canvas_width*0.48f, 0.0f};
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

    ImGui::SetCursorPos(ImVec2{0.0, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
    ImGui::Separator();
    ImGui::SetCursorPos(ImVec2{centering_offset, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
    ImGui::PushItemWidth(canvas_width);
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
    ImGui::PopItemWidth();

//    if (false) {
//        ImVec2 clr_button_pos = {rm_button_pos[0] + 0.52f*canvas_width, rm_button_pos[1]};
//        ImGui::SetCursorPos(clr_button_pos);
//        pushed_disabled_style = false;
//        if (current_interaction_index < 0 || color_popup_open) {
//            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
//            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
//            pushed_disabled_style = true;
//        }
//        if (ImGui::Button("Change Color", button_size)) {
//            ImGui::OpenPopup("Change Color");
//        }
//        if (pushed_disabled_style) {
//            ImGui::PopItemFlag();
//            ImGui::PopStyleVar();
//            pushed_disabled_style = false;
//        }

//        if (ImGui::BeginPopup("Change Color")) {
//            color_popup_open = true;
//    //        ImGui::SetCursorPos(ImVec2{0.0, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
//    //        ImGui::Separator();
//    //        ImGui::SetCursorPos(ImVec2{centering_offset, ImGui::GetCursorPosY() + 0.3f*ImGui::GetTextLineHeight()});
//    //        ImGui::PushItemWidth(canvas_width);
//            if (current_interaction_index >= 1 &&
//                current_interaction_index <= transfer_function.size() - 1)
//            {
//                float* rgba = glm::value_ptr(transfer_function[current_interaction_index].rgba);
//                if (ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel)) {
//                    transfer_function_dirty = true;
//                }
//            }
//            else {
//                float rgba[4];
//                ImGui::ColorPicker4("Change Color", rgba, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoLabel);
//            }
//            ImGui::EndPopup();
//        } else {
//            color_popup_open = false;
//        }
//    }

}

bool Bounding_Polygon_Menu::post_draw() {
    if (cage_dirty) {
        double depth = 0, width, height;
        Eigen::RowVector3d last_centroid = state.cage.keyframes.begin()->centroid_3d();
        for (const BoundingCage::KeyFrame& kf : state.cage.keyframes) {
            depth += (kf.centroid_3d() - last_centroid).norm();
            last_centroid = kf.centroid_3d();
        }
        depth = round(depth) * widget_3d.export_rescale_factor;

        Eigen::RowVector4d cage_bbox = state.cage.keyframe_bounding_box();
        width = std::max(round(fabs(cage_bbox[1] - cage_bbox[0])), 1.0) * widget_3d.export_rescale_factor;
        height = std::max(round(fabs(cage_bbox[3] - cage_bbox[2])), 1.0) * widget_3d.export_rescale_factor;

        glBindTexture(GL_TEXTURE_3D, state.low_res_volume.volume_texture);
        GLint old_min_filter, old_mag_filter;
        glGetTexParameteriv(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, &old_min_filter);
        glGetTexParameteriv(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, &old_mag_filter);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        exporter.set_export_dims(width, height, depth);
        exporter.update(state.cage, state.low_res_volume.volume_texture, G3i(state.low_res_volume.dims()));

        glBindTexture(GL_TEXTURE_3D, state.low_res_volume.volume_texture);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, old_min_filter);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, old_mag_filter);
        glBindTexture(GL_TEXTURE_3D, 0);
        cage_dirty = false;
    }

    if (transfer_function_dirty) {
        widget_3d.volume_renderer.set_transfer_function(transfer_function);
        transfer_function_dirty = false;
    }

    bool ret = FishUIViewerPlugin::post_draw();

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    widget_2d.position = glm::vec2(0.f, view_vsplit*window_height);
    widget_2d.size = glm::vec2(window_width*view_hsplit, (1.0-view_vsplit)*window_height);
    ret = widget_2d.post_draw(state.cage.keyframe_for_index(current_cut_index), is_2d_widget_in_focus());

    Eigen::Vector4f widget_3d_viewport(view_hsplit*window_width, view_vsplit*window_height,
                                       (1.0-view_hsplit)*window_width, (1.0-view_vsplit)*window_height);
    viewer->core.viewport = widget_3d_viewport;
    if (draw_straight) {
        ret = widget_3d.post_draw_straight(G4f(widget_3d_viewport), state.cage.keyframe_for_index(current_cut_index));
    } else {
        ret = widget_3d.post_draw_curved(G4f(widget_3d_viewport), state.cage.keyframe_for_index(current_cut_index));
    }


    ImGui::SetNextWindowBgAlpha(0.0f);
    float window_height_float = static_cast<float>(window_height);
    float window_width_float = static_cast<float>(window_width);
    ImGui::SetNextWindowPos(ImVec2(0.f, (1.0-view_vsplit)*window_height_float), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImVec2(window_width_float, window_height_float*view_vsplit), ImGuiSetCond_Always);
    ImGui::Begin("Select Boundary", nullptr,
                 ImGuiWindowFlags_NoSavedSettings |
                 ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoTitleBar);

    const float min_cut_index = static_cast<float>(state.cage.min_index());
    const float max_cut_index = static_cast<float>(state.cage.max_index());

    if (ImGui::Button("< Prev KF")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it--;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.begin();
        }
        current_cut_index = static_cast<float>(it->index());
    }
    ImGui::SameLine();
    if (ImGui::Button("<")) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index - keyframe_nudge_amount, max_cut_index));
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.85f);
    if(ImGui::SliderFloat("", &current_cut_index,
                          static_cast<float>(state.cage.min_index()),
                          static_cast<float>(state.cage.max_index()))) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index, max_cut_index));
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button(">")) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index + keyframe_nudge_amount, max_cut_index));
    }
    ImGui::SameLine();
    if (ImGui::Button("Next KF >")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it++;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.rbegin();
        }
        current_cut_index = static_cast<float>(it->index());
    }

    if (ImGui::InputFloat("Nudge Amount", &keyframe_nudge_amount, 0.01, 0.1, 5)) {}


    BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
    if (ImGui::Button("Insert KF")) {
        state.cage.insert_keyframe(current_cut_index);
        glfwPostEmptyEvent();
        cage_dirty = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove KF")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);

        state.cage.delete_keyframe(it);
        BoundingCage::KeyFrameIterator next = it++;
        if (next != state.cage.keyframes.end()) {
            current_cut_index = next->index();
        }
        glfwPostEmptyEvent();
        cage_dirty = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Rotation")) {
        if (kf->in_bounding_cage()) {
            kf->set_angle(0.0);
        }
        glfwPostEmptyEvent();
        cage_dirty = true;
    }

    /*
    ImGui::Separator();
    ImGui::Text("Num Keyframes: %d", state.cage.num_keyframes());
    ImGui::Separator();
    if (ImGui::InputInt("W", &exp_w)) {
        exporter.set_export_dims(exp_w, exp_h, exp_d);
    }
    ImGui::SameLine();
    if (ImGui::InputInt("H", &exp_h)) {
        exporter.set_export_dims(exp_w, exp_h, exp_d);
    }
    ImGui::SameLine();
    if (ImGui::InputInt("D", &exp_d)) {
        exporter.set_export_dims(exp_w, exp_h, exp_d);
    }
    ImGui::SameLine();
    if (ImGui::Button("Export Volume")) {
        exporter.update(state.cage, state.low_res_volume.volume_texture, G3i(state.low_res_volume.dims()));
        state.logger->debug("EXPORT");
        exporter.write_texture_data_to_file("out_volume.raw");
        state.logger->debug("DONE");
    }
    */

    ImGui::Separator();
    ImGui::Text("Display Options");
    ImGui::Checkbox("Show straight view", &draw_straight);

    bool pushed_disabled_style = false;
    if (show_display_options) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        pushed_disabled_style = true;
    }
    if (ImGui::Button("Display Options")) {
        show_display_options = true;
    }
    if (pushed_disabled_style) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    if (show_display_options) {
        ImGui::SetNextWindowSize(ImVec2(window_height_float*view_vsplit, 0), ImGuiSetCond_FirstUseEver);

        ImGui::Begin("Display Options");
//        ImGui::SetNextWindowSize(ImVec2(480, 720), ImGuiSetCond_FirstUseEver);
//        if (ImGui::Begin("Display Options", NULL)) {
        ImVec2 popup_pos = ImGui::GetWindowPos();
        ImVec2 popup_size = ImGui::GetWindowSize();
        double mouse_x, mouse_y;
        glfwGetCursorPos(viewer->window, &mouse_x, &mouse_y);

        post_draw_transfer_function();

        bool in_window_x = (mouse_x >= popup_pos[0]) && (mouse_x <= (popup_pos[0] + popup_size[0]));
        bool in_window_y = (mouse_y >= popup_pos[1]) && (mouse_y <= (popup_pos[1] + popup_size[1]));
        mouse_in_popup = (in_window_x && in_window_y);

        ImGui::Separator();
        if (ImGui::Button("Close")) {
            show_display_options = false;
        }
        ImGui::End();
    } else {
        show_display_options = false;
        mouse_in_popup = false;
    }

    ImGui::End();
    ImGui::Render();

    return ret;
}

