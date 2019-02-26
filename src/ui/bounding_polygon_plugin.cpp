#include "bounding_polygon_plugin.h"

#include "state.h"
#include <utils/colors.h>
#include <utils/utils.h>
#include <utils/open_file_dialog.h>
#include <utils/path_utils.h>

#include <igl/edges.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
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
    widget_3d.volume_renderer.set_transfer_function(tf_widget.transfer_function());

    exporter.init(128, 128, 1024);

    state.logger->trace("Done initializing bounding polygon plugin!");

    cage_dirty = true;
}

void Bounding_Polygon_Menu::deinitialize() {
    viewer->core.viewport = old_viewport;
    widget_2d.deinitialize();
    widget_3d.deinitialize();
    exporter.destroy();
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
        GLuint tex = use_hires_texture ? state.hi_res_volume.volume_texture : state.low_res_volume.volume_texture;
        exporter.update(state.cage, tex, G3i(state.low_res_volume.dims()));

        glBindTexture(GL_TEXTURE_3D, state.low_res_volume.volume_texture);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, old_min_filter);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, old_mag_filter);
        glBindTexture(GL_TEXTURE_3D, 0);
        cage_dirty = false;
    }

    if (tf_widget.transfer_function_dirty()) {
        widget_3d.volume_renderer.set_transfer_function(tf_widget.transfer_function());
        tf_widget.clear_dirty_bit();
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

    ImVec2 prev_kf_text_size = ImGui::CalcTextSize(" < Prev KF ");
    ImVec2 nudge_left_text_size = ImGui::CalcTextSize(" < ");
    ImVec2 next_kf_text_size = ImGui::CalcTextSize(" Next KF > ");
    ImVec2 nudge_right_text_size = ImGui::CalcTextSize(" > ");
    float text_button_w = std::max(prev_kf_text_size.x, next_kf_text_size.x);
    float nudge_button_w = std::max(nudge_left_text_size.x, nudge_right_text_size.x);

    ImGui::PushItemWidth(text_button_w);
    if (ImGui::Button("< Prev KF")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it--;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.begin();
        }
        current_cut_index = static_cast<float>(it->index());
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushItemWidth(nudge_button_w);
    if (ImGui::Button("<")) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index - keyframe_nudge_amount, max_cut_index));
    }
    ImGui::PopItemWidth();

    ImGui::SameLine();
    ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
    ImGui::PushItemWidth(ImGui::GetWindowWidth() - 2.f*cursor_pos.x);
    if(ImGui::SliderFloat("", &current_cut_index,
                          static_cast<float>(state.cage.min_index()),
                          static_cast<float>(state.cage.max_index()))) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index, max_cut_index));
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushItemWidth(nudge_button_w);
    if (ImGui::Button(">")) {
        current_cut_index = std::max(min_cut_index, std::min(current_cut_index + keyframe_nudge_amount, max_cut_index));
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushItemWidth(text_button_w);
    if (ImGui::Button("Next KF >")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        it++;
        if (it == state.cage.keyframes.end()) {
            it = state.cage.keyframes.rbegin();
        }
        current_cut_index = static_cast<float>(it->index());
    }
    ImGui::PopItemWidth();

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

    ImGui::Separator();
    ImGui::Text("Display Options");
    if (ImGui::Checkbox("Show Straight View", &draw_straight)) {
        if (draw_straight) {
            widget_3d.center_straight_mesh();
        } else {
            widget_3d.center_bounding_cage_mesh();
        }
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Show Hi-Res Texture", &use_hires_texture)) {
        cage_dirty = true;
    }
    ImGui::SameLine();
    bool pushed_disabled_style = false;
    if (show_edit_transfer_function) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        pushed_disabled_style = true;
    }

    if (ImGui::Button("Edit Transfer Function")) {
        show_edit_transfer_function = true;
    }
    if (pushed_disabled_style) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }
    ImGui::SameLine();
    if (ImGui::Button("Center View")) {
        if (draw_straight) {
            widget_3d.center_straight_mesh();
        } else {
            widget_3d.center_bounding_cage_mesh();
        }
    }
    if (show_edit_transfer_function) {
        ImGui::SetNextWindowSize(ImVec2(window_width*view_vsplit, 0), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Edit Transfer Function", &show_edit_transfer_function, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        ImVec2 popup_pos = ImGui::GetWindowPos();
        ImVec2 popup_size = ImGui::GetWindowSize();
        tf_widget.post_draw(!show_save_popup /* active */);

        double mouse_x, mouse_y;
        glfwGetCursorPos(viewer->window, &mouse_x, &mouse_y);
        bool in_window_x = (mouse_x >= popup_pos[0]) && (mouse_x <= (popup_pos[0] + popup_size[0]));
        bool in_window_y = (mouse_y >= popup_pos[1]) && (mouse_y <= (popup_pos[1] + popup_size[1]));
        mouse_in_popup = (in_window_x && in_window_y);

        ImGui::Separator();
        if (ImGui::Button("Close")) {
            show_edit_transfer_function = false;
        }
        ImGui::End();
    } else {
        show_edit_transfer_function = false;
        mouse_in_popup = false;
    }

    ImGui::NewLine();
    ImGui::Separator();
    if (ImGui::Button("Back")) {
        state.set_application_state(Application_State::EndPointSelection);
    }

    if (show_save_popup) {
        ImGui::SetNextWindowSize(ImVec2(window_width*0.4, 0), ImGuiSetCond_FirstUseEver);
        ImGui::OpenPopup("Save Result");
        ImGui::BeginPopupModal("Save Result");

        ImGui::Text("Project Name:");
        ImGui::PushItemWidth(-1);
        if (ImGui::InputText("##SaveDest", save_name_buf, PATH_BUFFER_SIZE)) {
            trim_path_in_place(save_name_buf);
            const bool found_slash = std::string(save_name_buf).find_first_of("/") != std::string::npos;
            const bool found_backslash = std::string(save_name_buf).find_first_of("\\") != std::string::npos;
            if (found_slash || found_backslash) {
                save_name_invalid = true;
                save_name_overwrite = false;
                save_name_error_message = "Output name cannot contain '/' or '\\'";
            } else {
                save_name_invalid = false;
                save_name_error_message = "";
            }

            if (!save_name_invalid) {
                const std::string save_file_name = std::string(save_name_buf);
                const std::string save_project_path = state.input_metadata.output_dir + "/" + save_file_name + ".fish.pro";
                const std::string save_datfile_path = state.input_metadata.output_dir + "/" + save_file_name + ".dat";
                const std::string save_rawfile_path = state.input_metadata.output_dir + "/" + save_file_name + ".raw";
                if (get_file_type(save_project_path.c_str()) != FT_DOES_NOT_EXIST) {
                    save_name_error_message = "Warning: A file named " + save_file_name + ".fish.pro exists. Saving will overwrite it.";
                    save_name_overwrite = true;
                } else if (get_file_type(save_datfile_path.c_str()) != FT_DOES_NOT_EXIST) {
                    save_name_error_message = "Warning: A file named " + save_file_name + ".dat exists. Saving will overwrite it.";
                    save_name_overwrite = true;
                } else if (get_file_type(save_rawfile_path.c_str()) != FT_DOES_NOT_EXIST) {
                    save_name_error_message = "Warning: A file named " + save_file_name + ".raw exists. Saving will overwrite it.";
                    save_name_overwrite = true;
                } else {
                    save_name_error_message = "";
                    save_name_overwrite = false;
                }
            }
        }
        ImGui::PopItemWidth();

        bool disabled = false;
        if (save_name_invalid) {
            ImGui::TextColored(ImColor(200, 20, 20, 255), "%s", save_name_error_message.c_str());
            disabled = true;
        }
        if (save_name_overwrite) {
            ImGui::TextColored(ImColor(200, 200, 20, 255), "%s", save_name_error_message.c_str());
        }

        auto reset_dims = [&]() {
            double d = -1.0;
            std::vector<double> kf_depths;
            state.cage.keyframe_depths(kf_depths);
            d = kf_depths.back();

            Eigen::Vector4d kfbb = state.cage.keyframe_bounding_box();
            double w = kfbb[1] - kfbb[0];
            double h = kfbb[3] - kfbb[2];

            output_dims[0] = int(w*state.input_metadata.downsample_factor);
            output_dims[1] = int(h*state.input_metadata.downsample_factor);
            output_dims[2] = int(d*state.input_metadata.downsample_factor);
        };

        // First time we open the popup
        if (output_dims[0] < 0) {
            reset_dims();
        }
        ImGui::Spacing();
        ImGui::Text("Output Dimensions:");
        ImGui::PushItemWidth(-1);
        int old_dims[3] = {output_dims[0], output_dims[1], output_dims[2]};
        if (ImGui::InputInt3("##Outdims", output_dims)) {
            int change_idx = 0;
            for (change_idx = 0; change_idx < 3; change_idx++) {
                if (old_dims[change_idx] != output_dims[change_idx]) {
                    break;
                }
            }

            if (output_preserve_aspect_ratio) {
                std::vector<double> kf_depths;
                state.cage.keyframe_depths(kf_depths);
                double d = kf_depths.back();

                Eigen::RowVector4d kfbb = state.cage.keyframe_bounding_box();
                double w = std::max(round(fabs(kfbb[1] - kfbb[0])), 1.0);
                double h = std::max(round(fabs(kfbb[3] - kfbb[2])), 1.0);

                double new_d, new_w, new_h;
                if (change_idx == 0) {
                    new_w = double(output_dims[0]);
                    new_h = (h/w)*new_w;
                    new_d = (d/w)*new_w;
                } else if (change_idx == 1) {
                    new_h = double(output_dims[1]);
                    new_w = (w/h)*new_h;
                    new_d = (d/h)*new_h;
                } else if (change_idx == 2) {
                    new_d = double(output_dims[2]);
                    new_w = (w/d)*new_d;
                    new_h = (h/d)*new_d;
                }

                int new_dims[3] = {int(new_w), int(new_h), int(new_d)};
                for (int i = 0; i < 3; i++) { output_dims[i] = new_dims[i]; }
            }

            bool was_negative = false;
            for (int i = 0; i < 3; i++) {
                if (output_dims[i] <= 0) {
                    was_negative = true;
                    break;
                }
            }

            if (was_negative) {
                output_dims[0] = old_dims[0];
                output_dims[1] = old_dims[1];
                output_dims[2] = old_dims[2];
            }
        }
        ImGui::PopItemWidth();
        ImGui::Checkbox("Preserve Aspect Ratio", &output_preserve_aspect_ratio);
        ImGui::SameLine();
        if (ImGui::Button("Reset Dims")) {
            reset_dims();
        }
        if (std::string(save_name_buf).size() == 0) {
            disabled = true;
        }

        if (!save_name_invalid && !save_name_overwrite) {
            ImGui::NewLine();
        }

        ImGui::Separator();

        if (disabled) {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        if (ImGui::Button("Save")) {
            const std::string save_file_name = std::string(save_name_buf);
            const std::string save_project_path = state.input_metadata.output_dir + "/" + save_file_name + ".fish.pro";
            const std::string save_datfile_path = state.input_metadata.output_dir + "/" + save_file_name + ".dat";
            const std::string save_rawfile_path = state.input_metadata.output_dir + "/" + save_file_name + ".raw";

            igl::serialize(state, "state", save_project_path, true);
            DatFile out_datfile;
            out_datfile.w = output_dims[0];
            out_datfile.h = output_dims[1];
            out_datfile.d = output_dims[2];
            out_datfile.m_raw_filename = save_file_name + ".raw";
            out_datfile.m_format = "UINT8";
            out_datfile.serialize(save_datfile_path, state.logger);


            {
                glBindTexture(GL_TEXTURE_3D, state.hi_res_volume.volume_texture);
                GLint old_min_filter, old_mag_filter;
                glGetTexParameteriv(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, &old_min_filter);
                glGetTexParameteriv(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, &old_mag_filter);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glBindTexture(GL_TEXTURE_3D, 0);
                exporter.set_export_dims(output_dims[0], output_dims[1], output_dims[2]);
                exporter.update(state.cage, state.hi_res_volume.volume_texture, G3f(state.low_res_volume.dims()));
                exporter.write_texture_data_to_file(save_rawfile_path);
                cage_dirty = true;
                glBindTexture(GL_TEXTURE_3D, state.hi_res_volume.volume_texture);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, old_min_filter);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, old_mag_filter);
                glBindTexture(GL_TEXTURE_3D, 0);
            }

            show_save_popup = false;
            output_dims[0] = -1;
            output_dims[1] = -1;
            output_dims[2] = -2;
        }
        if (disabled) {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            show_save_popup = false;
            output_dims[0] = -1;
            output_dims[1] = -1;
            output_dims[2] = -2;
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        show_save_popup = true;
    }

    ImGui::End();

    // Draw a line separating the two half views
    {
        ImGui::SetNextWindowPos(ImVec2(0.f,0.f), ImGuiSetCond_Always);
        ImGui::SetNextWindowSize(ImVec2(window_width_float, window_height_float), ImGuiSetCond_Always);
        ImGui::SetNextWindowBgAlpha(0.0f);
        ImGui::Begin("Full screen derp", NULL,
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoInputs |
                     ImGuiWindowFlags_NoTitleBar);
        ImVec2 line_start = {window_width*view_hsplit, 0.0f};
        ImVec2 line_end = {window_width*view_hsplit, (1.0f-view_vsplit)*window_height};
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(line_start, line_end, ImGui::GetColorU32(ImGuiCol_Separator));
        ImGui::End();
    }


    ImGui::Render();
    return ret;
}

