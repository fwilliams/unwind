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

    if (tf_widget.tfdirty()) {
        widget_3d.volume_renderer.set_transfer_function(tf_widget.tfunc());
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

        ImGui::Text("%s", "Edit Transfer Function");
        ImGui::Separator();
        tf_widget.post_draw();

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

