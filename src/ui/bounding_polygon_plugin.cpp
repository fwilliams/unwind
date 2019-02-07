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
    widget_2d.initialize(viewer);

    // Initialize the 3d volume viewer
    widget_3d.initialize(viewer);

    exporter.init(128, 128, 1024);

    state.logger->trace("Done initializing bounding polygon plugin!");
}

void Bounding_Polygon_Menu::deinitialize() {
    viewer->core.viewport = old_viewport;
}

bool Bounding_Polygon_Menu::mouse_move(int mouse_x, int mouse_y) {
    return widget_2d.mouse_move(mouse_x, mouse_y) || FishUIViewerPlugin::mouse_move(mouse_x, mouse_y);;
}


bool Bounding_Polygon_Menu::mouse_down(int button, int modifier) {
    return widget_2d.mouse_down(button, modifier) || FishUIViewerPlugin::mouse_down(button, modifier);
}


bool Bounding_Polygon_Menu::mouse_up(int button, int modifier) {
    return widget_2d.mouse_up(button, modifier) || FishUIViewerPlugin::mouse_up(button, modifier);
}

bool Bounding_Polygon_Menu::mouse_scroll(float delta_y) {
    return widget_2d.mouse_scroll(delta_y) || FishUIViewerPlugin::mouse_scroll(delta_y);
}


bool Bounding_Polygon_Menu::pre_draw() {
    bool ret = FishUIViewerPlugin::pre_draw();

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);

    return ret;
}


bool Bounding_Polygon_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

    int window_width, window_height;
    glfwGetWindowSize(viewer->window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    widget_2d.position = glm::vec2(0.f, view_vsplit*window_height);
    widget_2d.size = glm::vec2(window_width*view_hsplit, (1.0-view_vsplit)*window_height);
    ret = widget_2d.post_draw(state.cage.keyframe_for_index(current_cut_index), static_cast<int>(current_cut_index));

    Eigen::Vector4f widget_3d_viewport(view_hsplit*window_width, view_vsplit*window_height,
                                       (1.0-view_hsplit)*window_width, (1.0-view_vsplit)*window_height);
    viewer->core.viewport = widget_3d_viewport;
    ret = widget_3d.post_draw(G4f(widget_3d_viewport), state.cage.keyframe_for_index(current_cut_index));

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

    if (ImGui::InputFloat("Nudge Amount", &keyframe_nudge_amount, 0.01, 0.1, 5)) {

    }


    if (ImGui::Button("Insert KF")) {
        state.cage.insert_keyframe(current_cut_index);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove KF")) {
        BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
        state.cage.delete_keyframe(it);
        glfwPostEmptyEvent();
    }

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
        exporter.draw(state.cage, state.volume_rendering.volume_texture,
                      state.volume_rendering.parameters.volume_dimensions);
        state.logger->debug("EXPORT");
        exporter.write_texture_data_to_file("out_volume.raw");
        state.logger->debug("DONE");
    }

    const double angle_3deg = M_2_PI / 120.0;
    const double angle_10deg = M_2_PI / 36.0;
    if (ImGui::Button("-3deg")) {
        BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
        if (!kf->in_bounding_cage()) {
            kf = state.cage.insert_keyframe(current_cut_index);
        }
        kf->rotate_torsion_frame(-angle_3deg);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("+3deg")) {
        BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
        if (!kf->in_bounding_cage()) {
            kf = state.cage.insert_keyframe(current_cut_index);
        }
        kf->rotate_torsion_frame(angle_3deg);
        glfwPostEmptyEvent();
    }

    if (ImGui::Button("-10deg")) {
        BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
        if (!kf->in_bounding_cage()) {
            kf = state.cage.insert_keyframe(current_cut_index);
        }
        kf->rotate_torsion_frame(-angle_10deg);
        glfwPostEmptyEvent();
    }
    ImGui::SameLine();
    if (ImGui::Button("+10deg")) {
        BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
        if (!kf->in_bounding_cage()) {
            kf = state.cage.insert_keyframe(current_cut_index);
        }
        kf->rotate_torsion_frame(angle_10deg);
        glfwPostEmptyEvent();
    }

    ImGui::End();
    ImGui::Render();

    return ret;
}

