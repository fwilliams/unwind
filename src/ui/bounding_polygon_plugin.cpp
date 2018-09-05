#include "bounding_polygon_plugin.h"

#include <unordered_map>

#include <igl/edges.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include <utils/colors.h>
#include <utils/utils.h>


Bounding_Polygon_Menu::Bounding_Polygon_Menu(State& state)
  : state(state), widget_2d(Bounding_Polygon_Widget(state))
{}


void Bounding_Polygon_Menu::initialize() {
  for (int i = viewer->data_list.size()-1; i > 0; i--) {
    viewer->erase_mesh(i);
  }
  mesh_overlay_id = 0;
  viewer->selected_data_index = mesh_overlay_id;

  const Eigen::MatrixXd& TV = state.extracted_volume.TV;
  const Eigen::MatrixXi& TF = state.extracted_volume.TF;
  viewer->data().set_mesh(TV, TF);
  viewer->core.align_camera_center(TV, TF);

  viewer->append_mesh();
  points_overlay_id = viewer->selected_data_index;

  viewer->selected_data_index = mesh_overlay_id;

  int push_mesh_id = viewer->selected_data_index;
  viewer->selected_data_index = mesh_overlay_id;
  {
    viewer->data().clear();
    Eigen::MatrixXi E;
    igl::edges(state.extracted_volume.TF, E);
    viewer->data().set_edges(state.extracted_volume.TV, E, Eigen::RowVector3d(0.75, 0.75, 0.75));

    Eigen::MatrixXd P1(state.cage.skeleton_vertices().rows()-1, 3), P2(state.cage.skeleton_vertices().rows()-1, 3);
    for (int i = 0; i < state.cage.skeleton_vertices().rows()-1; i++) {
      P1.row(i) = state.cage.skeleton_vertices().row(i);
      P2.row(i) = state.cage.skeleton_vertices().row(i+1);
    }
    viewer->data().add_edges(P1, P2, ColorRGB::LIGHT_GREEN);
    viewer->data().point_size = 10.0;

    for (int i = 0; i < state.cage.smooth_skeleton_vertices().rows()-1; i++) {
      P1.row(i) = state.cage.smooth_skeleton_vertices().row(i);
      P2.row(i) = state.cage.smooth_skeleton_vertices().row(i+1);
    }
    viewer->data().add_edges(P1, P2, ColorRGB::RED);
    viewer->data().point_size = 20.0;

    viewer->selected_data_index = push_mesh_id;
  }

  // Draw the bounding cage mesh
  viewer->append_mesh();
  cage_mesh_overlay_id = viewer->selected_data_index;
  {
    viewer->data().clear();
    viewer->data().set_face_based(true);
    viewer->data().set_mesh(state.cage.vertices(), state.cage.faces());
    viewer->selected_data_index = push_mesh_id;
  }

  // Initialize the 2d cross section widget
  widget_2d.initialize(viewer);

  state.logger->trace("Done initializing bounding polygon plugin!");
}


bool Bounding_Polygon_Menu::mouse_move(int mouse_x, int mouse_y) {
  if (show_slice_view) {
    return widget_2d.mouse_move(mouse_x, mouse_y);
  } else {
    return FishUIViewerPlugin::mouse_move(mouse_x, mouse_y);
  }
}


bool Bounding_Polygon_Menu::mouse_down(int button, int modifier) {
  if (show_slice_view) {
    return widget_2d.mouse_down(button, modifier);
  } else {
    return FishUIViewerPlugin::mouse_down(button, modifier);
  }
}


bool Bounding_Polygon_Menu::mouse_up(int button, int modifier) {
  if (show_slice_view) {
    return widget_2d.mouse_up(button, modifier);
  } else {
    return FishUIViewerPlugin::mouse_up(button, modifier);
  }
}


bool Bounding_Polygon_Menu::post_draw() {
  bool ret = FishUIViewerPlugin::post_draw();

  int width;
  int height;

  glfwGetWindowSize(viewer->window, &width, &height);

  ImGui::SetNextWindowBgAlpha(0.5);
  ImGui::SetNextWindowPos(ImVec2(.0f, int(0.8*height)), ImGuiSetCond_Always);
  ImGui::SetNextWindowSize(ImVec2(width, int(height*0.35)), ImGuiSetCond_Always);
  ImGui::Begin("Select Boundary", NULL,
               ImGuiWindowFlags_NoSavedSettings |
               ImGuiWindowFlags_AlwaysAutoResize |
               ImGuiWindowFlags_NoTitleBar);

  const double delta = (state.cage.max_index() - state.cage.min_index()) / (2*state.cage.skeleton_vertices().rows());

  if (ImGui::Button("< Prev")) {
    current_cut_index = std::max(current_cut_index - delta, 0.0);
  }
  ImGui::SameLine();
  if (ImGui::SliderFloat("#vertexid", &current_cut_index, (float)state.cage.min_index(), (float)state.cage.max_index())) {
    current_cut_index = std::max(state.cage.min_index(), std::min((double)current_cut_index, state.cage.max_index()));
  }
  ImGui::SameLine();
  if (ImGui::Button("Next >")) {
    current_cut_index = std::min(current_cut_index + delta, state.cage.max_index());
  }

  ImGui::Checkbox("Show slice view", &show_slice_view);
  if (show_slice_view) {
    widget_2d.post_draw(PV, current_cut_index);
  }

  if (ImGui::Button("Insert KF")) {
    state.cage.insert_keyframe(current_cut_index);
  }
  if (ImGui::Button("Remove KF")) {
    BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
    state.cage.remove_keyframe(it);
  }
  BoundingCage::KeyFrameIterator it = state.cage.keyframe_for_index(current_cut_index);
  if (ImGui::InputInt("vertex: ", &current_vertex, 1, 1)) {
    if (current_vertex < 0) {
      current_vertex = 0;
    }
    if (current_vertex >= it->vertices_2d().rows()) {
      current_vertex = it->vertices_2d().rows()-1;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("+x")) {
    Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(1, 0);
    it->move_point_2d(current_vertex, pt, true, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("-x")) {
    Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(-1, 0);
    it->move_point_2d(current_vertex, pt, true, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("+y")) {
    Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(0, 1);
    it->move_point_2d(current_vertex, pt, true, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("-y")) {
    Eigen::RowVector2d pt = it->vertices_2d().row(current_vertex) + Eigen::RowVector2d(0, -1);
    it->move_point_2d(current_vertex, pt, true, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("Subdiv")) {
    state.cage.add_boundary_vertex(current_vertex, 0.5);
  }

  ImGui::End();
  ImGui::Render();

  return ret;
}


bool Bounding_Polygon_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();

  glDisable(GL_CULL_FACE);

  int push_overlay_id = viewer->selected_data_index;

  viewer->selected_data_index = points_overlay_id;
  {
    viewer->data().clear();
    viewer->data().line_width = 2.4;

    for (BoundingCage::KeyFrame& kf : state.cage.keyframes) {
      viewer->data().add_points(kf.center(), ColorRGB::GREEN);
      Eigen::MatrixXd V3d = kf.vertices_3d();
      for (int i = 0; i < V3d.rows(); i++) {
        Eigen::RowVector3d c = (i == current_vertex ? ColorRGB::AQUA : ColorRGB::RED);
        viewer->data().add_points(V3d.row(i), c);
      }
    }

    for (auto kf = state.cage.keyframes.rbegin(); kf != state.cage.keyframes.rend(); --kf) {
      Eigen::Matrix3d cf = kf->orientation();

      viewer->data().add_edges(kf->center(), kf->center() + 100.0*cf.row(0), ColorRGB::RED);
      viewer->data().add_edges(kf->center(), kf->center() + 100.0*cf.row(1), ColorRGB::GREEN);
      viewer->data().add_edges(kf->center(), kf->center() + 100.0*cf.row(2), ColorRGB::BLUE);
    }

    viewer->data().point_size = 10.0;
//    viewer->data().add_points(state.cage.vertices(), ColorRGB::RED);
    BoundingCage::KeyFrameIterator kf = state.cage.keyframe_for_index(current_cut_index);
    Eigen::MatrixXd P1, P2;
    edge_endpoints(state.cage.vertices(), state.cage.faces(), P1, P2);
    viewer->data().add_edges(P1, P2, ColorRGB::GREEN);
    viewer->data().add_points(kf->center(), ColorRGB::RED);
    viewer->data().add_points(kf->vertices_3d(), ColorRGB::LIGHT_GREEN);
    viewer->data().add_edges(kf->center(), kf->center() + 100*kf->orientation().row(0), ColorRGB::RED);
    viewer->data().add_edges(kf->center(), kf->center() + 100*kf->orientation().row(1), ColorRGB::GREEN);
    viewer->data().add_edges(kf->center(), kf->center() + 100*kf->orientation().row(2), ColorRGB::BLUE);
    viewer->selected_data_index = push_overlay_id;
  }


  viewer->selected_data_index = cage_mesh_overlay_id;
  {
    viewer->data().clear();
    viewer->data().set_face_based(true);
    viewer->data().set_mesh(state.cage.vertices(), state.cage.faces());
    viewer->selected_data_index = push_overlay_id;
  }

  return ret;
}

