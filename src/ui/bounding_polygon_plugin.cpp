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
  // viewer->data().add_points(state.bounding_cage.skeleton_vertices(), ColorRGB::GREEN);

  for (int i = 0; i < state.cage.smooth_skeleton_vertices().rows()-1; i++) {
    P1.row(i) = state.cage.smooth_skeleton_vertices().row(i);
    P2.row(i) = state.cage.smooth_skeleton_vertices().row(i+1);
  }
  viewer->data().add_edges(P1, P2, ColorRGB::RED);
  viewer->data().point_size = 20.0;
  // viewer->data().add_points(state.bounding_cage.smooth_skeleton_vertices(), ColorRGB::RED);

  viewer->selected_data_index = push_mesh_id;

  // Initialize the 2d cross section widget
  widget_2d.initialize(viewer);
}


bool Bounding_Polygon_Menu::mouse_move(int mouse_x, int mouse_y) {
  return widget_2d.mouse_move(mouse_x, mouse_y);
}


bool Bounding_Polygon_Menu::mouse_down(int button, int modifier) {
  return widget_2d.mouse_down(button, modifier);
}


bool Bounding_Polygon_Menu::mouse_up(int button, int modifier) {
  return widget_2d.mouse_up(button, modifier);
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

  if (ImGui::Button("Split")) {
    state.cage.split(current_cut_index);
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
  viewer->data().clear();

  int cell_count_forward = 0;
  for (const BoundingCage::Cell& cell : state.cage.cells) {
    Eigen::MatrixXd P1, P2;
    edge_endpoints(cell.vertices(), cell.faces(), P1, P2);
    viewer->data().add_edges(P1, P2, ColorRGB::GREEN);
    cell_count_forward += 1;
  }

  int cell_count_reverse = 0;
  for (auto cell = state.cage.cells.rbegin(); cell != state.cage.cells.rend(); --cell) {
    Eigen::MatrixXd P1, P2;
    edge_endpoints(cell->vertices(), cell->faces(), P1, P2);
    viewer->data().add_edges(P1, P2, ColorRGB::RED);
    cell_count_reverse += 1;
  }

//  std::cout << "count_fwd: " << cell_count_forward << ", count_rev: " << cell_count_reverse << std::endl;
  for (BoundingCage::KeyFrame& kf : state.cage.keyframes) {
    viewer->data().add_points(kf.points_3d(), ColorRGB::DARK_GRAY);
  }

  for (auto kf = state.cage.keyframes.rbegin(); kf != state.cage.keyframes.rend(); --kf) {
    viewer->data().add_points(kf->points_3d(), ColorRGB::CRIMSON);
    viewer->data().add_points(kf->center(), ColorRGB::GREEN);
  }

  viewer->data().point_size = 10.0;
  Eigen::MatrixXd pts = state.cage.vertices_3d_for_index(current_cut_index);
  std::pair<Eigen::RowVector3d, Eigen::RowVector3d> plane =
      state.cage.plane_for_index(current_cut_index);
  viewer->data().add_points(plane.first, ColorRGB::RED);
  viewer->data().add_points(pts, ColorRGB::LIGHT_GREEN);
  viewer->data().add_edges(plane.first, plane.first + 100*plane.second, ColorRGB::RED);
  viewer->selected_data_index = push_overlay_id;

  return ret;
}

