#include "bounding_polygon_plugin.h"

#include <igl/edges.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include <utils/colors.h>


static void make_plane(const Eigen::RowVector3d& normal, const Eigen::RowVector3d& up,
                       const Eigen::RowVector3d& ctr, double scale,
                       Eigen::MatrixXd& V, Eigen::MatrixXi& F) {

  Eigen::RowVector3d n = normal;
  n.normalize();
  Eigen::RowVector3d u = up;
  u.normalize();
  Eigen::RowVector3d r = n.cross(u);

  V.resize(4, 3);

  V.row(0) = ctr + scale*(0.5*r + 0.5*u);
  V.row(1) = ctr + scale*(-0.5*r + 0.5*u);
  V.row(2) = ctr + scale*(-0.5*r - 0.5*u);
  V.row(3) = ctr + scale*(0.5*r - 0.5*u);

  F.resize(2, 3);
  F.row(0) = Eigen::RowVector3i(0, 1, 3);
  F.row(1) = Eigen::RowVector3i(1, 2, 3);
}

Bounding_Polygon_Menu::Bounding_Polygon_Menu(State& state)
  : state(state)
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

  Eigen::MatrixXd P1(state.skeleton_vertices.rows()-1, 3), P2(state.skeleton_vertices.rows()-1, 3);
  for (int i = 0; i < state.skeleton_vertices.rows()-1; i++) {
    P1.row(i) = state.skeleton_vertices.row(i);
    P2.row(i) = state.skeleton_vertices.row(i+1);
  }
  viewer->data().add_edges(P1, P2, ColorRGB::LIGHT_GREEN);
  viewer->data().point_size = 10.0;
  viewer->data().add_points(state.skeleton_vertices, ColorRGB::GREEN);

  viewer->selected_data_index = push_mesh_id;
}


bool Bounding_Polygon_Menu::post_draw() {
  bool ret = FishUIViewerPlugin::post_draw();

  int width;
  int height;

  glfwGetWindowSize(viewer->window, &width, &height);

  ImGui::SetNextWindowBgAlpha(0.5);
  ImGui::SetNextWindowPos(ImVec2(.0f, int(0.8*height)), ImGuiSetCond_Always);
  ImGui::SetNextWindowSize(ImVec2(width, int(height*0.2)), ImGuiSetCond_Always);
  ImGui::Begin("Select Boundary", NULL,
               ImGuiWindowFlags_NoSavedSettings |
               ImGuiWindowFlags_AlwaysAutoResize |
               ImGuiWindowFlags_NoTitleBar);
  ImGui::Text("The UI will go here");

  if (ImGui::Button("< Prev")) {
    current_vertex_id = std::max(current_vertex_id-1, 0);
  }
  ImGui::SameLine();
  if (ImGui::SliderInt("", &current_vertex_id, 0, state.skeleton_vertices.rows()-2)) {
  }
  ImGui::SameLine();
  if (ImGui::Button("Next >")) {
    current_vertex_id = std::min(current_vertex_id+1, int(state.skeleton_vertices.rows()-2));
  }
  ImGui::End();

  ImGui::Render();
  return ret;
}


bool Bounding_Polygon_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();

  glDisable(GL_CULL_FACE);
  Eigen::RowVector3d n =
      state.skeleton_vertices.row(current_vertex_id+1) - state.skeleton_vertices.row(current_vertex_id);
  n.normalize();

  Eigen::RowVector3d right(1, 0, 0);

  Eigen::RowVector3d up = right.cross(n);
  up.normalize();

  Eigen::MatrixXd PV;
  Eigen::MatrixXi PF;
  make_plane(n, up, state.skeleton_vertices.row(current_vertex_id), 40.0, PV, PF);

  int push_overlay_id = viewer->selected_data_index;
  viewer->selected_data_index = points_overlay_id;
  viewer->data().clear();
  viewer->data().set_mesh(PV, PF);
  viewer->data().add_points(PV, ColorRGB::CRIMSON);
  viewer->data().point_size = 10.0;

  viewer->selected_data_index = push_overlay_id;

  return ret;
}
