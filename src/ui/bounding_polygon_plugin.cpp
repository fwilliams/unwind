#include "bounding_polygon_plugin.h"

#include <unordered_map>

#include <igl/edges.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include <utils/colors.h>
#include <utils/utils.h>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>


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

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


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

  Eigen::MatrixXd P1(state.skeleton_vertices.rows()-1, 3), P2(state.skeleton_vertices.rows()-1, 3);
  for (int i = 0; i < state.skeleton_vertices.rows()-1; i++) {
    P1.row(i) = state.skeleton_vertices.row(i);
    P2.row(i) = state.skeleton_vertices.row(i+1);
  }
  viewer->data().add_edges(P1, P2, ColorRGB::LIGHT_GREEN);
  viewer->data().point_size = 10.0;
//  viewer->data().add_points(state.skeleton_vertices, ColorRGB::GREEN);

  for (int i = 0; i < state.smooth_skeleton_vertices.rows()-1; i++) {
    P1.row(i) = state.smooth_skeleton_vertices.row(i);
    P2.row(i) = state.smooth_skeleton_vertices.row(i+1);
  }
  viewer->data().add_edges(P1, P2, ColorRGB::RED);
  viewer->data().point_size = 20.0;
//  viewer->data().add_points(state.smooth_skeleton_vertices, ColorRGB::RED);

  viewer->selected_data_index = push_mesh_id;

  // Initialize the 2d cross section widget
  widget_2d.initialize(viewer);
  make_bounding_cage();

}

bool Bounding_Polygon_Menu::make_bounding_cage() {
  // Compute initial bounding polyhedron
  const Eigen::MatrixXd& SV = state.smooth_skeleton_vertices;
  auto root = make_bounding_cage_component(0, SV.rows()-2, 0 /* level */);
  if (!root) {
    // TODO: Use the bounding box of the mesh instead
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    assert(false);
    return false;
  }

  make_bounding_cage_r(root);

  for (int i = 0; i < cage_components.size(); i++) {
    auto node = cage_components[i];
    Eigen::MatrixXd P1, P2;
    edge_endpoints(node->V, node->F, P1, P2);
    viewer->data().add_points(node->V, ColorRGB::BLUE);
    viewer->data().add_edges(P1, P2, ColorRGB::BLUE);
    viewer->data().add_edges(node->C, node->C + node->N*20.0, ColorRGB::CRIMSON);
  }
}

bool Bounding_Polygon_Menu::make_bounding_cage_r(
    std::shared_ptr<Bounding_Polygon_Menu::BoundingCageNode> root) {
  for (int i = 0; i < root->level; i++) { std::cout << "  "; }
  std::cout << root->level << ") make_bounding_cage" << std::endl;

  // If the node is in the cage, then we're done
  if (skeleton_in_cage(root->C, root->N, root->start, root->end)) {
    for (int i = 0; i < root->level; i++) { std::cout << "  "; }
    std::cout << root->level << ") cage contains skeleton" << std::endl;
    cage_components.push_back(root);
    return true;
  }

  // Otherwise split and recurse
  const int mid = root->start + (root->end - root->start) / 2;

  for (int i = 0; i < root->level; i++) { std::cout << "  "; }
  std::cout << root->level << ") " <<
               "root_start = " << root->start << ", " <<
               "root_end = " << root->end << ", " <<
               "root_mid = " << mid << std::endl;

  root->left = make_bounding_cage_component(root->start, mid, root->level+1);
  if (root->left) {
    for (int i = 0; i < root->level; i++) { std::cout << "  "; }
    std::cout << root->level << ") recurse left" << std::endl;
    make_bounding_cage_r(root->left);
  } else {
    for (int i = 0; i < root->level; i++) { std::cout << "  "; }
    std::cout << root->level << ") left was not recursable" << std::endl;
    root->left.reset();
    cage_components.push_back(root);
    return true;
  }

  root->right = make_bounding_cage_component(mid, root->end, root->level+1);
  if (root->right) {
    for (int i = 0; i < root->level; i++) { std::cout << "  "; }
    std::cout << root->level << ") recurse right" << std::endl;
    make_bounding_cage_r(root->right);
  } else {
    for (int i = 0; i < root->level; i++) { std::cout << "  "; }
    std::cout << root->level << ") right was not recursable" << std::endl;
    root->right.reset();
    cage_components.push_back(root);
    return true;
  }

  for (int i = 0; i < root->level; i++) { std::cout << "  "; }
  std::cout << root->level << ") returned normally" << std::endl;
  return true;
}

bool Bounding_Polygon_Menu::skeleton_in_cage(
    const Eigen::MatrixXd& CC,
    const Eigen::MatrixXd& CN,
    int start, int end) {
  assert(start < end);
  if ((end - start) <= 1) {
    return false;
  }

  const Eigen::MatrixXd& SV = state.smooth_skeleton_vertices;
  const int check_sign = sgn(CN.row(0).dot(SV.row(start+1)-CC.row(0)));
  for (int i = 0; i < end-start-1; i++) {
    const Eigen::RowVector3d V = SV.row(start+1+i);
    for (int j = 0; j < CN.rows(); j++) {
      const int sign = sgn(CN.row(j).dot(V-CC.row(j)));
      if (sign != check_sign) {
        return false;
      }
    }
  }

  return true;
}

std::shared_ptr<Bounding_Polygon_Menu::BoundingCageNode>
Bounding_Polygon_Menu::make_bounding_cage_component(int v1, int v2, int level) {
  std::shared_ptr<BoundingCageNode> node = std::make_shared<BoundingCageNode>();
  if ((v2 - v1) <= 1) {
    for (int i = 0; i < level-1; i++) { std::cout << "  "; }
    std::cout << level-1 << ") v1 = " << v1 << ", v2 = " << v2 << std::endl;
    node.reset();
    return node;
  }

  node->start = v1;
  node->end = v2;
  node->level = level;

  { // Construct the vertices of the bounding polyhedron
    auto p1 = plane_for_vertex(v1, 40.0);
    auto p2 = plane_for_vertex(v2, 40.0);
    Eigen::MatrixXd PV1 = std::get<0>(p1);
    Eigen::MatrixXd PV2 = std::get<0>(p2);
    Eigen::MatrixXd PV(2*PV1.rows(), 3);
    for (int i = 0; i < PV1.rows(); i++) {
      PV.row(i) = PV1.row(i);
      PV.row(i+PV1.rows()) = PV2.row(i);
    }
    igl::copyleft::cgal::convex_hull(PV, node->V, node->F);
    // If the planes self intersect, then return false
    if (PV.rows() != node->V.rows()) {
      node.reset();
      for (int i = 0; i < level-1; i++) { std::cout << "  "; }
      std::cout << level-1 << ") PV.rows() = " << PV.rows() << ", V.rows() = " << node->V.rows() << std::endl;
      return node;
    }
  }

  node->C.resize(node->F.rows(), 3);
  for (int i = 0; i < node->F.rows(); i++) {
    node->C.row(i) = (node->V.row(node->F(i, 0)) + node->V.row(node->F(i, 1)) + node->V.row(node->F(i, 2))) / 3;
  }
  igl::per_face_normals_stable(node->V, node->F, node->N);

  return node;
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

  if (ImGui::Button("< Prev")) {
    current_vertex_id = std::max(current_vertex_id - 1, 0);
  }
  ImGui::SameLine();
  if (ImGui::SliderInt("#vertexid", &current_vertex_id, 0, state.skeleton_vertices.rows() - 2)) {}
  ImGui::SameLine();
  if (ImGui::Button("Next >")) {
    current_vertex_id = std::min(current_vertex_id + 1, int(state.skeleton_vertices.rows() - 2));
  }

  ImGui::Checkbox("Show slice view", &show_slice_view);
  if (show_slice_view) {
    widget_2d.post_draw(PV, current_vertex_id);
  }
  ImGui::End();
  ImGui::Render();

  return ret;
}


bool Bounding_Polygon_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();

  glDisable(GL_CULL_FACE);
  Eigen::RowVector3d n =
      state.smooth_skeleton_vertices.row(current_vertex_id+1) -
      state.smooth_skeleton_vertices.row(current_vertex_id);
  n.normalize();
  Eigen::RowVector3d right(1, 0, 0);
  Eigen::RowVector3d up = right.cross(n);
  up.normalize();
  right = up.cross(n);

  Eigen::MatrixXi PF;
  make_plane(n, up, state.smooth_skeleton_vertices.row(current_vertex_id), 40.0, PV, PF);

  int push_overlay_id = viewer->selected_data_index;
  viewer->selected_data_index = points_overlay_id;
  viewer->data().clear();
  viewer->data().set_mesh(PV, PF);
  viewer->data().add_points(PV, ColorRGB::CRIMSON);
  viewer->data().point_size = 10.0;

  viewer->selected_data_index = push_overlay_id;

  return ret;
}


std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> Bounding_Polygon_Menu::plane_for_vertex(int vid, double radius) {
  Eigen::MatrixXd PV;
  Eigen::MatrixXi PF;
  Eigen::RowVector3d n1 =
      state.smooth_skeleton_vertices.row(vid+1) -
      state.smooth_skeleton_vertices.row(vid);
  n1.normalize();
  Eigen::RowVector3d right1(1, 0, 0);
  Eigen::RowVector3d up1 = right1.cross(n1);
  up1.normalize();
  right1 = up1.cross(n1);

  make_plane(n1, up1, state.smooth_skeleton_vertices.row(vid), radius, PV, PF);

  return std::make_tuple(PV, PF);
}
