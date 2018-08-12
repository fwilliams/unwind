#include "endpoint_selection_plugin.h"

#include <unordered_set>
#include <cmath>

#include <igl/unproject_onto_mesh.h>
#include <igl/boundary_facets.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>

#include <utils/colors.h>
#include <utils/utils.h>


static bool validate_endpoint_pairs(const std::vector<std::array<int, 2>>& endpoints, const Eigen::VectorXi& components) {
  bool success = true;
  std::unordered_set<int> computed_components;

  for (int i = 0; i < endpoints.size(); i++) {
    const int c1 = components[endpoints[i][0]];
    const int c2 = components[endpoints[i][0]];
    if (c1 != c2) {
      success = false;
      break;
    }
    if (computed_components.find(c1) != computed_components.end()) {
      success = false;
      break;
    } else {
      computed_components.insert(c1);
    }
  }

  return success;
}


static void compute_skeleton(const Eigen::MatrixXd& TV, const Eigen::MatrixXd& TT,
                             const Eigen::VectorXd normalized_distances,
                             const std::vector<std::array<int, 2>>& endpoint_pairs,
                             const Eigen::VectorXi& connected_components,
                             Eigen::MatrixXd& skeleton_vertices) {
}


EndPoint_Selection_Menu::EndPoint_Selection_Menu(State& state)
  : state(state)
{}

void EndPoint_Selection_Menu::initialize() {
  for (int i = viewer->data_list.size()-1; i > 0; i++) {
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

  current_endpoints = { -1, -1 };
  selecting_endpoints = false;
  extracting_skeleton = false;
  done_extracting_skeleton = false;
}


bool EndPoint_Selection_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();
  const Eigen::MatrixXd& TV = state.extracted_volume.TV;

  int push_mesh_id = viewer->selected_data_index;
  viewer->selected_data_index = points_overlay_id;
  viewer->data().clear();

  if (selecting_endpoints) {
    for (int i = 0; i < current_endpoint_idx; i++) {
      const int vid = current_endpoints[i];
      viewer->data().add_points(TV.row(vid), i == 0 ? ColorRGB::GREEN : ColorRGB::RED);
    }
  }

  for (int i = 0; i < endpoint_pairs.size(); i++) {
    std::array<int, 2> ep = endpoint_pairs[i];
    viewer->data().add_points(TV.row(ep[0]), ColorRGB::GREEN);
    viewer->data().add_points(TV.row(ep[1]), ColorRGB::RED);
  }

  viewer->selected_data_index = push_mesh_id;
  return ret;
}

bool EndPoint_Selection_Menu::post_draw() {
  bool ret = FishUIViewerPlugin::post_draw();
  bool _menu_visible = true;

  int width;
  int height;

  glfwGetWindowSize(viewer->window, &width, &height);
  ImGui::SetNextWindowPos(ImVec2(.0f, .0f), ImGuiSetCond_Always);
  ImGui::SetNextWindowSize(ImVec2(int(width*0.2), height), ImGuiSetCond_Always);
  ImGui::Begin("", &_menu_visible,
               ImGuiWindowFlags_NoSavedSettings |
               ImGuiWindowFlags_AlwaysAutoResize);

  if (done_extracting_skeleton) {
    state.application_state = Application_State::BoundingPolygon;
  }

  if (extracting_skeleton) {
    ImGui::OpenPopup("Extracting Skeleton");
    ImGui::BeginPopupModal("Extracting Skeleton");
    ImGui::Text("Extracting Fish Skeleton. Please wait, this may take a few seconds.");
    ImGui::NewLine();
    ImGui::EndPopup();
  }

  if (bad_selection) {
    ImGui::OpenPopup("Invalid Endpoint Selection");
    ImGui::BeginPopupModal("Invalid Endpoint Selection");
    ImGui::Text("%s", bad_selection_error_message.c_str());
    ImGui::NewLine();
    ImGui::Separator();
    if (ImGui::Button("OK")) {
      bad_selection = false;
    }
    ImGui::EndPopup();
  }

  std::string button_text("New Endpoint Pair");
  if (selecting_endpoints) {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

    if (current_endpoint_idx == 0) {
      button_text = std::string("Select Front");
    } else if (current_endpoint_idx == 1) {
      button_text = std::string("Select Back");
    } else {
      assert(false);
    }
  }

  int num_digits_ep = endpoint_pairs.size() > 0 ? (int) log10 ((double) endpoint_pairs.size()) + 1 : 1;
  if (endpoint_pairs.size() > 0) {
    ImGui::Text("Endpoint Pairs:");
    ImGui::Separator();
    for (int i = 0; i < endpoint_pairs.size(); i++) {
      int num_digits_i = (i+1) > 0 ? (int) log10 ((double) (i+1)) + 1 : 1;
      std::string label_text = "Endpoint ";
      for (int zi = 0; zi < num_digits_ep-num_digits_i; zi++) {
        label_text += std::string("0");
      }
      label_text += std::to_string(i);
      label_text += std::string(": ");
      std::string rm_button_text = std::string("Remove##") + std::to_string(i+1);
      ImGui::BulletText("%s", label_text.c_str());
      ImGui::SameLine();
      if (ImGui::Button(rm_button_text.c_str(), ImVec2(-1, 0))) {
        assert(i < endpoint_pairs.size());
        endpoint_pairs.erase(endpoint_pairs.begin() + i);
      }
    }
    ImGui::NewLine();
    ImGui::Separator();
  }

  if (ImGui::Button(button_text.c_str(), ImVec2(-1,0))) {
    selecting_endpoints = true;
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }

  if (selecting_endpoints) {
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();

    if (ImGui::Button("Cancel", ImVec2(-1,0))) {
      selecting_endpoints = false;
    }
  }
  ImGui::NewLine();
  ImGui::Separator();
  if (ImGui::Button("Back")) {
    // TODO: Back button
  }
  ImGui::SameLine();
  if (endpoint_pairs.size() == 0) {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }
  if (ImGui::Button("Next")) {
    extract_skeleton();
  }
  if (endpoint_pairs.size() == 0) {
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();
  }
  ImGui::End();

  ImGui::Render();
  return ret;
}

bool EndPoint_Selection_Menu::mouse_down(int button, int modifier) {
  bool ret = FishUIViewerPlugin::mouse_down(button, modifier);
  if (!selecting_endpoints) {
    return ret;
  }

  int fid;            // ID of the clicked face
  Eigen::Vector3f bc; // Barycentric coords of the click point on the face
  double x = viewer->current_mouse_x;
  double y = viewer->core.viewport(3) - viewer->current_mouse_y;

  if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y),
                              viewer->core.view * viewer->core.model,
                              viewer->core.proj, viewer->core.viewport,
                              state.extracted_volume.TV, state.extracted_volume.TF, fid, bc)) {

    int max;
    bc.maxCoeff(&max);
    int vid = state.extracted_volume.TF(fid, max);
    current_endpoints[current_endpoint_idx] = vid;

    current_endpoint_idx += 1;

    if (current_endpoint_idx >= 2) { // We've selected 2 endpoints
      endpoint_pairs.push_back(current_endpoints);

      if (current_endpoints[0] == current_endpoints[1]) {
        bad_selection = true;
        bad_selection_error_message = "Invalid Endpoints: Selected endpoints are the same.";
        endpoint_pairs.pop_back();
      } else if (!validate_endpoint_pairs(endpoint_pairs, state.extracted_volume.connected_components)) {
        bad_selection = true;
        bad_selection_error_message = "Invalid Endpoints: You can only have one endpoint pair per connected component.";
        endpoint_pairs.pop_back();
      }

      current_endpoints = { -1, -1 };
      current_endpoint_idx = 0;
      selecting_endpoints = false;
    }
  }

  return ret;
}


void EndPoint_Selection_Menu::extract_skeleton() {
  auto thread_fun = [&]() {
    extracting_skeleton = true;
    const Eigen::MatrixXd& TV = state.extracted_volume.TV;
    const Eigen::MatrixXi& TT = state.extracted_volume.TT;
    Eigen::VectorXd gdists;

    geodesic_distances(TV, TT, endpoint_pairs, gdists);
    scale_zero_one(gdists, state.geodesic_dists);
    extracting_skeleton = false;
  };

  extract_skeleton_thread = std::thread(thread_fun);
  extract_skeleton_thread.detach();
}
