#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/marching_tets.h>
#include <igl/colormap.h>
#include <igl/harmonic.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <array>

#include "MshLoader.h"

typedef igl::opengl::glfw::Viewer Viewer;
// Visualize the tet mesh as a wireframe
void draw_tet_wireframe(Viewer& viewer,
                        const Eigen::MatrixXd& TV,
                        const Eigen::MatrixXi& TT,
                        const Eigen::VectorXd& isovals)
{
  // Make a black line for each edge in the tet mesh which we'll draw
  std::vector<std::pair<int, int>> edges;
  for (int i = 0; i < TT.rows(); i++)
  {
    int tf1 = TT(i, 0);
    int tf2 = TT(i, 1);
    int tf3 = TT(i, 2);
    int tf4 = TT(i, 2);
    edges.push_back(std::make_pair(tf1, tf2));
    edges.push_back(std::make_pair(tf1, tf3));
    edges.push_back(std::make_pair(tf1, tf4));
    edges.push_back(std::make_pair(tf2, tf3));
    edges.push_back(std::make_pair(tf2, tf4));
    edges.push_back(std::make_pair(tf3, tf4));
  }
  std::vector<std::pair<int, int>> deduped_edges;
  std::sort(edges.begin(), edges.end());
  for (int i = 0; i < edges.size();) {
    int v1 = edges[i].first;
    int v2 = edges[i].second;
    deduped_edges.push_back(edges[i]);
    while (v1 == edges[i].first && v2 == edges[i].second) {
      i += 1;
    }
  }

  Eigen::MatrixXd v1(deduped_edges.size(), 3), v2(deduped_edges.size(), 3);
  for (int i = 0; i < deduped_edges.size(); i++)
  {
    v1.row(i) = TV.row(deduped_edges[i].first);
    v2.row(i) = TV.row(deduped_edges[i].second);
  }


  Eigen::MatrixXd C;
  if (isovals.rows() > 0) {
    // Normalize the isovalues between 0 and 1 for the colormap
    const double isoval_min = isovals.minCoeff();
    const double isoval_max = isovals.maxCoeff();
    const double isoval_spread = isoval_max - isoval_min;
    const std::size_t n_isovals = isovals.size();
    Eigen::VectorXd isovals_normalized =
      (isovals - isoval_min * Eigen::VectorXd::Ones(n_isovals)) / isoval_spread;

    // Draw colored vertices of tet mesh based on their isovalue and black
    // lines connecting the vertices
    viewer.data().point_size = 5.0;
    igl::colormap(igl::COLOR_MAP_TYPE_MAGMA, isovals_normalized, false, C);
  } else {
    // If no isovalues provided, then just make them grey dots
    viewer.data().point_size = 5.0;
    C = Eigen::RowVector3d(0.5, 0.5, 0.5);
  }
  viewer.data().add_points(TV, C);
  viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.1, 0.1, 0.1));
}

void draw_skeleton(Viewer& viewer, const Eigen::MatrixXd& TV, const Eigen::VectorXi& SV) {
  Eigen::MatrixXd v1(SV.rows()-1, 3), v2(SV.rows()-1, 3);

  for (int i = 0; i < SV.rows()-1; i++) {
    v1.row(i) = TV.row(SV[i]);
    v2.row(i) = TV.row(SV[i+1]);
  }
  viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.8, 0.1, 0.4));
  viewer.data().add_points(v1, Eigen::RowVector3d(1.0, 0.0, 0.0));
  viewer.data().add_points(v2.row(SV.rows()-2), Eigen::RowVector3d(1.0, 0.0, 0.0));
}


void load_yixin_tetmesh(const std::string& filename, Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
  using namespace std;
  MshLoader vol_loader(filename);
  assert(vol_loader.m_nodes_per_element == 4);
  assert(vol_loader.m_data_size == 8);

  int tv_rows = vol_loader.m_nodes.rows() / 3;
  int tt_rows = vol_loader.m_elements.rows() / vol_loader.m_nodes_per_element;

  TT.resize(tt_rows, vol_loader.m_nodes_per_element);
  TV.resize(tv_rows, 3);

  std::vector<array<int, 3>> tris;

  int vcount = 0;
  for (int i = 0; i < vol_loader.m_nodes.rows(); i += 3) {
    TV.row(vcount++) = Eigen::RowVector3d(vol_loader.m_nodes[i], vol_loader.m_nodes[i+2], vol_loader.m_nodes[i+1]);
  }
  int tcount = 0;
  for (int i = 0; i < vol_loader.m_elements.rows(); i += vol_loader.m_nodes_per_element) {
    const int e1 = vol_loader.m_elements[i];
    const int e2 = vol_loader.m_elements[i+1];
    const int e3 = vol_loader.m_elements[i+2];
    const int e4 = vol_loader.m_elements[i+3];
    TT.row(tcount++) = Eigen::RowVector4i(e1, e2, e3, e4);
    array<int, 3> t1, t2, t3, t4;
    t1 = array<int, 3>{{ e1, e2, e3 }};
    t2 = array<int, 3>{{ e1, e2, e3 }};
    t3 = array<int, 3>{{ e2, e3, e4 }};
    t4 = array<int, 3>{{ e1, e3, e4 }};
    sort(t1.begin(), t1.end());
    sort(t2.begin(), t2.end());
    sort(t3.begin(), t3.end());
    sort(t4.begin(), t4.end());
    tris.push_back(t1);
    tris.push_back(t2);
    tris.push_back(t3);
    tris.push_back(t4);
  }

  int fcount;
  TF.resize(tris.size(), 3);
  sort(tris.begin(), tris.end());
  for (int i = 0; i < TF.rows();) {
    int v1 = tris[i][0], v2 = tris[i][1], v3 = tris[i][2];
    int count = 0;
    while (v1 == tris[i][0] && v2 == tris[i][1] && v3 == tris[i][2]) {
      i += 1;
      count += 1;
    }
    if (count == 1) {
      TF.row(fcount++) = Eigen::RowVector3i(v1, v2, v3);
    }
  }

  TF.conservativeResize(fcount, 3);
}

void extract_skeleton(int num_samples, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                      const Eigen::VectorXd isovals, Eigen::VectorXi& sk_verts) {
  using namespace std;

  Eigen::MatrixXd LV;
  Eigen::MatrixXi LF;
  sk_verts.resize(num_samples);
  int vcount = 0;
  for(int i = 1; i < num_samples; i++) {
    double isovalue = i * (1.0/num_samples);
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
    if (LV.rows() == 0) {
      continue;
    }
    Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();
    int idx = -1;
    double min_norm = std::numeric_limits<double>::infinity();
    for (int k = 0; k < TV.rows(); k++) {
      double norm = (TV.row(k) - C).norm();
      if (norm < min_norm) {
        idx = k;
        min_norm = norm;
      }
    }
    sk_verts[vcount] = idx;
    vcount += 1;
  }

  sk_verts.conservativeResize(vcount);
}

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu {
  std::string current_model;
  bool m_selecting_points = false;
  int m_current_point_idx = 0;
  std::array<int, 2> m_selected_end_coords{{-1, -1}};
  std::array<int, 2> m_tmp_selected_end_coords{{-1, -1}};
  int m_num_bones = 20;
  Viewer& m_viewer;

  bool m_draw_isovalues = false;
  bool m_draw_surface = true;
  bool m_draw_tet_wireframe = false;
  bool m_draw_skeleton = false;

  // Used so we can restore draw state
  bool m_old_draw_isovalues = false;
  bool m_old_draw_surface = true;
  bool m_old_draw_tet_wireframe = false;
  bool m_old_draw_skeleton = false;

  void pushDrawState() {
    m_old_draw_isovalues = m_draw_isovalues;
    m_old_draw_skeleton = m_draw_skeleton;
    m_old_draw_surface = m_draw_surface;
    m_old_draw_tet_wireframe = m_draw_tet_wireframe;
  }

  void popDrawState() {
    m_draw_isovalues = m_old_draw_isovalues;
    m_draw_skeleton = m_old_draw_skeleton;
    m_draw_surface = m_old_draw_surface;
    m_draw_tet_wireframe = m_old_draw_tet_wireframe;
  }

  // Will be set to true if we need to redraw
  bool draw_state_changed = false;

public:
  Eigen::MatrixXd TV;
  Eigen::MatrixXi TF;
  Eigen::MatrixXi TT;

  Eigen::VectorXd isovals;
  Eigen::VectorXi skeletonV;

  CustomMenu(const std::string& filename, Viewer& viewer) : m_viewer(viewer) {
    current_model = filename;
    load_yixin_tetmesh(filename, TV, TF, TT);
    redraw();
    viewer.callback_mouse_down = [&] (Viewer& v, int a, int b) -> bool { return mouse_down_callback(v, a, b); };
    viewer.plugins.push_back(this);
    viewer.callback_pre_draw = [&](Viewer&) -> bool {
      if (draw_state_changed) {
        redraw();
        draw_state_changed = false;
        m_viewer.draw();
      }
    };
  }

  bool mouse_down_callback(Viewer& viewer, int, int) {
    using namespace std;
    int fid;
    Eigen::Vector3f bc; // Barycentric coordinates of the click point on the face
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view * viewer.core.model,
                                viewer.core.proj, viewer.core.viewport, TV, TF, fid, bc)) {
      if (!m_selecting_points) {
        return false;
      }
      assert(m_current_point_idx <= 1);

      int max;
      bc.maxCoeff(&max);
      int vid = TF(fid, max);
      m_tmp_selected_end_coords[m_current_point_idx] = vid;

      m_current_point_idx += 1;
      if (m_current_point_idx == 2) {
        m_current_point_idx = 0;
        m_selecting_points = false;
        m_selected_end_coords[0] = m_tmp_selected_end_coords[0];
        m_selected_end_coords[1] = m_tmp_selected_end_coords[1];
        isovals.resize(0); // You selected endpoints so we need to recompute the harmonic function
        skeletonV.resize(0); // You need to recompute the skeleton too
      }
      draw_state_changed = true;
    }
  }

  void redraw() {
    m_viewer.data().clear();
    m_viewer.data().set_mesh(TV, TF);
    m_viewer.data().show_faces = false;
    m_viewer.data().show_lines = false;

    if (m_draw_surface) {
      m_viewer.data().show_faces = true;
      m_viewer.data().show_lines = true;
    }

    // Draw selected endpoints
    m_viewer.data().point_size = 7.0;
    if (m_selecting_points) {
      for (int i = 0; i < m_current_point_idx; i++) {
        const int vid = m_tmp_selected_end_coords[i];
        m_viewer.data().add_points(TV.row(vid), Eigen::RowVector3d(0.0, 1.0, 0.0));
      }
    } else if (m_selected_end_coords[0] != -1 && m_selected_end_coords[1] != -1) {
      const int vid1 = m_selected_end_coords[0];
      const int vid2 = m_selected_end_coords[1];
      m_viewer.data().add_points(TV.row(vid1), Eigen::RowVector3d(0.0, 1.0, 0.0));
      m_viewer.data().add_points(TV.row(vid2), Eigen::RowVector3d(1.0, 0.0, 0.0));
    }

    if (m_draw_tet_wireframe && m_draw_isovalues) {
      draw_tet_wireframe(m_viewer, TV, TT, isovals);
    } else if (m_draw_tet_wireframe && !m_draw_isovalues) {
      draw_tet_wireframe(m_viewer, TV, TT, Eigen::VectorXd());
    }

    if (m_draw_skeleton) {
      draw_skeleton(m_viewer, TV, skeletonV);
    }
  }

  virtual void draw_viewer_menu() override {
    using namespace std;

    string title_text = string("Current Model: ") + current_model;
    ImGui::Text(title_text.c_str());

    // Interface for selecting endpoints
    if (ImGui::CollapsingHeader("Endpoint Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool disabled = false;
      string button_text("Select Endpoints");
      if (m_selecting_points) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        disabled = true;
        if (m_current_point_idx == 0) {
          button_text = string("Select Front");
        } else if (m_current_point_idx == 1) {
          button_text = string("Select Back");
        } else {
          assert(false);
        }
      }
      if (ImGui::Button(button_text.c_str(), ImVec2(-1,0))) {
        m_selecting_points = true;
        m_current_point_idx = 0;
        pushDrawState();
        m_draw_surface = true;
        m_draw_tet_wireframe = false;
        m_draw_skeleton = false;
        m_draw_isovalues = false;
        draw_state_changed = true;
      }
      if (disabled) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();

        if (ImGui::Button("Cancel", ImVec2(-1,0))) {
          m_selecting_points = false;
          m_current_point_idx = 0;
          popDrawState();
        }
      }
    }

    if (m_selected_end_coords[0] >= 0 && m_selected_end_coords[1] >= 0) {
      if (ImGui::CollapsingHeader("Skeleton Extraction", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("Number of bones", &m_num_bones, 1.0f, 1, 100);
        if (ImGui::Button("Extract Skeleton", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            Eigen::MatrixXi constraint_indices;
            Eigen::MatrixXd constraint_values;
            constraint_indices.resize(2, 1);
            constraint_values.resize(2, 1);
            constraint_indices(0, 0) = m_selected_end_coords[0];
            constraint_indices(1, 0) = m_selected_end_coords[1];
            constraint_values(0, 0) = 1.0;
            constraint_values(1, 0) = 0.0;

            igl::harmonic(TV, TT, constraint_indices, constraint_values, 1, isovals);
            cout << "Done!" << endl;
          }
          cout << "Extracting Skeleton with " << m_num_bones << " bones..." << endl;
          extract_skeleton(m_num_bones, TV, TT, isovals, skeletonV);
          m_draw_surface = false;
          m_draw_skeleton = true;
          m_draw_tet_wireframe = true;
          m_draw_isovalues = true;
          draw_state_changed = true;
          cout << "Done!" << endl;
        }
      }
    }
    if (ImGui::CollapsingHeader("Drawing Options", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Draw Mesh Surface", &m_draw_surface)) {
        draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Tetrahedral Wireframe", &m_draw_tet_wireframe)) {
        draw_state_changed = true;
      }

      if (isovals.rows() > 0) {
        if (ImGui::Checkbox("Draw Isovalues", &m_draw_isovalues)) {
          draw_state_changed = true;
        }
      }
      if (skeletonV.rows() > 0) {
        if (ImGui::Checkbox("Draw Skeleton", &m_draw_skeleton)) {
          draw_state_changed = true;
        }
      }
    }
    if (ImGui::Button("Save Skeleton", ImVec2(-1,0))) {

    }
  }

  virtual void draw_custom_window() override {}
};

int main(int argc, char *argv[]) {
  using namespace Eigen;
  using namespace std;
  igl::opengl::glfw::Viewer viewer;
  CustomMenu menu("outReoriented_.msh", viewer);
  return viewer.launch();
}
