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
#include <fstream>

#include "yixin_loader.h"

typedef igl::opengl::glfw::Viewer Viewer;

void save_skeleton(const std::string& filename, const Eigen::MatrixXd& TV,
                   const Eigen::MatrixXi& TF, const Eigen::MatrixXi& TT,
                   const Eigen::MatrixXi& SV, const Eigen::VectorXd& isovals) {
  using namespace std;

}

class FishPreprocessingMenu :
    public igl::opengl::glfw::imgui::ImGuiMenu {

  std::string m_current_model_filename;
  bool m_selecting_points = false;
  int m_current_point_idx = 0;
  std::array<int, 2> m_selected_end_coords{{-1, -1}};
  std::array<int, 2> m_tmp_selected_end_coords{{-1, -1}};
  int m_num_bones = 20;

  // Will be set to true if we need to redraw
  bool m_draw_state_changed = false;

  // Draw State Variables. TODO: Refactor these into a struct
  bool m_draw_isovalues = false;
  bool m_draw_surface = true;
  bool m_draw_tet_wireframe = false;
  bool m_draw_skeleton = false;

  // Used so we can restore draw state
  bool m_old_draw_isovalues = false;
  bool m_old_draw_surface = true;
  bool m_old_draw_tet_wireframe = false;
  bool m_old_draw_skeleton = false;

  void push_draw_state() {
    m_old_draw_isovalues = m_draw_isovalues;
    m_old_draw_skeleton = m_draw_skeleton;
    m_old_draw_surface = m_draw_surface;
    m_old_draw_tet_wireframe = m_draw_tet_wireframe;
  }

  void pop_draw_state() {
    m_draw_isovalues = m_old_draw_isovalues;
    m_draw_skeleton = m_old_draw_skeleton;
    m_draw_surface = m_old_draw_surface;
    m_draw_tet_wireframe = m_old_draw_tet_wireframe;
  }

  // Calculate the endpoints of edges for the tetmesh. Used for drawing.
  void tet_mesh_edges(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                      Eigen::MatrixXd& V1, Eigen::MatrixXd& V2) {
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
      const int v1 = edges[i].first;
      const int v2 = edges[i].second;
      deduped_edges.push_back(edges[i]);
      while (v1 == edges[i].first && v2 == edges[i].second) {
        i += 1;
      }
    }

    V1.resize(deduped_edges.size(), 3);
    V2.resize(deduped_edges.size(), 3);
    for (int i = 0; i < deduped_edges.size(); i++) {
      V1.row(i) = TV.row(deduped_edges[i].first);
      V2.row(i) = TV.row(deduped_edges[i].second);
    }
  }

  void isoval_colors() {
    using namespace Eigen;

    // Normalize the isovalues between 0 and 1 for the colormap
    const double isoval_min = isovals.minCoeff();
    const double isoval_max = isovals.maxCoeff();
    const double isoval_spread = isoval_max - isoval_min;
    const std::size_t n_isovals = isovals.size();
    VectorXd isovals_normalized =
        (isovals - isoval_min * VectorXd::Ones(n_isovals)) / isoval_spread;

    // Draw colored vertices of tet mesh based on their isovalue and black
    // lines connecting the vertices
    igl::colormap(igl::COLOR_MAP_TYPE_MAGMA, isovals_normalized,
                  false, m_isovalColors);
  }

  void draw_mesh() {
    using namespace Eigen;

    m_viewer.data().clear();
    m_viewer.data().set_mesh(TV, TF);
    m_viewer.data().show_lines = false;
    m_viewer.data().show_faces = false;

    if (m_draw_surface) {
      m_viewer.data().show_lines = true;
      m_viewer.data().show_faces = true;
    }

    // Draw selected endpoints
    m_viewer.data().point_size = 7.0;
    if (m_selecting_points) {
      for (int i = 0; i < m_current_point_idx; i++) {
        const int vid = m_tmp_selected_end_coords[i];
        m_viewer.data().add_points(TV.row(vid), RowVector3d(0.0, 1.0, 0.0));
      }
    } else if (m_selected_end_coords[0] != -1 &&
               m_selected_end_coords[1] != -1) {
      const int vid1 = m_selected_end_coords[0];
      const int vid2 = m_selected_end_coords[1];
      m_viewer.data().add_points(TV.row(vid1), RowVector3d(0.0, 1.0, 0.0));
      m_viewer.data().add_points(TV.row(vid2), RowVector3d(1.0, 0.0, 0.0));
    }

    // Draw the tet mesh wireframe and possibly isovalues
    if (m_draw_tet_wireframe && m_draw_isovalues) {
      m_viewer.data().add_points(TV, m_isovalColors);
      m_viewer.data().add_edges(m_TEV1, m_TEV2, RowVector3d(0.1, 0.1, 0.1));
    } else if (m_draw_tet_wireframe && !m_draw_isovalues) {
      m_viewer.data().add_points(TV, RowVector3d(0.5, 0.5, 0.5));
      m_viewer.data().add_edges(m_TEV1, m_TEV2, RowVector3d(0.1, 0.1, 0.1));
    } else if (!m_draw_tet_wireframe && m_draw_isovalues) {
      m_viewer.data().add_points(TV, m_isovalColors);
    }

    // Draw the skeleton
    if (m_draw_skeleton) {
      MatrixXd v1(skeletonV.rows()-1, 3), v2(skeletonV.rows()-1, 3);

      for (int i = 0; i < skeletonV.rows()-1; i++) {
        v1.row(i) = TV.row(skeletonV[i]);
        v2.row(i) = TV.row(skeletonV[i+1]);
      }
      m_viewer.data().add_edges(v1, v2, RowVector3d(0.1, 0.1, 1.0));
      m_viewer.data().add_points(v1, RowVector3d(0.0, 0.0, 1.0));
      m_viewer.data().add_points(v2.row(skeletonV.rows()-2),
                                  RowVector3d(0.0, 0.0, 1.0));
    }
  }

  void extract_skeleton(int num_samples) {
    using namespace std;

    Eigen::MatrixXd LV;
    Eigen::MatrixXi LF;
    skeletonV.resize(num_samples);
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
      skeletonV[vcount] = idx;
      vcount += 1;
    }

    skeletonV.conservativeResize(vcount);
  }

  void compute_diffusion() {
    Eigen::MatrixXi constraint_indices;
    Eigen::MatrixXd constraint_values;
    constraint_indices.resize(2, 1);
    constraint_values.resize(2, 1);
    constraint_indices(0, 0) = m_selected_end_coords[0];
    constraint_indices(1, 0) = m_selected_end_coords[1];
    constraint_values(0, 0) = 1.0;
    constraint_values(1, 0) = 0.0;

    igl::harmonic(TV, TT, constraint_indices,
                  constraint_values, 1, isovals);
  }

  Eigen::MatrixXd m_TEV1;
  Eigen::MatrixXd m_TEV2;
  Eigen::MatrixXd m_isovalColors;

  Viewer& m_viewer;

public:
  Eigen::MatrixXd TV;
  Eigen::MatrixXi TF;
  Eigen::MatrixXi TT;

  Eigen::VectorXd isovals;
  Eigen::VectorXi skeletonV;

  FishPreprocessingMenu(const std::string& filename, Viewer& viewer) : m_viewer(viewer) {
    using namespace std;

    m_current_model_filename = filename;
    load_yixin_tetmesh(filename, TV, TF, TT);
    cout << "Loaded " << filename << " with " << TV.rows() << " vertices, " <<
            TF.rows() << " boundary faces, and " << TT.rows() <<
            " tets." << endl;

    tet_mesh_edges(TV, TT, m_TEV1, m_TEV2);
    m_draw_state_changed = true;

    draw_mesh(); // We need this here for the viewer to draw anything... WTF

    viewer.plugins.push_back(this);
  }

  virtual bool pre_draw() override {
    if (m_draw_state_changed) {
      std::cout << "Draw State Changed!" << std::endl;
      draw_mesh();
      m_draw_state_changed = false;
    }
    return ImGuiMenu::pre_draw();
  }

  virtual bool mouse_down(int button, int modifier) override {
    using namespace std;
    if (m_selecting_points) {
      int fid;            // ID of the clicked face
      Eigen::Vector3f bc; // Barycentric coords of the click point on the face
      double x = m_viewer.current_mouse_x;
      double y = m_viewer.core.viewport(3) - m_viewer.current_mouse_y;
      if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y),
                                  m_viewer.core.view * m_viewer.core.model,
                                  m_viewer.core.proj, m_viewer.core.viewport,
                                  TV, TF, fid, bc)) {
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

          // You selected new endpoints so we need to recompute the harmonic
          // function and the skeleton
          isovals.resize(0);
          skeletonV.resize(0);
        }
        m_draw_state_changed = true;
      }
    }

    return ImGuiMenu::mouse_down(button, modifier);
  }

  virtual void draw_viewer_menu() override {
    using namespace std;

    string title_text = string("Current Model: ") + m_current_model_filename;
    ImGui::Text("%s", title_text.c_str());

    //
    // Interface for selecting endpoints
    //
    if (ImGui::CollapsingHeader("Endpoint Selection",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      bool disabled = false;
      string button_text("Select Endpoints");
      if (m_selecting_points) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha,
                            ImGui::GetStyle().Alpha * 0.5f);
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
        push_draw_state();
        m_draw_surface = true;
        m_draw_tet_wireframe = false;
        m_draw_skeleton = false;
        m_draw_isovalues = false;
        m_draw_state_changed = true;
      }
      if (disabled) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();

        if (ImGui::Button("Cancel", ImVec2(-1,0))) {
          m_selecting_points = false;
          m_current_point_idx = 0;
          pop_draw_state();
        }
      }
    }

    //
    // Skeleton extraction interface
    //
    if (m_selected_end_coords[0] >= 0 && m_selected_end_coords[1] >= 0) {
      if (ImGui::CollapsingHeader("Skeleton Extraction",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("Number of bones", &m_num_bones, 1.0f, 1, 100);
        if (ImGui::Button("Extract Skeleton", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            compute_diffusion();
            isoval_colors();
            cout << "Done!" << endl;
          }
          cout << "Extracting Skeleton with " <<
                  m_num_bones << " bones..." << endl;
          extract_skeleton(m_num_bones);
          m_draw_surface = false;
          m_draw_skeleton = true;
          m_draw_tet_wireframe = true;
          m_draw_isovalues = true;
          m_draw_state_changed = true;
          cout << "Done!" << endl;
        }
      }
    }

    //
    // Drawing options interface
    //
    if (ImGui::CollapsingHeader("Drawing Options",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Draw Mesh Surface", &m_draw_surface)) {
        m_draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Tetrahedral Wireframe",
                          &m_draw_tet_wireframe)) {
        m_draw_state_changed = true;
      }
      if (isovals.rows() > 0) {
        if (ImGui::Checkbox("Draw Isovalues", &m_draw_isovalues)) {
          m_draw_state_changed = true;
        }
      }
      if (skeletonV.rows() > 0) {
        if (ImGui::Checkbox("Draw Skeleton", &m_draw_skeleton)) {
          m_draw_state_changed = true;
        }
      }
    }

    if (ImGui::Button("Save Skeleton", ImVec2(-1,0))) {

    }
  }

};

int main(int argc, char *argv[]) {
  Viewer viewer;
  FishPreprocessingMenu menu("outReoriented_.msh", viewer);
  viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
  return viewer.launch();
}
