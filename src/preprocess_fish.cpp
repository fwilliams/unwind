#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/marching_tets.h>
#include <igl/colormap.h>
#include <igl/harmonic.h>
#include <igl/slim.h>
#include <igl/grad.h>
#include <igl/adjacency_list.h>
#include <igl/massmatrix.h>
#include <igl/cotmatrix.h>

#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>

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
  igl::SLIMData m_sData;
  bool m_first_slim; // True if we need to initialize slim
  int m_num_slim_iterations = 10;
  int m_num_skel_verts = 20;

  int m_current_vertex = 0; // used to select up to where we are straightening
  float m_current_vertex_angle = 0; // used to control the up vector


  // Will be set to true if we need to redraw
  bool m_draw_state_changed = false;

  // Draw State Variables. TODO: Refactor these into a struct
  bool m_draw_isovalues = false;
  bool m_draw_surface = true;
  bool m_draw_tet_wireframe = false;
  bool m_draw_skeleton = false;
  bool m_draw_slim_res = false;

  // Used so we can restore draw state
  bool m_old_draw_isovalues = false;
  bool m_old_draw_surface = true;
  bool m_old_draw_tet_wireframe = false;
  bool m_old_draw_skeleton = false;
  bool m_old_draw_slim_res = false;

  void push_draw_state() {
    m_old_draw_isovalues = m_draw_isovalues;
    m_old_draw_skeleton = m_draw_skeleton;
    m_old_draw_surface = m_draw_surface;
    m_old_draw_tet_wireframe = m_draw_tet_wireframe;
    m_old_draw_slim_res = m_draw_slim_res;
  }

  void pop_draw_state() {
    m_draw_isovalues = m_old_draw_isovalues;
    m_draw_skeleton = m_old_draw_skeleton;
    m_draw_surface = m_old_draw_surface;
    m_draw_tet_wireframe = m_old_draw_tet_wireframe;
    m_draw_slim_res = m_old_draw_slim_res;
  }

  int nearest_vertex(const Eigen::RowVector3d& p) {
    int idx = -1;
    double min_norm = std::numeric_limits<double>::infinity();
    for (int k = 0; k < TV.rows(); k++) {
      double norm = (TV.row(k) - p).norm();
      if (norm < min_norm) {
        idx = k;
        min_norm = norm;
      }
    }
    return idx;
  }


  bool point_in_tet(int tet, const Eigen::RowVector3d pt) {
    // TODO: check if tet contains vertex
    using namespace  Eigen;

    auto sgn = [](double val) -> int {
      return (double(0) < val) - (val < double(0));
    };

    Matrix4Xd D0, D1, D2, D3, D4;
    RowVector3d v1 = TV.row(TT(tet, 0)), v2 = TV.row(TT(tet, 1));
    RowVector3d v3 = TV.row(TT(tet, 2)), v4 = TV.row(TT(tet, 3));

    D0 << v1[0], v1[1], v1[2], 1,
          v2[0], v2[1], v2[2], 1,
          v3[0], v3[1], v3[2], 1,
          v4[0], v4[1], v4[2], 1;

    RowVector4d pt_row(pt[0], pt[1], pt[2], 1);
    D1 = D0;
    D1.row(0) = pt_row;

    D2 = D0;
    D2.row(1) = pt_row;

    D3 = D0;
    D3.row(2) = pt_row;

    D4 = D0;
    D4.row(3) = pt_row;

    const double det0 = D0.determinant();
    assert(det0 != 0);
    const double det1 = D1.determinant();
    const double det2 = D2.determinant();
    const double det3 = D3.determinant();
    const double det4 = D4.determinant();

    return sgn(det1) == sgn(det2) && sgn(det1) == sgn(det3) && sgn(det1) == sgn(det4);
   }

  int containing_tet(const Eigen::RowVector3d& p) {
    for (int i = 0; i < TT.rows(); i++) {
      if (point_in_tet(i, p)) {
        return i;
      }
    }
    return -1;
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
      m_viewer.data().add_edges(TV, TV+Vfield*10, RowVector3d(0.1, 0.1, 1.0));
    } else if (m_draw_tet_wireframe && !m_draw_isovalues) {
      m_viewer.data().add_points(TV, RowVector3d(0.5, 0.5, 0.5));
      m_viewer.data().add_edges(m_TEV1, m_TEV2, RowVector3d(0.1, 0.1, 0.1));
    } else if (!m_draw_tet_wireframe && m_draw_isovalues) {
      m_viewer.data().add_points(TV, m_isovalColors);
      m_viewer.data().add_edges(TV, TV+Vfield*10, RowVector3d(0.1, 0.1, 1.0));
    }

    // Draw the skeleton
    if (m_draw_skeleton) {
      MatrixXd v1(skeletonV.rows()-1, 3), v2(skeletonV.rows()-1, 3);

      for (int i = 0; i < skeletonV.rows()-1; i++) {
        v1.row(i) = skeletonJoints.row(i);//TV.row(skeletonV[i]);
        v2.row(i) = skeletonJoints.row(i+1);//TV.row(skeletonV[i+1]);
      }
      m_viewer.data().add_edges(v1, v2, RowVector3d(0.1, 0.1, 1.0));
      m_viewer.data().add_points(v1, RowVector3d(0.0, 0.0, 1.0));
      m_viewer.data().add_points(v2.row(skeletonV.rows()-2),
                                  RowVector3d(0.0, 0.0, 1.0));

      if (m_sData.bc.rows() > 0) {
        for (int i = 0; i < skeletonV.rows()-1; i++) {
          v1.row(i) = m_sData.bc.row(i);
          v2.row(i) = m_sData.bc.row(i+1);
        }
        m_viewer.data().add_edges(v1, v2, RowVector3d(0.1, 1.0, 1.0));
        m_viewer.data().add_points(v1, RowVector3d(1.0, 0.0, 1.0));
        m_viewer.data().add_points(v2.row(skeletonV.rows()-2),
                                    RowVector3d(1.0, 0.0, 1.0));
      }
    }

    // Draw the straightened skeleton
    if (m_draw_slim_res) {
      m_viewer.data().add_points(m_sData.V_o, m_isovalColors);
      m_viewer.data().add_edges(m_TEV1_straight, m_TEV2_straight, RowVector3d(0.1, 0.1, 0.1));
    }
  }

  void extract_skeleton(int num_verts) {
    using namespace std;
    using namespace  Eigen;

    MatrixXd LV;
    MatrixXi LF;

    skeletonV.resize(num_verts);
    skeletonJoints.resize(num_verts, 3);
    int vcount = 0;

    for(int i = 1; i < num_verts; i++) {
      double isovalue = i * (1.0/num_verts);
      igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
      if (LV.rows() == 0) {
        continue;
      }
      Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();
      skeletonJoints.row(vcount) = C;
      skeletonV[vcount] = nearest_vertex(C);
      vcount += 1;
    }
    skeletonJoints.conservativeResize(vcount, 3);
    skeletonV.conservativeResize(vcount);
  }

  Eigen::MatrixXd Vfield;

  void compute_diffusion() {
    using namespace std;

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

    Eigen::SparseMatrix<double> G;
    igl::grad(TV, TT, G);
    Eigen::VectorXd g = G * isovals;
    cout << "g rows: " << g.rows() << endl;
    cout << "TT rows: " << TT.rows() << endl;
    cout << "g.rows / 3: " << g.rows() / 3 << endl;

    Eigen::Map<Eigen::MatrixXd> X(g.data(), TT.rows(), 3);
    X = X.rowwise().normalized();
    Eigen::VectorXd incidenceCount(TV.rows());
    Eigen::MatrixXd X_per_V(TV.rows(), 3);
    incidenceCount.setZero();
    X_per_V.setZero();

    for (int i = 0; i < TT.rows(); i++) {
      Eigen::RowVector3d gf = X.row(i);
      for (int v = 0; v < 4; v++) {
        const int vid = TT(i, v);
        incidenceCount[vid] += 1;
        const int n = incidenceCount[vid];
        X_per_V.row(vid) += (gf - X_per_V.row(vid)) / n;
      }
    }
    X_per_V = X_per_V.rowwise().normalized();
    Vfield = X_per_V;

    Eigen::MatrixXd JX = G * X_per_V;
    Eigen::VectorXd div = JX.col(0).segment(0, TT.rows()) + JX.col(1).segment(TT.rows(), TT.rows()) + JX.col(2).segment(2*TT.rows(), TT.rows());
    Eigen::VectorXd div_per_V(TV.rows());
    incidenceCount.setZero();
    div_per_V.setZero();

    for (int i = 0; i < TT.rows(); i++) {
      double divf = div[i];
      for (int v = 0; v < 4; v++) {
        const int vid = TT(i, v);
        incidenceCount[vid] += 1;
        const int n = incidenceCount[vid];
        div_per_V[vid] += (divf - div_per_V[vid]) / n;
      }
    }

    Eigen::SparseMatrix<double> L;
    igl::cotmatrix(TV, TT, L);

    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(L);
    isovals = solver.solve(div_per_V);
    const double isoval_min = isovals.minCoeff();
    const double isoval_max = isovals.maxCoeff();
    const double isoval_spread = isoval_max - isoval_min;
    const std::size_t n_isovals = isovals.size();
    isovals = (isovals - isoval_min * Eigen::VectorXd::Ones(n_isovals)) / isoval_spread;
  }

  void init_slim(int n, const Eigen::MatrixXd& V_0) {
    assert(n >= 1);
    using namespace std;
    Eigen::MatrixXd bc(n, 3);
    Eigen::RowVector3d v = TV.row(skeletonV[1]) - TV.row(skeletonV[0]);
    v /= v.norm();
    //    v = Eigen::RowVector3d(0, 0, 1);

    bc.row(0) = TV.row(skeletonV[0]);
    for (int i = 0; i < n-1; i++) {
      Eigen::RowVector3d s = TV.row(skeletonV[i]);
      Eigen::RowVector3d n = TV.row(skeletonV[i+1]);
      double d = (n - s).norm();
      bc.row(i+1) = bc.row(i) + v*d;
    }

    Eigen::MatrixXd TV_0 = V_0;
    Eigen::VectorXi b = skeletonV.segment(0, n); // skeletonV.segment(0, n+1);
    double soft_const_p = 1e5;
    m_sData.exp_factor = 5.0;
    slim_precompute(TV, TT, TV_0, m_sData, igl::SLIMData::EXP_CONFORMAL,
                    b, bc, soft_const_p);
  }

  Eigen::MatrixXd m_TEV1_straight;
  Eigen::MatrixXd m_TEV2_straight;
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
  Eigen::MatrixXd skeletonJoints;


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
          // function, the skeleton and SLIM
          isovals.resize(0);
          skeletonV.resize(0);
          m_first_slim = true;
          m_sData.V_o.resize(0, 3);
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
        m_draw_slim_res = false;
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
        ImGui::DragInt("Distance between bone verts", &m_num_skel_verts, 1.0f, 5, 100);
        if (ImGui::Button("Extract Skeleton", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            compute_diffusion();
            isoval_colors();
            cout << "Done!" << endl;
          }
          cout << "Extracting Skeleton..." << endl;
          extract_skeleton(m_num_skel_verts);

          // Need to recompute slim for a new skeleton
          m_first_slim = true;
          m_sData.V_o.resize(0, 3);

          m_draw_surface = false;
          m_draw_skeleton = true;
          m_draw_tet_wireframe = true;
          m_draw_isovalues = true;
          m_draw_slim_res = false;
          m_draw_state_changed = true;
          cout << "Done!" << endl;
        }

        if (skeletonV.rows() > 0) {
          if (ImGui::Button("Save Skeleton", ImVec2(-1,0))) {
          }
        }
      }
    }

    //
    // Skeleton Straightening Options
    //
    if (skeletonV.rows() > 0) {
      if (ImGui::CollapsingHeader("Straightening Options",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("SLIM Iterations", &m_num_slim_iterations, 1.0f, 1, 100);
        ImGui::DragInt("Up-To Vertex", &m_current_vertex, 1.0f, 0, m_num_skel_verts);
        ImGui::DragFloat("Up Angle", &m_current_vertex_angle, 0.5f, 0.0f, 360.0f);

        if (ImGui::Button("Straighten Skeleton", ImVec2(-1,0))) {
          if (m_first_slim) {
            cout << "Initializing SLIM..." << endl;
            init_slim(skeletonV.rows(), TV);
            cout << "Done" << endl;
            m_first_slim = false;
          }

          cout << "Running " << m_num_slim_iterations << " iterations of SLIM..." << endl;
          cout << "Initial SLIM energy is " << m_sData.energy << endl;
          for(int i = 0; i < m_num_slim_iterations; i++) {
            igl::slim_solve(m_sData, 1);
            cout << "Iteration " << i + 1 << endl;
          }
          cout << "Final SLIM energy is " << m_sData.energy << endl;
          cout << "Done!" << endl;
          tet_mesh_edges(m_sData.V_o, m_sData.F, m_TEV1_straight, m_TEV2_straight);
          m_draw_surface = false;
          m_draw_tet_wireframe = false;
          m_draw_skeleton = false;
          m_draw_isovalues = false;
          m_draw_slim_res = true;
          m_draw_state_changed = true;
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
      if (m_sData.V_o.rows() > 0) {
        if (ImGui::Checkbox("Draw Straightened Tet Mesh", &m_draw_slim_res)) {
          m_draw_state_changed = true;
        }
      }
    }
  }

};

int main(int argc, char *argv[]) {
  Viewer viewer;
  FishPreprocessingMenu menu("data/Sternopygus_arenatus-small.dat.out_.msh", viewer);
  viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
  return viewer.launch();
}
