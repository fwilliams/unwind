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
#include <Eigen/SVD>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <array>
#include <unordered_map>

#include "yixin_loader.h"

typedef igl::opengl::glfw::Viewer Viewer;

void save_skeleton(const std::string& filename, const Eigen::MatrixXd& TV,
                   const Eigen::MatrixXi& TF, const Eigen::MatrixXi& TT,
                   const Eigen::MatrixXi& SV, const Eigen::VectorXd& isovals) {
  using namespace std;

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

void isoval_colors(const Eigen::VectorXd& isovals, Eigen::MatrixXd& isovalColors) {
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
                false, isovalColors);
}


// Compute approximate geodesic distance
void compute_diffusion(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, int tip, int tail, Eigen::VectorXd& isovals) {
  using namespace std;
  using namespace Eigen;

  typedef SparseMatrix<double> SparseMatrixXd;

  // Discrete Laplacian and Discrete Gradient operator
  SparseMatrixXd L;
  SparseMatrixXd G;
  igl::grad(TV, TT, G);
  igl::cotmatrix(TV, TT, L);

  SparseQR<SparseMatrixXd, COLAMDOrdering<int>> solver;
  MatrixXi constraint_indices;
  MatrixXd constraint_values;
  constraint_indices.resize(2, 1);
  constraint_values.resize(2, 1);
  constraint_indices(0, 0) = tail;
  constraint_indices(1, 0) = tip;
  constraint_values(0, 0) = 1.0;
  constraint_values(1, 0) = 0.0;
  igl::harmonic(TV, TT, constraint_indices,
                constraint_values, 1, isovals);

//  double dt = 0.000001;
//  SparseMatrixXd I(TV.rows(), TV.rows());
//  I.setIdentity();
//  igl::cotmatrix(TV, TT, L);
//  VectorXd u0(TV.rows());
//  u0.setZero();
//  u0[tip] = 1.0;

//  solver.compute(dt * (I - L));
//  isovals = solver.solve(u0);


  Eigen::VectorXd g = G * isovals;
  Eigen::Map<Eigen::MatrixXd> X(g.data(), TT.rows(), 3);
  X = X.rowwise().normalized();

  // Used to count number of incident faces to a vertex
  Eigen::VectorXd incidenceCount(TV.rows());
  incidenceCount.setZero();

  // Per vertex gradient is average over adjacent faces
  Eigen::MatrixXd X_per_V(TV.rows(), 3);
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


  Eigen::MatrixXd JX = G * X_per_V;
  Eigen::VectorXd div = JX.col(0).segment(0, TT.rows()) +
                        JX.col(1).segment(TT.rows(), TT.rows()) +
                        JX.col(2).segment(2*TT.rows(), TT.rows());

  // Per vertex divergence is average over adjacent faces
  Eigen::VectorXd div_per_V(TV.rows());
  div_per_V.setZero();
  incidenceCount.setZero();
  for (int i = 0; i < TT.rows(); i++) {
    double divf = div[i];
    for (int v = 0; v < 4; v++) {
      const int vid = TT(i, v);
      incidenceCount[vid] += 1;
      const int n = incidenceCount[vid];
      div_per_V[vid] += (divf - div_per_V[vid]) / n;
    }
  }

  solver.compute(L);
  isovals = solver.solve(div_per_V);
  const double isovals_min = isovals.minCoeff();
  const double isovals_max = isovals.maxCoeff();
  isovals -= isovals_min * Eigen::VectorXd::Ones(isovals.rows());
  isovals /= (isovals_max - isovals_min);

  using namespace std;
  cout << "Isovals min/max:" << endl;
  cout << isovals.minCoeff() << endl;
  cout << isovals.maxCoeff() << endl;
}


bool fit_plane(const Eigen::MatrixXd& P, Eigen::RowVector3d& c, Eigen::Matrix3d& frame) {
  using namespace std;
  if (P.rows() < 3) {
    return false;
  }

  c = P.colwise().sum() / P.rows();
  Eigen::MatrixXd A = P.rowwise() - c;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  frame = svd.matrixV();
  for (int i = 0; i < 3; i++) {
    double n = frame.col(i).norm();
    if (n > 1e-6) {
      frame.col(i) /= n;
    } else {
      return false;
    }
  }
  return true;
}

// Check if the point pt is in the tet at ID tet
bool point_in_tet(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, int tet, const Eigen::RowVector3d pt) {
  // TODO: check if tet contains vertex
  using namespace  Eigen;

  auto sgn = [](double val) -> int {
    return (double(0) < val) - (val < double(0));
  };

  Matrix4d D0, D1, D2, D3, D4;
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

// Return the index of the tet containing the point p or -1 if the vertex is in no tets
int containing_tet(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::RowVector3d& p) {
  for (int i = 0; i < TT.rows(); i++) {
    if (point_in_tet(TV, TT, i, p)) {
      return i;
    }
  }
  return -1;
}

// Return the index of the closest vertex to p
int nearest_vertex(const Eigen::MatrixXd& TV, const Eigen::RowVector3d& p) {
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





class FishPreprocessingMenu :
    public igl::opengl::glfw::imgui::ImGuiMenu {

  std::string m_current_model_filename;
  // Endpoint selection variables
  bool m_selecting_points = false;
  int m_current_endpoint_idx = 0;
  std::array<int, 2> m_selected_end_coords{{-1, -1}};
  std::array<int, 2> m_tmp_selected_end_coords{{-1, -1}};

  // Skeleton extraction variables
  int m_num_skel_verts = 20;

  // Level set variables
  float m_isovalue = 0.0;
  Eigen::MatrixXd isoV;
  Eigen::MatrixXi isoF;
  Eigen::RowVector3d isoC;
  Eigen::Matrix3d isoFrame;

  // Slim variables
  igl::SLIMData m_sData;
  int m_num_slim_iterations = 10;

  // Incremental straightening variables
  int m_current_vertex = 0; // used to select up to where we are straightening
  float m_current_vertex_angle = 0; // used to control the up vector

  // Drawing parameters
  float m_vfield_scale = 10.0f;
  float m_linewidth = 1.0;

  Eigen::MatrixXd m_oldTV;

  std::vector<int> m_containing_tet;
  std::vector<int> m_nearest_vertex;

  // Maps skeleton vertex to up vector angle and tet index
  std::unordered_map<int, std::pair<double, int>> m_up_vectors;

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

  void draw_mesh() {
    using namespace Eigen;

    m_viewer.data().line_width = m_linewidth;
    size_t old_data_index = m_viewer.selected_data_index;
    m_viewer.selected_data_index = 0;
    m_viewer.data().clear();
    m_viewer.data().point_size = 10.0;
    m_viewer.data().line_width = m_linewidth;
    m_viewer.selected_data_index = old_data_index;
    m_viewer.data().clear();
    m_viewer.data().set_mesh(TV, TF);
    m_viewer.data().show_lines = false;
    m_viewer.data().show_faces = false;

    if (m_draw_surface) {
      m_viewer.data().show_lines = true;
      m_viewer.data().show_faces = true;
    }

    m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(1000, 0, 0), RowVector3d(1, 0, 0));
    m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(0, 1000, 0), RowVector3d(0, 1, 0));
    m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(0, 0, 1000), RowVector3d(0, 0, 1));

    // Draw selected endpoints
    m_viewer.data().point_size = 7.0;
    if (m_selecting_points) {
      for (int i = 0; i < m_current_endpoint_idx; i++) {
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
    if (m_draw_tet_wireframe) {
      m_viewer.data().add_edges(m_TEV1, m_TEV2, RowVector3d(0.1, 0.1, 0.1));
    } else if (m_draw_tet_wireframe && !m_draw_isovalues) {
      m_viewer.data().add_points(TV, RowVector3d(0.5, 0.5, 0.5));
      m_viewer.data().add_edges(m_TEV1, m_TEV2, RowVector3d(0.1, 0.1, 0.1));
    }

    if (m_draw_isovalues) {
      m_viewer.data().add_points(TV, m_isovalColors);
      size_t old_data_index = m_viewer.selected_data_index;
      m_viewer.selected_data_index = 0;
      m_viewer.data().set_mesh(isoV, isoF);
      RowVector3d isoN = isoFrame.col(2);
      RowVector3d isoX = isoFrame.col(1);
      RowVector3d isoY = isoFrame.col(0);
      m_viewer.data().add_edges(isoC, isoC+isoX*m_vfield_scale, RowVector3d(220.0/255.0, 20.0/255.0, 60.0/255.0));
      m_viewer.data().add_edges(isoC, isoC+isoY*m_vfield_scale, RowVector3d(144.0/255.0, 238.0/255.0, 144.0/255.0));
      m_viewer.data().add_edges(isoC, isoC+isoN*m_vfield_scale, RowVector3d(70.0/255.0, 130.0/255.0, 180.0/255.0));
      m_viewer.data().add_points(isoC, RowVector3d(75.0/255.0, 0.0, 130.0/255.0));
      m_viewer.selected_data_index = old_data_index;
    }

    // Draw the skeleton
    if (m_draw_skeleton) {
      MatrixXd v1(skeletonJoints.rows()-1, 3), v2(skeletonJoints.rows()-1, 3);

      for (int i = 0; i < skeletonJoints.rows()-1; i++) {
        v1.row(i) = skeletonJoints.row(i);//TV.row(skeletonV[i]);
        v2.row(i) = skeletonJoints.row(i+1);//TV.row(skeletonV[i+1]);
      }
      m_viewer.data().add_edges(v1, v2, RowVector3d(0.1, 0.1, 1.0));
      m_viewer.data().add_points(v1, RowVector3d(0.0, 0.0, 1.0));
      m_viewer.data().add_points(v2.row(skeletonJoints.rows()-2),
                                  RowVector3d(0.0, 0.0, 1.0));

      if (m_sData.bc.rows() > 0) {
        for (int i = 0; i < m_sData.bc.rows(); i++) {
          m_viewer.data().add_points(m_sData.bc.row(i), RowVector3d(1, 0, 0));
        }
      }

//      Matrix3d frame = get_frame(m_current_vertex, m_current_vertex_angle);
//      RowVector3d right = frame.row(0);
//      RowVector3d up = frame.row(1);
//      RowVector3d dir = frame.row(2);

//      RowVector3d sv = skeletonJoints.row(m_current_vertex);
//      m_viewer.data().add_edges(sv, sv + dir*m_vfield_scale, RowVector3d(1, 0.3, 0.3));
//      m_viewer.data().add_edges(sv, sv + up*m_vfield_scale, RowVector3d(0.3, 1.0, 0.3));
//      m_viewer.data().add_edges(sv, sv + right*m_vfield_scale, RowVector3d(0.3, 0.3, 1.0));
    }
  }

  Eigen::Matrix3d get_frame(int idx, double angle) {
    using namespace Eigen;
    RowVector3d dir = (skeletonJoints.row(idx+1) - skeletonJoints.row(idx)).normalized();
    RowVector3d up(0, 1, 0);
    up -= dir*up.dot(dir);
    up = up.normalized();
    RowVector3d right = dir.cross(up);

    RowVector3d rotatedUp = right*sin(angle) + up*cos(angle);
    RowVector3d rotatedRight = rotatedUp.cross(dir);

    Matrix3d ret;
    ret.row(0) = rotatedRight;
    ret.row(1) = rotatedUp;
    ret.row(2) = dir;

    return ret;
  }

  void extract_skeleton(int num_verts) {
    using namespace std;
    using namespace  Eigen;

    MatrixXd LV;
    MatrixXi LF;
    m_up_vectors.clear();
    m_nearest_vertex.clear();
    m_containing_tet.clear();

    skeletonJoints.resize(num_verts, 3);
    int vcount = 0;

    unordered_map<int, int> tet_to_vertex;
    unordered_map<int, int> snap_to_vertex;

    for(int i = 1; i < num_verts; i++) {
      double isovalue = i * (1.0/num_verts);
      igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
      if (LV.rows() == 0) {
        cerr << "WARNING: Empty Level Set " << isovalue << endl;
        continue;
      }
      Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();

      int tet = containing_tet(TV, TT, C);
      if (tet < 0) {
        continue;
      }
      if (tet_to_vertex.find(tet) == tet_to_vertex.end()) {
        int nv = nearest_vertex(TV, C);
        if (snap_to_vertex.find(nv) == snap_to_vertex.end()) {
          skeletonJoints.row(vcount) = C;
          m_nearest_vertex.push_back(nv);
          m_containing_tet.push_back(tet);
          tet_to_vertex[tet] = 1;
          snap_to_vertex[nv] = 1;
          vcount += 1;
        }
      }
    }
    skeletonJoints.conservativeResize(vcount, 3);
  }

  void init_slim3(int up_to) {
    using namespace Eigen;
    VectorXi b(4*m_up_vectors.size() + skeletonJoints.rows() + 2);
    MatrixXd bc(4*m_up_vectors.size() + skeletonJoints.rows() + 2, 3);

    b[0] = m_selected_end_coords[1];
    bc.row(0) = RowVector3d(0, 0, 0);
    double dist = (skeletonJoints.row(0) - TV.row(m_selected_end_coords[1])).norm();
    int vcount = 1;

    for(int i = 0; i < up_to+1; i++) {
      auto it = m_up_vectors.find(i);
      if (it != m_up_vectors.end()) {
        int tet = it->second.second; // Index of containing tet
        const Matrix3d frame = get_frame(i, it->second.first);
        Vector3d ctr = Vector3d(0, 0, dist);
        RowVector3d v1 = frame * (TV.row(TT(tet, 0)) - skeletonJoints.row(i)).transpose() + ctr;
        RowVector3d v2 = frame * (TV.row(TT(tet, 1)) - skeletonJoints.row(i)).transpose() + ctr;
        RowVector3d v3 = frame * (TV.row(TT(tet, 2)) - skeletonJoints.row(i)).transpose() + ctr;
        RowVector3d v4 = frame * (TV.row(TT(tet, 3)) - skeletonJoints.row(i)).transpose() + ctr;

        b[vcount] = TT(tet, 0);
        bc.row(vcount) = v1;
        vcount += 1;

        b[vcount] = TT(tet, 1);
        bc.row(vcount) = v2;
        vcount += 1;

        b[vcount] = TT(tet, 2);
        bc.row(vcount) = v3;
        vcount += 1;

        b[vcount] = TT(tet, 3);
        bc.row(vcount) = v4;
        vcount += 1;
      } else {
        b[vcount] = m_nearest_vertex[i];
        bc.row(vcount) = RowVector3d(0, 0, dist);
        vcount += 1;
      }
      dist += (skeletonJoints.row(i+1) - skeletonJoints.row(i)).norm();
    }

    if (up_to == skeletonJoints.rows()-1) {
      b[vcount] = m_nearest_vertex[skeletonJoints.rows()-1];
      bc.row(vcount) = RowVector3d(0, 0, dist);
      dist += (TV.row(m_selected_end_coords[0]) - skeletonJoints.row(skeletonJoints.rows()-1)).norm();
      vcount += 1;
      b[vcount] = m_selected_end_coords[0];
      bc.row(vcount) = RowVector3d(0, 0, dist);
    }
    b.conservativeResize(vcount);
    bc.conservativeResize(vcount, 3);
    Eigen::MatrixXd TV_0 = TV;
    double soft_const_p = 1e5;
    m_sData.exp_factor = 5.0;
    slim_precompute(TV, TT, TV_0, m_sData, igl::SLIMData::EXP_CONFORMAL,
                    b, bc, soft_const_p);
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
  Eigen::MatrixXd skeletonJoints;


  FishPreprocessingMenu(const std::string& filename, Viewer& viewer) : m_viewer(viewer) {
    using namespace std;

    m_current_model_filename = filename;
    load_yixin_tetmesh(filename, TV, TF, TT);
    m_oldTV = TV;
    cout << "Loaded " << filename << " with " << TV.rows() << " vertices, " <<
            TF.rows() << " boundary faces, and " << TT.rows() <<
            " tets." << endl;

    tet_mesh_edges(TV, TT, m_TEV1, m_TEV2);
    m_draw_state_changed = true;

    m_viewer.append_mesh();
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
        assert(m_current_endpoint_idx <= 1);

        int max;
        bc.maxCoeff(&max);
        int vid = TF(fid, max);
        m_tmp_selected_end_coords[m_current_endpoint_idx] = vid;

        m_current_endpoint_idx += 1;
        if (m_current_endpoint_idx == 2) {
          m_current_endpoint_idx = 0;
          m_selecting_points = false;
          m_selected_end_coords[0] = m_tmp_selected_end_coords[0];
          m_selected_end_coords[1] = m_tmp_selected_end_coords[1];

          // You selected new endpoints so we need to recompute the harmonic
          // function, the skeleton and SLIM
          isovals.resize(0);
          skeletonJoints.resize(0, 0);
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
        if (m_current_endpoint_idx == 0) {
          button_text = string("Select Front");
        } else if (m_current_endpoint_idx == 1) {
          button_text = string("Select Back");
        } else {
          assert(false);
        }
      }
      if (ImGui::Button(button_text.c_str(), ImVec2(-1,0))) {
        m_selecting_points = true;
        m_current_endpoint_idx = 0;
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
          m_current_endpoint_idx = 0;
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
        ImGui::DragInt("Number of Joints", &m_num_skel_verts, 1.0f, 5, 100);
        if (ImGui::Button("Extract Skeleton", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            compute_diffusion(TV, TT, m_selected_end_coords[0], m_selected_end_coords[1], isovals);
            igl::marching_tets(TV, TT, isovals, double(m_isovalue), isoV, isoF);
            if (isoV.rows() >= 3) {
              fit_plane(isoV, isoC, isoFrame);
            }
            isoval_colors(isovals, m_isovalColors);
            cout << "Done!" << endl;
          }
          cout << "Extracting Skeleton..." << endl;
          extract_skeleton(m_num_skel_verts);

          // Need to recompute slim for a new skeleton
          m_sData.V_o.resize(0, 3);

          m_draw_surface = false;
          m_draw_skeleton = true;
          m_draw_tet_wireframe = true;
          m_draw_isovalues = true;
          m_draw_state_changed = true;
          cout << "Done!" << endl;
        }

        if (skeletonJoints.rows() > 0) {
          if (ImGui::Button("Save Skeleton", ImVec2(-1,0))) {
          }
        }
      }
    }

    //
    // Skeleton Straightening Options
    //
    if (skeletonJoints.rows() > 0) {
      if (ImGui::CollapsingHeader("Straightening Options",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("SLIM Iterations", &m_num_slim_iterations, 1.0f, 1, 100);
        if (ImGui::SliderInt("Up-To Vertex", &m_current_vertex, 0, skeletonJoints.rows()-1)) {
          m_draw_state_changed = true;
        }
        if (ImGui::SliderAngle("Up Angle", &m_current_vertex_angle, 0.0f, 360.0f)) {
          m_draw_state_changed = true;
        }
        if (ImGui::DragFloat("Isovalue", &m_isovalue, 0.01f, 0.0f, 1.0f)) {
          igl::marching_tets(TV, TT, isovals, double(m_isovalue), isoV, isoF);
          if (isoV.rows() >= 3) {
            fit_plane(isoV, isoC, isoFrame);
          }
          m_draw_state_changed = true;
        }

        if (ImGui::Button("Fix Up", ImVec2(-1, 0))) {
          double angle = m_current_vertex_angle;
          int tet = m_containing_tet[m_current_vertex];
          m_up_vectors[m_current_vertex] = make_pair(angle, tet);
        }
        if (ImGui::Button("Straighten Skeleton", ImVec2(-1,0))) {
          init_slim3(m_current_vertex);
          cout << "Running " << m_num_slim_iterations << " iterations of SLIM..." << endl;
          cout << "Initial SLIM energy is " << m_sData.energy << endl;
          for(int i = 0; i < m_num_slim_iterations; i++) {
            igl::slim_solve(m_sData, 1);
            cout << "Iteration " << i + 1 << endl;
          }
          cout << "Final SLIM energy is " << m_sData.energy << endl;
          cout << "Done!" << endl;
          TV = m_sData.V_o;
          tet_mesh_edges(TV, TT, m_TEV1, m_TEV2);
          //          tet_mesh_edges(m_sData.V_o, m_sData.F, m_TEV1_straight, m_TEV2_straight);
          m_draw_surface = true;
          m_draw_tet_wireframe = true;
          m_draw_skeleton = true;
          m_draw_isovalues = true;
          m_draw_state_changed = true;
        }
        if (ImGui::Button("Moar SLIM", ImVec2(-1,0))) {
          cout << "Running " << m_num_slim_iterations << " iterations of SLIM..." << endl;
          cout << "Initial SLIM energy is " << m_sData.energy << endl;
          for(int i = 0; i < m_num_slim_iterations; i++) {
            igl::slim_solve(m_sData, 1);
            cout << "Iteration " << i + 1 << endl;
          }
          cout << "Final SLIM energy is " << m_sData.energy << endl;
          cout << "Done!" << endl;
          TV = m_sData.V_o;
          tet_mesh_edges(TV, TT, m_TEV1, m_TEV2);
//          tet_mesh_edges(m_sData.V_o, m_sData.F, m_TEV1_straight, m_TEV2_straight);
          m_draw_surface = true;
          m_draw_tet_wireframe = true;
          m_draw_skeleton = true;
          m_draw_isovalues = true;
          m_draw_state_changed = true;
        }
      }
    }

    //
    // Drawing options interface
    //
    if (ImGui::CollapsingHeader("Drawing Options",
                                ImGuiTreeNodeFlags_DefaultOpen)) {

      if (ImGui::DragFloat("Line Width", &m_linewidth, 0.1f, 1.0f, 10.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::DragFloat("Vector Scale", &m_vfield_scale, 1.0f, 10.0f, 1000.0f)) {
        m_draw_state_changed = true;
      }
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
      if (skeletonJoints.rows() > 0) {
        if (ImGui::Checkbox("Draw Skeleton", &m_draw_skeleton)) {
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
