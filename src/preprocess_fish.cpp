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
#include <GLFW/glfw3.h>

#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>
#include <Eigen/SVD>
#include <Eigen/CholmodSupport>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <thread>
#include <mutex>
#include <atomic>
#include <tuple>

#include "yixin_loader.h"
#include "auto.h"

typedef igl::opengl::glfw::Viewer Viewer;

namespace ColorRGB {
  const Eigen::RowVector3d RED = Eigen::RowVector3d(1, 0, 0);
  const Eigen::RowVector3d GREEN = Eigen::RowVector3d(0, 1, 0);
  const Eigen::RowVector3d BLUE = Eigen::RowVector3d(0, 0, 1);
  const Eigen::RowVector3d DARK_GRAY = Eigen::RowVector3d(0.1, 0.1, 0.1);
  const Eigen::RowVector3d GRAY = Eigen::RowVector3d(0.5, 0.5, 0.5);
  const Eigen::RowVector3d STEEL_BLUE = Eigen::RowVector3d(70.0/255.0, 130.0/255.0, 180.0/255.0);
  const Eigen::RowVector3d LIGHT_GREEN = Eigen::RowVector3d(144.0/255.0, 238.0/255.0, 144.0/255.0);
  const Eigen::RowVector3d CRIMSON = Eigen::RowVector3d(220.0/255.0, 20.0/255.0, 60.0/255.0);
  const Eigen::RowVector3d BLACK = Eigen::RowVector3d(0, 0, 0);
  const Eigen::RowVector3d DARK_MAGENTA = Eigen::RowVector3d(139.0/255.0, 0.0/255.0, 139.0/255.0);
}


// Calculate the endpoints of edges for the tetmesh. Used for drawing.
void tet_mesh_edges(const Eigen::MatrixXd& TV,
                    const Eigen::MatrixXi& TT,
                    Eigen::MatrixXd& V1,
                    Eigen::MatrixXd& V2) {
  // Make a black line for each edge in the tet mesh which we'll draw
  std::vector<std::pair<int, int>> edges;
  for (int i = 0; i < TT.rows(); i++) {
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

// Calculate the colors for a set of isovalues
void isoval_colors(const Eigen::VectorXd& isovals,
                   Eigen::MatrixXd& isovalColors) {
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
void diffusion_distances(const Eigen::MatrixXd& TV,
                       const Eigen::MatrixXi& TT,
                       const std::array<int, 2>& endpoints,
                       Eigen::VectorXd& isovals) {
  using namespace std;
  using namespace Eigen;

  typedef SparseMatrix<double> SparseMatrixXd;

  // Discrete Laplacian and Discrete Gradient operator
  SparseMatrixXd G;
  cout << "Computing Discrete Gradient" << endl;
  igl::grad(TV, TT, G);
  cout << "Done" << endl;

  SimplicialLDLT<SparseMatrixXd> solver;

  MatrixXi constraint_indices;
  MatrixXd constraint_values;
  constraint_indices.resize(2, 1);
  constraint_values.resize(2, 1);
  constraint_indices(0, 0) = endpoints[1];
  constraint_indices(1, 0) = endpoints[0];
  constraint_values(0, 0) = 1.0;
  constraint_values(1, 0) = 0.0;

  cout << "Computing harmonic..." << endl;
  igl::harmonic(TV, TT, constraint_indices,
                constraint_values, 1, isovals);
  cout << "Done." << endl;
  double isovals_min = isovals.minCoeff();
  double isovals_max = isovals.maxCoeff();
  isovals -= isovals_min * Eigen::VectorXd::Ones(isovals.rows());
  isovals /= (isovals_max - isovals_min);


  VectorXd g = G*isovals;
  Map<MatrixXd> V(g.data(), TT.rows(), 3);
  V.rowwise().normalize();

  solver.compute(G.transpose()*G);
  isovals = solver.solve(G.transpose()*g);
  isovals_min = isovals.minCoeff();
  isovals_max = isovals.maxCoeff();
  isovals -= isovals_min * Eigen::VectorXd::Ones(isovals.rows());
  isovals /= (isovals_max - isovals_min);
}

// Fit a plane to the set of points P and return the center c and a frame
bool fit_plane(const Eigen::MatrixXd& P,
               Eigen::RowVector3d& c,
               Eigen::Matrix3d& frame) {
  using namespace std;
  if (P.rows() < 3) {
    return false;
  }

  c = P.colwise().sum() / P.rows();
  Eigen::MatrixXd A = P.rowwise() - c;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::Matrix3d f = svd.matrixV();
  for (int i = 0; i < 3; i++) {
    double n = f.col(i).norm();
    if (n > 1e-6) {
      f.col(i) /= n;
    } else {
      return false;
    }
  }
  frame = f.transpose();
  return true;
}

// Check if the point pt is in the tet at ID tet
bool point_in_tet(const Eigen::MatrixXd& TV,
                  const Eigen::MatrixXi& TT,
                  const Eigen::RowVector3d& pt,
                  int tet) {
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
int containing_tet(const Eigen::MatrixXd& TV,
                   const Eigen::MatrixXi& TT,
                   const Eigen::RowVector3d& p) {
  for (int i = 0; i < TT.rows(); i++) {
    if (point_in_tet(TV, TT, p, i)) {
      return i;
    }
  }
  return -1;
}

// Return the index of the closest vertex to p
int nearest_vertex(const Eigen::MatrixXd& TV,
                   const Eigen::RowVector3d& p) {
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

// Sample the level sets to extract a skeleton
void extract_skeleton(const Eigen::MatrixXd& TV,
                      const Eigen::MatrixXi& TT,
                      const Eigen::VectorXd& isovals,
                      int num_verts,
                      Eigen::MatrixXd& joints) {
  using namespace std;
  using namespace  Eigen;

  MatrixXd LV;
  MatrixXi LF;

  joints.resize(num_verts, 3);
  int vcount = 0;

  for(int i = 1; i < num_verts; i++) {
    double isovalue = i * (1.0/num_verts);
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
    if (LV.rows() == 0) {
      continue;
    }
    Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();
    joints.row(vcount) = C;
    vcount += 1;
  }
  joints.conservativeResize(vcount, 3);
}



struct DrawState {
  Eigen::MatrixXd m_TV; // Tet mesh vertices
  Eigen::MatrixXi m_TT; // Tet mesh tets
  Eigen::MatrixXi m_TF; // Tet mesh faces
  Eigen::MatrixXd m_TEV1, m_TEV2; // Endoints of tet mesh edges
  Eigen::MatrixXd m_isoV;       // Vertices in the level set
  Eigen::MatrixXi m_isoF;       // Faces in the level set
  Eigen::VectorXi m_isoT;       // Indices into TT of tetrahedra in the level set
  Eigen::RowVector3d m_isoC;    // Centroid of the level set
  Eigen::MatrixXd m_isoTV;      // Vertices of tetrahedra in level set
  Eigen::MatrixXi m_isoTT;      // Tetrahedra in level set
  Eigen::MatrixXd m_joints;     // Skeleton Joints
  Eigen::MatrixXd m_bTV;        // Positions of SLIM boundary constraints
  Eigen::MatrixXd m_bV;         // Target positions of SLIM boundary constraints

  void update_tet_mesh(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT) {
    m_TV = TV;
    m_TT = TT;
    tet_mesh_edges(m_TV, m_TT, m_TEV1, m_TEV2);
  }

  void update_skeleton(const Eigen::VectorXd& isovals, int num_verts) {
    extract_skeleton(m_TV, m_TT, isovals, num_verts, m_joints);
  }

  void update_constraint_points(const Eigen::VectorXi& b, const Eigen::MatrixXd& bc) {
    using namespace std;
    m_bTV.resize(b.rows(), 3);
    m_bV.resize(b.rows(), 3);
    for (int i = 0; i < m_bV.rows(); i++) {
      m_bTV.row(i) = m_TV.row(b[i]);
      m_bV.row(i) = bc.row(i);
    }
  }

  void update_isovalue(const Eigen::VectorXd& isovals, double isoval) {
    igl::marching_tets(m_TV, m_TT, isovals, isoval, m_isoV, m_isoF, m_isoT);
    m_isoTV.resize(m_isoT.rows()*4, 3);
    m_isoTT.resize(m_isoT.rows(), 4);
    m_isoC = m_isoV.colwise().sum() / m_isoV.rows();
    int tvcount = 0;
    for (int i = 0; i < m_isoT.rows(); i++) {
      for (int v = 0; v < 4; v++) {
        m_isoTT(i, v) = tvcount;
        m_isoTV.row(tvcount++) = m_TV.row(m_TT(m_isoT[i], v));
      }
    }
  }
};


struct ConstraintState {
  std::vector<int> m_bone_constraints_idx;
  std::vector<Eigen::RowVector3d> m_bone_constraints_pos;
  std::vector<int> m_orientation_constraints_idx;
  std::vector<Eigen::RowVector3d> m_orientation_constraints_pos;

  std::vector<double> m_level_set_isovalues;
  std::vector<double> m_level_set_distances;

  // Indices of tets which can possibly have rotation constraints
  std::vector<int> m_constrainable_tets_idx;
  std::unordered_map<int, std::tuple<int, double, bool>> m_tet_constraints;

  std::pair<Eigen::Matrix3d, Eigen::RowVector3d> frame_for_tet(const Eigen::MatrixXd& TV,
                                                               const Eigen::MatrixXi& TT,
                                                               const Eigen::VectorXd& isovals,
                                                               int idx, double angle, bool flip_x) {
    using namespace Eigen;
    Eigen::MatrixXd LV;
    Eigen::MatrixXi LF;

    RowVector3d fwd_dir;

    const int N_LOOKAHEAD = 4;
    if (idx < m_constrainable_tets_idx.size() - N_LOOKAHEAD) {
      igl::marching_tets(TV, TT, isovals, m_level_set_isovalues[idx], LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(TV, TT, isovals, m_level_set_isovalues[idx+N_LOOKAHEAD], LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    } else {
      igl::marching_tets(TV, TT, isovals, m_level_set_isovalues[idx-N_LOOKAHEAD], LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(TV, TT, isovals, m_level_set_isovalues[idx], LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    }

    RowVector3d up_dir = RowVector3d(0, 1, 0);
    up_dir -= fwd_dir*(up_dir.dot(fwd_dir));
    up_dir = up_dir.normalized();
    RowVector3d right_dir = up_dir.cross(fwd_dir);
    if (flip_x) {
      right_dir *= -1;
    }

    Matrix3d frame;
    frame.row(0) = right_dir;
    frame.row(1) = up_dir;
    frame.row(2) = fwd_dir;

    Matrix3d rot;
    rot << cos(angle), -sin(angle), 0,
           sin(angle),  cos(angle), 0,
           0,       0,              1;

    MatrixXd ret_frame = rot * frame;

    const RowVector3d tv1 = TV.row(TT(m_constrainable_tets_idx[idx], 0));
    const RowVector3d tv2 = TV.row(TT(m_constrainable_tets_idx[idx], 1));
    const RowVector3d tv3 = TV.row(TT(m_constrainable_tets_idx[idx], 2));
    const RowVector3d tv4 = TV.row(TT(m_constrainable_tets_idx[idx], 3));

    RowVectorXd ret_tet_ctr = (tv1 + tv2 + tv3 + tv4) / 4.0;

    return std::make_pair(ret_frame, ret_tet_ctr);
  }

  int num_constraints() const {
    assert(m_bone_constraints_idx.size() == m_bone_constraints_pos.size());
    assert(m_orientation_constraints_idx.size() == m_orientation_constraints_pos.size());
    return m_bone_constraints_idx.size() + m_orientation_constraints_idx.size();
  }

  int num_orientation_constraints() const {
    assert(m_orientation_constraints_idx.size() == m_orientation_constraints_pos.size());
    return m_orientation_constraints_idx.size();
  }

  int num_bone_constraints() const {
    assert(m_bone_constraints_idx.size() == m_bone_constraints_pos.size());
    return m_bone_constraints_idx.size();
  }

  int num_constrainable_tets() const {
    return m_constrainable_tets_idx.size();
  }

  void clear_orientation_constraints() {
    m_orientation_constraints_idx.clear();
    m_orientation_constraints_pos.clear();
    m_tet_constraints.clear();
  }

  double update_bone_constraints(const Eigen::MatrixXd& TV,
                                 const Eigen::MatrixXi& TT,
                                 const Eigen::VectorXd& isovals,
                                 const std::array<int, 2>& endpoints,
                                 int num_verts) {
    using namespace std;
    using namespace Eigen;

    MatrixXd LV;
    MatrixXi LF;

    unordered_set<int> vmap;
    vmap.max_load_factor(0.5);
    vmap.reserve(num_verts);

    RowVector3d last_ctr = TV.row(endpoints[0]);
    m_bone_constraints_idx.push_back(endpoints[0]);
    m_bone_constraints_pos.push_back(RowVector3d(0, 0, 0));

    double dist = 0.0;
    for(int i = 1; i < num_verts; i++) {
      const double isovalue = i * (1.0/num_verts);

      igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);

      if (LV.rows() == 0) {
        cout << "Empty level set" << endl;
        continue;
      }

      RowVector3d ctr = LV.colwise().sum() / LV.rows();
      dist += (ctr - last_ctr).norm();
      last_ctr = ctr;

      const int tet = containing_tet(TV, TT, ctr);
      if (tet < 0) {
        cout << "Vertex not in tet" << endl;
        continue;
      }

      Matrix3d v;
      for (int k = 0; k < 3; k++) { v.row(k) = TV.row(TT(tet, k)); }
      int nv = TT(tet, nearest_vertex(v, ctr));

      if (vmap.find(nv) == vmap.end()) {
        vmap.insert(nv);
        m_bone_constraints_idx.push_back(nv);
        m_bone_constraints_pos.push_back(RowVector3d(0, 0, dist));
        m_constrainable_tets_idx.push_back(tet);
        m_level_set_distances.push_back(dist);
        m_level_set_isovalues.push_back(isovalue);
      }
    }

    dist += (TV.row(endpoints[1]) - last_ctr).norm();
    m_bone_constraints_idx.push_back(endpoints[1]);
    m_bone_constraints_pos.push_back(RowVector3d(0, 0, dist));
    return dist;
  }

  double update_orientation_constraint(const Eigen::MatrixXd& TV,
                                       const Eigen::MatrixXi& TT,
                                       const Eigen::VectorXd& isovals,
                                       int idx, double angle, bool flipped_x) {
    using namespace Eigen;

    const int tt1 = TT(m_constrainable_tets_idx[idx], 0);
    const int tt2 = TT(m_constrainable_tets_idx[idx], 1);
    const int tt3 = TT(m_constrainable_tets_idx[idx], 2);
    const int tt4 = TT(m_constrainable_tets_idx[idx], 3);
    const RowVector3d tv1 = TV.row(tt1);
    const RowVector3d tv2 = TV.row(tt2);
    const RowVector3d tv3 = TV.row(tt3);
    const RowVector3d tv4 = TV.row(tt4);

    int bc_idx = -1;
    auto it = m_tet_constraints.find(idx);
    if (it == m_tet_constraints.end()) {
      bc_idx = m_orientation_constraints_idx.size();
      m_tet_constraints[idx] = std::make_tuple(bc_idx, angle, flipped_x);

      m_orientation_constraints_idx.push_back(tt1);
      m_orientation_constraints_idx.push_back(tt2);
      m_orientation_constraints_idx.push_back(tt3);
      m_orientation_constraints_idx.push_back(tt4);

      m_orientation_constraints_pos.push_back(tv1);
      m_orientation_constraints_pos.push_back(tv2);
      m_orientation_constraints_pos.push_back(tv3);
      m_orientation_constraints_pos.push_back(tv4);
    } else {
      bc_idx = std::get<0>(it->second);
    }

    auto frame_ctr = frame_for_tet(TV, TT, isovals, idx, angle, flipped_x);
    const Matrix3d frame = frame_ctr.first;
    const RowVector3d tet_ctr = frame_ctr.second;
    const RowVector3d constrained_tet_ctr(0, 0, m_level_set_distances[idx]);

    m_orientation_constraints_pos[bc_idx+0] = (tv1 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    m_orientation_constraints_pos[bc_idx+1] = (tv2 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    m_orientation_constraints_pos[bc_idx+2] = (tv3 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    m_orientation_constraints_pos[bc_idx+3] = (tv4 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
  }

  void slim_constraints(Eigen::VectorXi& slim_b, Eigen::MatrixXd& slim_bc, bool only_ends_and_tets) {
    slim_b.resize(num_constraints());
    slim_bc.resize(num_constraints(), 3);
    int c_count = 0;
    if (!only_ends_and_tets) {
      for (int i = 0; i < m_bone_constraints_idx.size(); i++) {
        slim_b[c_count] = m_bone_constraints_idx[i];
        slim_bc.row(c_count) = m_bone_constraints_pos[i];
        c_count += 1;
      }
    } else {
      slim_b[c_count] = m_bone_constraints_idx[0];
      slim_bc.row(c_count) = m_bone_constraints_pos[0];
      c_count += 1;
      slim_b[c_count] = m_bone_constraints_idx.back();
      slim_bc.row(c_count) = m_bone_constraints_pos.back();
      c_count += 1;
    }
    for (int i = 0; i < m_orientation_constraints_idx.size(); i++) {
      slim_b[c_count] = m_orientation_constraints_idx[i];
      slim_bc.row(c_count) = m_orientation_constraints_pos[i];
      c_count += 1;
    }

    slim_b.conservativeResize(c_count);
    slim_bc.conservativeResize(c_count, 3);
  }
};


struct UIState {
  // Overlay draw toggles
  bool m_draw_isovalues = false;
  bool m_draw_surface = true;
  bool m_draw_tet_wireframe = false;
  bool m_draw_skeleton = false;
  bool m_draw_level_set = false;
  bool m_draw_constraints = false;
  bool m_draw_original_mesh = false;

  // Show different sub-menus or not
  bool m_show_diffusion_menu = false;
  bool m_show_straighteining_menu = false;

  // Selection Mode Parameters
  bool m_show_point_selection_mode = false; // True if we're in point selection mode
  int m_current_endpoint_idx = 0; // The index of the endpoint we're selecting. Will be 2 if we've made a selection
  std::array<int, 2> m_selected_end_coords{{-1, -1}}; // The indexes of selected endpoints

  // Straightening UI parameters
  int m_num_skel_verts = 100;
  float m_current_angle = 0; // used to control the up vector
  int m_current_level_set = 0; // The current selected level set
  bool m_only_ends_and_tets = false; // Only constrain the endpoints and selected tets
  bool m_flip_x = false;

  // Appearance parameters when we draw overlay points
  float m_vfield_scale = 150.0f;
  float m_overlay_linewidth = 1.5;
  float m_overlay_point_size = 7.0;

  void begin_endpoint_selection() {
    m_show_point_selection_mode = true;

    m_current_endpoint_idx = 0;

    m_show_diffusion_menu = false;
    m_show_straighteining_menu = false;

    m_draw_surface = true;
    m_draw_tet_wireframe = false;
    m_draw_skeleton = false;
    m_draw_isovalues = false;
    m_draw_level_set = false;
    m_draw_constraints = false;
    m_draw_original_mesh = false;
  }

  void end_endpoint_selection(std::array<int, 2>& endpoints) {
    m_show_diffusion_menu = true;
    m_show_straighteining_menu = false;
    m_selected_end_coords = endpoints;
    m_current_endpoint_idx = 2;
  }

  void after_compute_diffusion() {
    m_draw_surface = false;
    m_draw_skeleton = true;
    m_draw_tet_wireframe = true;
    m_draw_isovalues = true;
    m_draw_level_set = true;
    m_draw_original_mesh = false;

    m_show_straighteining_menu = true;
  }
};


class FishPreprocessingMenu :
    public igl::opengl::glfw::imgui::ImGuiMenu {

  std::string m_current_model_filename;

  // Original mesh data. This will not change after we load the mesh in the constructor
  Eigen::MatrixXd TV;
  Eigen::MatrixXi TF;
  Eigen::MatrixXi TT;
  Eigen::MatrixXd TEV1, TEV2;
  Eigen::VectorXd isovals;
  Eigen::MatrixXd m_isovalColors;

  // UI variables set by imgui
  UIState m_ui_state;
  std::vector<UIState> m_ui_state_stack;

  // Double buffered draw state
  std::array<DrawState, 2> m_ds;
  ConstraintState m_constraints;

  // SLIM thread state
  int m_current_buf = 0;
  bool m_slim_running = false;
  std::thread m_slim_thread;
  std::mutex m_constraints_lock;
  std::mutex m_double_buf_lock;

  // Control flow flags
  std::atomic<bool> m_draw_state_changed;  // Set this when we need to redraw
  std::atomic<bool> m_constraints_changed; // Set this when we add or remove constraints

  // Drawing parameters
  Viewer& m_viewer;
  int m_main_mesh_id;
  int m_overlay_mesh_id;
  int m_background_mesh_id;

  void select_overlay_mesh() {
    m_viewer.selected_data_index = m_overlay_mesh_id;
  }

  void select_main_mesh() {
    m_viewer.selected_data_index = m_main_mesh_id;
  }

  void select_background_mesh() {
    m_viewer.selected_data_index = m_background_mesh_id;
  }

  void update_current_level_set() {
    m_double_buf_lock.lock();
    DrawState& ds = m_ds[m_current_buf];
    auto it = m_constraints.m_tet_constraints.find(m_ui_state.m_current_level_set);
    if (it != m_constraints.m_tet_constraints.end()) {
      m_ui_state.m_current_angle = std::get<1>(it->second);
      m_ui_state.m_flip_x = std::get<2>(it->second);
    }
    ds.update_isovalue(isovals, m_constraints.m_level_set_isovalues[m_ui_state.m_current_level_set]);
    m_double_buf_lock.unlock();
    m_draw_state_changed = true;
  }

  void slim_thread() {
    using namespace std;
    using namespace Eigen;

    igl::SLIMData sData;

    cout << "SLIM Thread: Starting SLIM background thread.." << endl;
    while (m_slim_running) {

      m_constraints_lock.lock();
      if (m_constraints_changed) {
        cout << "SLIM Thread: Reinitializing SLIM with " << m_constraints.num_constraints() << " constraints." << endl;

        VectorXi slim_b;
        MatrixXd slim_bc;
        m_constraints.slim_constraints(slim_b, slim_bc, m_ui_state.m_only_ends_and_tets);

        const double soft_const_p = 1e5;
        const MatrixXd TV_0 = m_ds[m_current_buf].m_TV;
        sData.exp_factor = 5.0;
        slim_precompute(TV, TT, TV_0, sData, igl::SLIMData::EXP_CONFORMAL,
                        slim_b, slim_bc, soft_const_p);
        cout << "SLIM Thread: Done reinitializing SLIM" << endl;
        m_constraints_changed = false;
      }
      m_constraints_lock.unlock();

      if (m_constraints.num_constraints() == 0) {
        this_thread::yield();
        continue;
      }

      igl::slim_solve(sData, 1);

      int buffer = (m_current_buf + 1) % 2;
      DrawState& ds = m_ds[buffer];
      ds.update_tet_mesh(sData.V_o, TT);
      ds.update_isovalue(isovals, m_constraints.m_level_set_isovalues[m_ui_state.m_current_level_set]);
      ds.update_skeleton(isovals, m_ui_state.m_num_skel_verts);
      ds.update_constraint_points(sData.b, sData.bc);
      m_double_buf_lock.lock();
      m_current_buf = buffer;
      m_double_buf_lock.unlock();
      m_draw_state_changed = true;
    }
  }

public:
  FishPreprocessingMenu(const std::string& filename, Viewer& viewer) : m_viewer(viewer) {
    using namespace std;

    // Load the tet mesh
    m_current_model_filename = filename;
    load_yixin_tetmesh(filename, TV, TF, TT);
    tet_mesh_edges(TV, TT, TEV1, TEV2);

    cout << "Loaded " << filename << " with " << TV.rows() << " vertices, " <<
            TF.rows() << " boundary faces, and " << TT.rows() <<
            " tets." << endl;

    // Initialize the draw state
    m_current_buf = 0;
    m_ds[m_current_buf].update_tet_mesh(TV, TT);
    m_ds[0].m_TF = TF;
    m_ds[1].m_TF = TF;
    m_draw_state_changed = true;

    // Start the SLIM background thread
    m_slim_running = true;
    m_slim_thread = thread(&FishPreprocessingMenu::slim_thread, this);

    // Create mesh layers in viewer
    m_main_mesh_id = m_viewer.selected_data_index;
    m_viewer.append_mesh();
    m_overlay_mesh_id = m_viewer.selected_data_index;
    m_viewer.append_mesh();
    m_background_mesh_id = m_viewer.selected_data_index;
    select_main_mesh();
    viewer.core.align_camera_center(TV, TF);

    // Make sure we draw the first frame
    m_draw_state_changed = true;

    viewer.plugins.push_back(this);
  }

  ~FishPreprocessingMenu() {
    m_slim_running = false;
    m_slim_thread.join();
  }

  virtual bool pre_draw() override {
    using namespace Eigen;
    using namespace std;

    if (m_draw_state_changed) {
      m_draw_state_changed = false;

      m_double_buf_lock.lock();
      Auto(m_double_buf_lock.unlock());

      DrawState& ds = m_ds[m_current_buf];

      // Draw the background
      select_background_mesh();
      m_viewer.data().clear();
      m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(100000, 0, 0), ColorRGB::RED);
      m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(0, 100000, 0), ColorRGB::GREEN);
      m_viewer.data().add_edges(RowVector3d(0, 0, 0), RowVector3d(0, 0, 100000), ColorRGB::BLUE);

      // Clear the main mesh
      select_main_mesh();
      m_viewer.data().clear();
      m_viewer.data().point_size = 5.0;
      m_viewer.data().set_mesh(ds.m_TV, ds.m_TF);
      m_viewer.data().show_lines = false;
      m_viewer.data().show_faces = false;

      // Clear the overlay mesh
      select_overlay_mesh();
      m_viewer.data().clear();
      m_viewer.data().line_width = m_ui_state.m_overlay_linewidth;
      m_viewer.data().point_size = m_ui_state.m_overlay_point_size;


      // Draw the tet mesh surface
      select_main_mesh();
      if (m_ui_state.m_draw_surface) {
        m_viewer.data().show_lines = true;
        m_viewer.data().show_faces = true;
      }

      // Draw selected endpoints
      select_overlay_mesh();
      for (int i = 0; i < m_ui_state.m_current_endpoint_idx; i++) {
        const int vid = m_ui_state.m_selected_end_coords[i];
        m_viewer.data().add_points(ds.m_TV.row(vid), i == 0 ? ColorRGB::GREEN : ColorRGB::RED);
      }

      // Draw the tet mesh wireframe
      select_main_mesh();
      if (m_ui_state.m_draw_tet_wireframe) {
        m_viewer.data().add_edges(ds.m_TEV1, ds.m_TEV2, ColorRGB::DARK_GRAY);
      } else if (m_ui_state.m_draw_tet_wireframe && !m_ui_state.m_draw_isovalues) {
        m_viewer.data().add_points(ds.m_TV, ColorRGB::GRAY);
        m_viewer.data().add_edges(ds.m_TEV1, ds.m_TEV2, ColorRGB::DARK_GRAY);
      }

      // Draw the original mesh
      select_main_mesh();
      if (m_ui_state.m_draw_original_mesh) {
        m_viewer.data().add_edges(TEV1, TEV2, ColorRGB::DARK_GRAY);
        m_viewer.data().add_points(TV, ColorRGB::DARK_MAGENTA);
      }

      // Draw the isovalues
      select_overlay_mesh();
      if (m_ui_state.m_draw_isovalues) {
        m_viewer.data().add_points(ds.m_TV, m_isovalColors);
      }

      // Draw the SLIM constraints
      select_overlay_mesh();
      if (m_ui_state.m_draw_constraints) {
        m_viewer.data().add_points(ds.m_bTV, ColorRGB::LIGHT_GREEN);
        m_viewer.data().add_points(ds.m_bV, ColorRGB::STEEL_BLUE);
      }

      // Draw the level set
      select_overlay_mesh();
      if (m_ui_state.m_draw_level_set) {
        m_viewer.data().set_mesh(ds.m_isoV, ds.m_isoF);
        pair<Matrix3d, RowVector3d> frame_ctr = m_constraints.frame_for_tet(ds.m_TV, ds.m_TT, isovals, m_ui_state.m_current_level_set, m_ui_state.m_current_angle, m_ui_state.m_flip_x);
        Matrix3d frame = frame_ctr.first;
        RowVector3d ctr = frame_ctr.second;

        RowVector3d isoX = frame.row(0);
        RowVector3d isoY = frame.row(1);
        RowVector3d isoN = frame.row(2);
        m_viewer.data().add_edges(ctr, ctr+isoX*m_ui_state.m_vfield_scale, ColorRGB::CRIMSON);
        m_viewer.data().add_edges(ctr, ctr+isoY*m_ui_state.m_vfield_scale, ColorRGB::LIGHT_GREEN);
        m_viewer.data().add_edges(ctr, ctr+isoN*m_ui_state.m_vfield_scale, ColorRGB::STEEL_BLUE);
        m_viewer.data().add_points(ctr, ColorRGB::DARK_MAGENTA);
      }

      // Draw the skeleton
      select_overlay_mesh();
      if (m_ui_state.m_draw_skeleton) {
        MatrixXd v1(ds.m_joints.rows()-1, 3), v2(ds.m_joints.rows()-1, 3);

        for (int i = 0; i < ds.m_joints.rows()-1; i++) {
          v1.row(i) = ds.m_joints.row(i);//TV.row(skeletonV[i]);
          v2.row(i) = ds.m_joints.row(i+1);//TV.row(skeletonV[i+1]);
        }
        m_viewer.data().add_edges(v1, v2, ColorRGB::STEEL_BLUE);
        m_viewer.data().add_points(v1, ColorRGB::CRIMSON);
        m_viewer.data().add_points(v2.row(ds.m_joints.rows()-2), ColorRGB::CRIMSON);
      }
    }

    return ImGuiMenu::pre_draw();
  }

  virtual bool mouse_down(int button, int modifier) override {
    using namespace std;

    if (m_ui_state.m_show_point_selection_mode) {
      int fid;            // ID of the clicked face
      Eigen::Vector3f bc; // Barycentric coords of the click point on the face
      double x = m_viewer.current_mouse_x;
      double y = m_viewer.core.viewport(3) - m_viewer.current_mouse_y;
      if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y),
                                  m_viewer.core.view * m_viewer.core.model,
                                  m_viewer.core.proj, m_viewer.core.viewport,
                                  TV, TF, fid, bc)) {
        if (!m_ui_state.m_show_point_selection_mode) {
          return false;
        }
        assert(m_current_endpoint_idx <= 1);

        int max;
        bc.maxCoeff(&max);
        int vid = TF(fid, max);
        m_ui_state.m_selected_end_coords[m_ui_state.m_current_endpoint_idx] = vid;

        m_ui_state.m_current_endpoint_idx += 1;
        if (m_ui_state.m_current_endpoint_idx == 2) {
          UIState restore_ui_state = m_ui_state_stack.back();
          m_ui_state_stack.pop_back();
          restore_ui_state.end_endpoint_selection(m_ui_state.m_selected_end_coords);
          m_ui_state = restore_ui_state;
        }
        m_draw_state_changed = true;
      }
    }

    return ImGuiMenu::mouse_down(button, modifier);
  }

  virtual bool key_up(int key, int modifier) override {

    if (m_ui_state.m_show_straighteining_menu) {
      const double one_deg = M_PI / 180.0;
      int step = 1;
      if (modifier & GLFW_MOD_ALT) {
        step = 10;
      }
      switch(key) {
      case GLFW_KEY_RIGHT:
        m_ui_state.m_current_level_set = std::min(m_ui_state.m_current_level_set+step, m_constraints.num_constrainable_tets()-1);
        update_current_level_set();
        break;
      case GLFW_KEY_LEFT:
        m_ui_state.m_current_level_set = std::max(m_ui_state.m_current_level_set-step, 0);
        update_current_level_set();
        break;
      case GLFW_KEY_UP:
        m_ui_state.m_current_angle = std::min(m_ui_state.m_current_angle+one_deg*step, 2*M_PI);
        break;
      case GLFW_KEY_DOWN:
        m_ui_state.m_current_angle = std::max(m_ui_state.m_current_angle-one_deg*step, -2*M_PI);
        break;
      }
    }
    return ImGuiMenu::key_pressed(key, modifier);
  }

  virtual void draw_viewer_menu() override {
    using namespace std;

    string title_text = string("Current Model: ") + m_current_model_filename;
    ImGui::Text("%s", title_text.c_str());

    //
    // Interface for selecting endpoints
    //
    if (ImGui::CollapsingHeader("Endpoint Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
      string button_text("Select Endpoints");
      if (m_ui_state.m_show_point_selection_mode) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        if (m_ui_state.m_current_endpoint_idx == 0) {
          button_text = string("Select Front");
        } else if (m_ui_state.m_current_endpoint_idx == 1) {
          button_text = string("Select Back");
        } else {
          assert(false);
        }
      }

      if (ImGui::Button(button_text.c_str(), ImVec2(-1,0))) {
        m_ui_state_stack.push_back(m_ui_state);
        m_ui_state.begin_endpoint_selection();
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        m_draw_state_changed = true;
      }

      if (m_ui_state.m_show_point_selection_mode) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();

        if (ImGui::Button("Cancel", ImVec2(-1,0))) {
          m_ui_state.m_show_point_selection_mode = false;
          m_ui_state = m_ui_state_stack.back();
          m_ui_state_stack.pop_back();
        }
      }
    }

    //
    // Interface for computing diffusion distance and quantizing the level sets
    //
    if (m_ui_state.m_show_diffusion_menu) {
      if (ImGui::CollapsingHeader("Diffusion Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("Number of Joints", &m_ui_state.m_num_skel_verts, 1.0f, 5, 100);

        if (ImGui::Button("Compute Diffusion Distances", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            diffusion_distances(TV, TT, m_ui_state.m_selected_end_coords, isovals);
            isoval_colors(isovals, m_isovalColors);
            cout << "Done!" << endl;
          }

          m_double_buf_lock.lock();
          DrawState& ds = m_ds[m_current_buf];
          ds.update_skeleton(isovals, m_ui_state.m_num_skel_verts);
          m_double_buf_lock.unlock();

          // Add constraints at each of the skeleton joints
          m_constraints_lock.lock();
          double dist = m_constraints.update_bone_constraints(TV, TT, isovals, m_ui_state.m_selected_end_coords, m_ui_state.m_num_skel_verts);
          m_constraints_changed = true;
          m_constraints_lock.unlock();

          cout << "Skeleton is " << dist << " units long." << endl;

          // Show the diffusion menu and draw the isovalues
          m_ui_state.after_compute_diffusion();

          m_draw_state_changed = true;
        }
      }
    }

    //
    // Skeleton Straightening Options
    //
    if (m_ui_state.m_show_straighteining_menu) {
      if (ImGui::CollapsingHeader("Straightening Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderAngle("Up Angle", &m_ui_state.m_current_angle, -360.0f, 360.0f)) {
          m_draw_state_changed = true;
        }

        if (ImGui::DragInt("Level Set", &m_ui_state.m_current_level_set, 1.0f, 0, m_constraints.num_constrainable_tets())) {
          update_current_level_set();
          m_draw_state_changed = true;
        }

        if (ImGui::Button("Set Orientation Constraint", ImVec2(-1, 0))) {
          m_double_buf_lock.lock();
          m_constraints_lock.lock();

          DrawState& ds = m_ds[m_current_buf];
          m_constraints.update_orientation_constraint(ds.m_TV, ds.m_TT, isovals, m_ui_state.m_current_level_set, m_ui_state.m_current_angle, m_ui_state.m_flip_x);
          m_constraints_changed = true;

          m_constraints_lock.unlock();
          m_double_buf_lock.unlock();

          m_draw_state_changed = true;
        }

        if (ImGui::Checkbox("Flip X", &m_ui_state.m_flip_x)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Only Tets and Ends", &m_ui_state.m_only_ends_and_tets)) {
          m_constraints_lock.lock();
          m_constraints_changed = true;
          m_constraints_lock.unlock();
          m_draw_state_changed = true;
        }
        if (ImGui::Button("Clear Orientation Constraints")) {
          m_constraints_lock.lock();
          m_constraints.clear_orientation_constraints();
          m_constraints_lock.unlock();
          m_draw_state_changed = true;
        }
      }
    }

    //
    // Drawing options interface
    //
    if (ImGui::CollapsingHeader("Drawing Options", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Line Width", &m_ui_state.m_overlay_linewidth, 0.1f, 1.0f, 10.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::DragFloat("Vector Scale", &m_ui_state.m_vfield_scale, 1.0f, 10.0f, 1000.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::DragFloat("Overaly Point Size", &m_ui_state.m_overlay_point_size, 1.0f, 1.0f, 10.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Mesh Surface", &m_ui_state.m_draw_surface)) {
        m_draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Tetrahedral Wireframe", &m_ui_state.m_draw_tet_wireframe)) {
        m_draw_state_changed = true;
      }

      if (m_ui_state.m_show_straighteining_menu) {
        if (ImGui::Checkbox("Draw Isovalues", &m_ui_state.m_draw_isovalues)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Level Set", &m_ui_state.m_draw_level_set)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Constraints", &m_ui_state.m_draw_constraints)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Skeleton", &m_ui_state.m_draw_skeleton)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Original Mesh", &m_ui_state.m_draw_original_mesh)) {
          m_draw_state_changed = true;
        }
      }
    }
  }

};

int main(int argc, char *argv[]) {
  Viewer viewer;
//  FishPreprocessingMenu menu("./data/Sternopygus_pejeraton-small.dat.out_.msh", viewer);
//  FishPreprocessingMenu menu("/home/francis/Sternopygus_arenatus-small.dat.out_.msh", viewer);
//  FishPreprocessingMenu menu("./data/p-tapinosoma.dat.out_.msh", viewer);
  if (argc == 1) {
    FishPreprocessingMenu menu("./data/small_fish/Sternopygus_arenatus-small.dat.out_.msh", viewer);
    viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
    return viewer.launch();
  } else {
    FishPreprocessingMenu menu(argv[1], viewer);
    viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
    return viewer.launch();
  }
}
