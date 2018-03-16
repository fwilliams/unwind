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

#include "yixin_loader.h"

typedef igl::opengl::glfw::Viewer Viewer;


// Calculate the endpoints of edges for the tetmesh. Used for drawing.
void tet_mesh_edges(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                    Eigen::MatrixXd& V1, Eigen::MatrixXd& V2) {
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
void compute_diffusion(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const std::array<int, 2>& endpoints, Eigen::VectorXd& isovals) {
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
bool fit_plane(const Eigen::MatrixXd& P, Eigen::RowVector3d& c, Eigen::Matrix3d& frame) {
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

Eigen::Matrix3d z_rotated_frame(const Eigen::Matrix3d& frame, double angle, bool flip_xy, bool flip_z) {
  Eigen::Matrix3d R;
  R << cos(angle), -sin(angle), 0,
       sin(angle),  cos(angle), 0,
       0,       0,              1;
  Eigen::MatrixXd ret = R * frame;
  if (flip_xy) {
    Eigen::RowVector3d r0 = ret.row(0);
    ret.row(0) = ret.row(1);
    ret.row(1) = r0;
  }
  if (flip_z) {
    ret.row(2) *= -1;
  }
  return ret;
}

// Sample the level sets to extract a skeleton
void extract_skeleton(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd& isovals, int num_verts, Eigen::MatrixXd& joints) {
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

// Generate constraints along the skeleton
double skeleton_constraints(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                          const Eigen::VectorXd& isovals, int num_verts,
                          const std::array<int, 2>& endpoints,
                          std::vector<int>& b,
                          std::vector<Eigen::RowVector3d>& bc,
                          std::vector<int>& tets,
                          std::vector<std::pair<double, double>>& level_sets) {
  using namespace std;
  using namespace Eigen;

  MatrixXd LV;
  MatrixXi LF;

  unordered_set<int> vmap;
  vmap.max_load_factor(0.5);
  vmap.reserve(num_verts);

  RowVector3d lastC = TV.row(endpoints[0]);
  b.push_back(endpoints[0]);
  bc.push_back(RowVector3d(0, 0, 0));
  double dist = 0.0;
  for(int i = 1; i < num_verts; i++) {
    const double isovalue = i * (1.0/num_verts);
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);

    if (LV.rows() == 0) {
      cout << "Empty level set" << endl;
      continue;
    }

    RowVector3d C = LV.colwise().sum() / LV.rows();
    dist += (C - lastC).norm();
    lastC = C;

    const int tet = containing_tet(TV, TT, C);
    if (tet < 0) {
      cout << "Vertex not in tet" << endl;
      continue;
    }

    Matrix3d v;
    for (int k = 0; k < 3; k++) { v.row(k) = TV.row(TT(tet, k)); }
    int nv = TT(tet, nearest_vertex(v, C));
    if (vmap.find(nv) == vmap.end()) {
      vmap.insert(nv);
      b.push_back(nv);
      bc.push_back(RowVector3d(0, 0, dist));
      tets.push_back(tet);
      level_sets.push_back(make_pair(isovalue, dist));
    }
  }
  dist += (TV.row(endpoints[1]) - lastC).norm();
  b.push_back(endpoints[1]);
  bc.push_back(RowVector3d(0, 0, dist));
  return dist;
}


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


struct FishConstraints {

  std::vector<int> bone_constraints_idx;
  std::vector<Eigen::RowVector3d> bone_constraints_pos;
  std::vector<int> orientation_constraints_idx;
  std::vector<Eigen::RowVector3d> orientation_constraints_pos;

  std::array<int, 2>& endpoints;

  std::vector<double> level_set_isovalues;
  std::vector<double> level_set_distances;

  std::vector<int> constrained_tets_idx;
  std::unordered_map<int, std::pair<int, double>> tet_constraints;

  std::pair<Eigen::Matrix3d, Eigen::RowVector3d> frame_for_tet(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd& isovals, int idx, double angle) {
    using namespace Eigen;
    Eigen::MatrixXd LV;
    Eigen::MatrixXi LF;

    RowVector3d fwd_dir;

    const int N_LOOKAHEAD = 4;
    if (idx < constrained_tets_idx.size() - N_LOOKAHEAD) {
      igl::marching_tets(TV, TT, isovals, level_set_isovalues[idx], LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(TV, TT, isovals, level_set_isovalues[idx+N_LOOKAHEAD], LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    } else {
      igl::marching_tets(TV, TT, isovals, level_set_isovalues[idx-N_LOOKAHEAD], LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(TV, TT, isovals, level_set_isovalues[idx], LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    }

    RowVector3d up_dir = RowVector3d(0, 1, 0);
    up_dir -= fwd_dir*(up_dir.dot(fwd_dir));
    up_dir = up_dir.normalized();
    RowVector3d right_dir = fwd_dir.cross(up_dir);

    Matrix3d frame;
    frame.row(0) = right_dir;
    frame.row(1) = up_dir;
    frame.row(2) = fwd_dir;

    Matrix3d rot;
    rot << cos(angle), -sin(angle), 0,
           sin(angle),  cos(angle), 0,
           0,       0,              1;

    MatrixXd ret_frame = rot * frame;

    const RowVector3d tv1 = TV.row(TT(constrained_tets_idx[idx], 0));
    const RowVector3d tv2 = TV.row(TT(constrained_tets_idx[idx], 1));
    const RowVector3d tv3 = TV.row(TT(constrained_tets_idx[idx], 2));
    const RowVector3d tv4 = TV.row(TT(constrained_tets_idx[idx], 3));

    RowVectorXd ret_tet_ctr = (tv1 + tv2 + tv3 + tv4) / 4.0;

    return std::make_pair(ret_frame, ret_tet_ctr);
  }

  double update_bone_constraints(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd& isovals, int num_verts) {
    using namespace std;
    using namespace Eigen;

    MatrixXd LV;
    MatrixXi LF;

    unordered_set<int> vmap;
    vmap.max_load_factor(0.5);
    vmap.reserve(num_verts);

    RowVector3d last_ctr = TV.row(endpoints[0]);
    bone_constraints_idx.push_back(endpoints[0]);
    bone_constraints_pos.push_back(RowVector3d(0, 0, 0));

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
        bone_constraints_idx.push_back(nv);
        bone_constraints_pos.push_back(RowVector3d(0, 0, dist));
        constrained_tets_idx.push_back(tet);
        level_set_distances.push_back(dist);
        level_set_isovalues.push_back(isovalue);
      }
    }

    dist += (TV.row(endpoints[1]) - last_ctr).norm();
    bone_constraints_idx.push_back(endpoints[1]);
    bone_constraints_pos.push_back(RowVector3d(0, 0, dist));
    return dist;
  }

  double update_orientation_constraint(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd& isovals, int idx, double angle) {
    using namespace Eigen;

    const int tt1 = TT(constrained_tets_idx[idx], 0);
    const int tt2 = TT(constrained_tets_idx[idx], 1);
    const int tt3 = TT(constrained_tets_idx[idx], 2);
    const int tt4 = TT(constrained_tets_idx[idx], 3);
    const RowVector3d tv1 = TV.row(tt1);
    const RowVector3d tv2 = TV.row(tt2);
    const RowVector3d tv3 = TV.row(tt3);
    const RowVector3d tv4 = TV.row(tt4);

    int bc_idx = -1;
    auto it = tet_constraints.find(idx);
    if (it == tet_constraints.end()) {
      bc_idx = orientation_constraints_idx.size();
      tet_constraints[idx] = std::make_pair(bc_idx, angle);

      orientation_constraints_idx.push_back(tt1);
      orientation_constraints_idx.push_back(tt2);
      orientation_constraints_idx.push_back(tt3);
      orientation_constraints_idx.push_back(tt4);

      orientation_constraints_pos.push_back(tv1);
      orientation_constraints_pos.push_back(tv2);
      orientation_constraints_pos.push_back(tv3);
      orientation_constraints_pos.push_back(tv4);
    } else {
      bc_idx = it->second.first;
    }

    auto frame_ctr = frame_for_tet(TV, TT, isovals, idx, angle);
    const Matrix3d frame = frame_ctr.first;
    const RowVector3d tet_ctr = frame_ctr.second;
    const RowVector3d constrained_tet_ctr(0, 0, level_set_distances[idx]);

    orientation_constraints_pos[bc_idx+0] = (tv1 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    orientation_constraints_pos[bc_idx+1] = (tv2 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    orientation_constraints_pos[bc_idx+2] = (tv3 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
    orientation_constraints_pos[bc_idx+3] = (tv4 - tet_ctr) * frame.transpose() + constrained_tet_ctr;
  }
};


class FishPreprocessingMenu :
    public igl::opengl::glfw::imgui::ImGuiMenu {

  std::string m_current_model_filename;

  // Endpoint selection variables
  bool m_selecting_points = false;
  int m_current_endpoint_idx = 0;
  std::array<int, 2> m_selected_end_coords{{-1, -1}};
  std::array<int, 2> m_tmp_selected_end_coords{{-1, -1}};

  // Skeleton extraction variables
  int m_num_skel_verts = 100;

  // Level set variables
  float m_isovalue = 0.0;

  // Incremental straightening variables
  float m_current_angle = 0; // used to control the up vector
  int m_current_level_set = 0; // The current selected level set

  // Drawing parameters
  float m_vfield_scale = 150.0f;
  float m_overlay_linewidth = 1.5;
  float m_overlay_point_size = 7.0;
  Eigen::MatrixXd m_isovalColors;


  // Will be set to true if we need to redraw
  std::atomic<bool> m_draw_state_changed;

  // Draw State Variables. TODO: Refactor these into a struct
  bool m_draw_isovalues = false;
  bool m_draw_surface = true;
  bool m_draw_tet_wireframe = false;
  bool m_draw_skeleton = false;
  bool m_draw_level_set = false;
  bool m_draw_constraints = false;
  bool m_draw_original_mesh = false;

  // Used so we can restore draw state
  bool m_old_draw_isovalues = false;
  bool m_old_draw_surface = true;
  bool m_old_draw_tet_wireframe = false;
  bool m_old_draw_skeleton = false;
  bool m_old_draw_level_set = false;
  bool m_old_draw_constraints = false;
  bool m_old_draw_original_mesh = false;

  void push_draw_state() {
    m_old_draw_isovalues = m_draw_isovalues;
    m_old_draw_skeleton = m_draw_skeleton;
    m_old_draw_surface = m_draw_surface;
    m_old_draw_tet_wireframe = m_draw_tet_wireframe;
    m_old_draw_level_set = m_draw_level_set;
    m_old_draw_constraints = m_draw_constraints;
    m_old_draw_original_mesh = m_draw_original_mesh;
  }

  void pop_draw_state() {
    m_draw_isovalues = m_old_draw_isovalues;
    m_draw_skeleton = m_old_draw_skeleton;
    m_draw_surface = m_old_draw_surface;
    m_draw_tet_wireframe = m_old_draw_tet_wireframe;
    m_draw_level_set = m_old_draw_level_set;
    m_draw_constraints = m_old_draw_constraints;
    m_draw_original_mesh = m_old_draw_original_mesh;
  }

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

  void draw(const DrawState& ds) {
    using namespace Eigen;
    using namespace std;

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
    m_viewer.data().line_width = m_overlay_linewidth;
    m_viewer.data().point_size = m_overlay_point_size;


    // Draw the tet mesh surface
    select_main_mesh();
    if (m_draw_surface) {
      m_viewer.data().show_lines = true;
      m_viewer.data().show_faces = true;
    }

    // Draw selected endpoints
    select_overlay_mesh();
    if (m_selecting_points) {
      for (int i = 0; i < m_current_endpoint_idx; i++) {
        const int vid = m_tmp_selected_end_coords[i];
        m_viewer.data().add_points(ds.m_TV.row(vid), ColorRGB::GREEN);
      }
    } else if (m_selected_end_coords[0] != -1 &&
               m_selected_end_coords[1] != -1) {
      const int vid1 = m_selected_end_coords[0];
      const int vid2 = m_selected_end_coords[1];
      m_viewer.data().add_points(ds.m_TV.row(vid1), ColorRGB::GREEN);
      m_viewer.data().add_points(ds.m_TV.row(vid2), ColorRGB::RED);
    }

    // Draw the tet mesh wireframe
    select_main_mesh();
    if (m_draw_tet_wireframe) {
      m_viewer.data().add_edges(ds.m_TEV1, ds.m_TEV2, ColorRGB::DARK_GRAY);
    } else if (m_draw_tet_wireframe && !m_draw_isovalues) {
      m_viewer.data().add_points(ds.m_TV, ColorRGB::GRAY);
      m_viewer.data().add_edges(ds.m_TEV1, ds.m_TEV2, ColorRGB::DARK_GRAY);
    }

    // Draw the original mesh
    select_main_mesh();
    if (m_draw_original_mesh) {
      m_viewer.data().add_edges(TEV1, TEV2, ColorRGB::DARK_GRAY);
      m_viewer.data().add_points(TV, ColorRGB::DARK_MAGENTA);
    }

    // Draw the isovalues
    select_overlay_mesh();
    if (m_draw_isovalues) {
      m_viewer.data().add_points(ds.m_TV, m_isovalColors);
    }

    // Draw the SLIM constraints
    select_overlay_mesh();
    if (m_draw_constraints) {
      m_viewer.data().add_points(ds.m_bTV, ColorRGB::LIGHT_GREEN);
      m_viewer.data().add_points(ds.m_bV, ColorRGB::STEEL_BLUE);
    }

    // Draw the level set
    select_overlay_mesh();
    if (m_draw_level_set) {
      m_viewer.data().set_mesh(ds.m_isoV, ds.m_isoF);
      Matrix3d rFrame;
      RowVector3d ctr;
      compute_frame(rFrame, ctr);
      RowVector3d isoX = rFrame.row(0);
      RowVector3d isoY = rFrame.row(1);
      RowVector3d isoN = rFrame.row(2);
      m_viewer.data().add_edges(ctr, ctr+isoX*m_vfield_scale, ColorRGB::CRIMSON);
      m_viewer.data().add_edges(ctr, ctr+isoY*m_vfield_scale, ColorRGB::LIGHT_GREEN);
      m_viewer.data().add_edges(ctr, ctr+isoN*m_vfield_scale, ColorRGB::STEEL_BLUE);
      m_viewer.data().add_points(ctr, ColorRGB::DARK_MAGENTA);

      // If the level set is non-empty, draw it in the origin frame so we can see the orientation
//      if (ds.m_isoTV.rows() > 0) {
//        MatrixXd lv1, lv2;
//        tet_mesh_edges(ds.m_isoTV, ds.m_isoTT, lv1, lv2);
//        m_viewer.data().add_edges(lv1, lv2, ColorRGB::STEEL_BLUE);
//        m_viewer.data().add_points(ds.m_isoTV, ColorRGB::CRIMSON);

//        MatrixXd risoTV = (ds.m_isoTV.rowwise() - ds.m_isoC) * rFrame.transpose();
//        MatrixXd risolv1 = (lv1 - ds.m_isoC) * rFrame.transpose();
//        MatrixXd risolv2 = (lv2 - ds.m_isoC) * rFrame.transpose();
//        m_viewer.data().add_edges(risolv1, risolv2, ColorRGB::STEEL_BLUE);
//        m_viewer.data().add_points(risoTV, ColorRGB::CRIMSON);
//      }
    }

    // Draw the skeleton
    select_overlay_mesh();
    if (m_draw_skeleton) {
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

  void update_orientation_constraint() {
    using namespace std;
    using namespace Eigen;

    DrawState& ds = m_ds[m_current_buf];

    const int tt1 = ds.m_TT(m_constraint_tets_idx[m_current_level_set], 0);
    const int tt2 = ds.m_TT(m_constraint_tets_idx[m_current_level_set], 1);
    const int tt3 = ds.m_TT(m_constraint_tets_idx[m_current_level_set], 2);
    const int tt4 = ds.m_TT(m_constraint_tets_idx[m_current_level_set], 3);
    const RowVector3d tv1 = ds.m_TV.row(tt1);
    const RowVector3d tv2 = ds.m_TV.row(tt2);
    const RowVector3d tv3 = ds.m_TV.row(tt3);
    const RowVector3d tv4 = ds.m_TV.row(tt4);

    int bc_idx = -1;
    auto it = m_tet_constraints.find(m_current_level_set);
    if (it == m_tet_constraints.end()) {
      bc_idx = m_b.size();
      m_tet_constraints[m_current_level_set] = make_pair(bc_idx, m_current_angle);

      m_b.push_back(tt1);
      m_b.push_back(tt2);
      m_b.push_back(tt3);
      m_b.push_back(tt4);

      m_bc.push_back(tv1);
      m_bc.push_back(tv2);
      m_bc.push_back(tv3);
      m_bc.push_back(tv4);
    } else {
      bc_idx = it->second.first;
    }

    Eigen::Matrix3d rframe;
    Eigen::RowVector3d tet_ctr;
    compute_frame(rframe, tet_ctr);

    RowVector3d constr_ctr(0, 0, m_level_sets[m_current_level_set].second);
    cout << rframe << endl << endl;
    m_bc[bc_idx+0] = (tv1 - tet_ctr) * rframe.transpose() + constr_ctr;
    m_bc[bc_idx+1] = (tv2 - tet_ctr) * rframe.transpose() + constr_ctr;
    m_bc[bc_idx+2] = (tv3 - tet_ctr) * rframe.transpose() + constr_ctr;
    m_bc[bc_idx+3] = (tv4 - tet_ctr) * rframe.transpose() + constr_ctr;

    m_constraints_changed = true;
  }

  void update_current_level_set() {
    m_double_buf_lock.lock();
    DrawState& ds = m_ds[m_current_buf];
    m_isovalue = m_level_sets[m_current_level_set].first;
    auto it = m_tet_constraints.find(m_current_level_set);
    if (it != m_tet_constraints.end()) {
      m_current_angle = it->second.second;
    }
    ds.update_isovalue(isovals, m_isovalue);
    m_double_buf_lock.unlock();
    m_draw_state_changed = true;
  }

  void compute_frame(Eigen::Matrix3d& rframe, Eigen::RowVector3d& tet_ctr) {
    using namespace Eigen;
    Eigen::MatrixXd LV;
    Eigen::MatrixXi LF;

    DrawState& ds = m_ds[m_current_buf];
    RowVector3d fwd_dir;
    if (m_current_level_set < m_constraint_tets_idx.size()-4) {
      igl::marching_tets(ds.m_TV, ds.m_TT, isovals, m_level_sets[m_current_level_set].first, LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(ds.m_TV, ds.m_TT, isovals, m_level_sets[m_current_level_set+4].first, LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    } else {
      igl::marching_tets(ds.m_TV, ds.m_TT, isovals, m_level_sets[m_current_level_set-4].first, LV, LF);
      RowVector3d c1 = LV.colwise().sum() / LV.rows();

      igl::marching_tets(ds.m_TV, ds.m_TT, isovals, m_level_sets[m_current_level_set].first, LV, LF);
      RowVector3d c2 = LV.colwise().sum() / LV.rows();

      fwd_dir = (c2 - c1).normalized();
    }

    RowVector3d up_dir = RowVector3d(0, 1, 0);
    up_dir -= fwd_dir*(up_dir.dot(fwd_dir));
    up_dir = up_dir.normalized();
    RowVector3d right_dir = fwd_dir.cross(up_dir);

    Matrix3d frame;
    frame.row(0) = right_dir;
    frame.row(1) = up_dir;
    frame.row(2) = fwd_dir;

    Matrix3d rot;
    rot << cos(m_current_angle), -sin(m_current_angle), 0,
           sin(m_current_angle),  cos(m_current_angle), 0,
           0,                     0,                    1;

    rframe = rot * frame;

    RowVector3d tv1 = ds.m_TV.row(ds.m_TT(m_constraint_tets_idx[m_current_level_set], 0));
    RowVector3d tv2 = ds.m_TV.row(ds.m_TT(m_constraint_tets_idx[m_current_level_set], 1));
    RowVector3d tv3 = ds.m_TV.row(ds.m_TT(m_constraint_tets_idx[m_current_level_set], 2));
    RowVector3d tv4 = ds.m_TV.row(ds.m_TT(m_constraint_tets_idx[m_current_level_set], 3));

    tet_ctr = (tv1 + tv2 + tv3 + tv4) / 4.0;
  }

  void slim_thread() {
    using namespace std;
    using namespace Eigen;

    igl::SLIMData sData;

    cout << "SLIM Thread: Starting SLIM background thread.." << endl;
    while (m_slim_running) {

      m_constraints_lock.lock();
      if (m_last_num_constraints != m_b.size() || m_constraints_changed) {
        cout << "SLIM Thread: Reinitializing SLIM with " << m_b.size() << " constraints." << endl;
        VectorXi slim_b(m_b.size());
        MatrixXd slim_bc(m_bc.size(), 3);
        if (m_only_ends_and_tets) {
          int ccount = 0;
          for (int i = m_num_skel_constraints; i < m_b.size(); i++) {
            slim_b[ccount] = m_b[i];
            slim_bc.row(ccount) = m_bc[i];
            ccount += 1;
          }
          slim_b[ccount] = m_b[0];
          slim_bc.row(ccount) = m_bc[0];
          ccount += 1;
          slim_b[ccount] = m_b[m_num_skel_constraints-1];
          slim_bc.row(ccount) = m_bc[m_num_skel_constraints-1];
          ccount += 1;
          slim_b.conservativeResize(ccount);
          slim_bc.conservativeResize(ccount, 3);
        } else {
          for (int i = 0; i < m_b.size(); i++) {
            slim_b[i] = m_b[i];
            slim_bc.row(i) = m_bc[i];
          }
        }
        m_last_num_constraints = m_b.size();
        double soft_const_p = 1e5;
        sData.exp_factor = 5.0;
        m_double_buf_lock.lock();
        MatrixXd TV_0 = m_ds[m_current_buf].m_TV;
        m_double_buf_lock.unlock();
        slim_precompute(TV, TT, TV_0, sData, igl::SLIMData::EXP_CONFORMAL,
                        slim_b, slim_bc, soft_const_p);
        cout << "SLIM Thread: Done reinitializing SLIM" << endl;
        m_constraints_changed = false;
      }
      m_constraints_lock.unlock();

      if (m_last_num_constraints == 0) {
        // this_thread::yield();
        continue;
      }

      igl::slim_solve(sData, 1);

      int buffer = (m_current_buf + 1) % 2;
      DrawState& ds = m_ds[buffer];
      ds.update_tet_mesh(sData.V_o, TT);
      ds.update_isovalue(isovals, m_isovalue);
      ds.update_skeleton(isovals, m_num_skel_verts);
      ds.update_constraint_points(sData.b, sData.bc);
      m_double_buf_lock.lock();
      m_current_buf = buffer;
      m_double_buf_lock.unlock();
      m_draw_state_changed = true;
    }
  }

public:
  Eigen::MatrixXd TV;
  Eigen::MatrixXi TF;
  Eigen::MatrixXi TT;
  Eigen::MatrixXd TEV1, TEV2;
  Eigen::VectorXd isovals;

  // Double buffered draw state
  std::array<DrawState, 2> m_ds;

  // SLIM thread state
  std::thread m_slim_thread;
  int m_current_buf = 0;
  bool m_slim_running = false;
  std::mutex m_constraints_lock;
  std::mutex m_double_buf_lock;
  std::size_t m_last_num_constraints = 0;


  // Constraint variables
  std::vector<int> m_b;
  std::vector<Eigen::RowVector3d> m_bc;
  std::vector<int> m_constraint_tets_idx;
  std::vector<std::pair<double, double>> m_level_sets;
  std::vector<double> m_level_set_dists;
  std::unordered_map<int, std::pair<int, double>> m_tet_constraints;
  int m_num_skel_constraints = 0;
  bool m_only_ends_and_tets = false;
  bool m_constraints_changed = false;
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
    m_last_num_constraints = 0;
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
    if (m_draw_state_changed) {

      m_double_buf_lock.lock();
      draw(m_ds[m_current_buf]);
      m_double_buf_lock.unlock();

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
        }
        m_draw_state_changed = true;
      }
    }

    return ImGuiMenu::mouse_down(button, modifier);
  }

  virtual bool key_down(int key, int modifier) override {
    int step = 1;
    if (modifier & GLFW_MOD_ALT) {
      step = 10;
    }
    switch(key) {
    case GLFW_KEY_RIGHT:
      m_current_level_set = std::min(m_current_level_set+step, m_num_skel_constraints-1);
      update_current_level_set();
      break;
    case GLFW_KEY_LEFT:
      m_current_level_set = std::max(m_current_level_set-step, 0);
      update_current_level_set();
      break;
    }
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
        m_draw_level_set = false;
        m_draw_constraints = false;
        m_draw_original_mesh = false;
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
      if (ImGui::CollapsingHeader("Diffusion Options",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("Number of Joints", &m_num_skel_verts, 1.0f, 5, 100);

        if (ImGui::Button("Compute Diffusion Distances", ImVec2(-1,0))) {
          if (isovals.rows() == 0) {
            cout << "Solving diffusion on tet mesh..." << endl;
            compute_diffusion(TV, TT, m_selected_end_coords, isovals);
            isoval_colors(isovals, m_isovalColors);
            cout << "Done!" << endl;
          }

          m_double_buf_lock.lock();
          DrawState& ds = m_ds[m_current_buf];
          ds.update_skeleton(isovals, m_num_skel_verts);
          m_double_buf_lock.unlock();

          m_constraints_lock.lock();
          if (m_b.size() == 0) {
            double dist = skeleton_constraints(TV, TT, isovals, m_num_skel_verts, m_selected_end_coords, m_b, m_bc, m_constraint_tets_idx, m_level_sets);
            m_num_skel_constraints = m_b.size();
            cout << "Skeleton is " << dist << " units long." << endl;
          }
          m_constraints_lock.unlock();
          m_draw_surface = false;
          m_draw_skeleton = true;
          m_draw_tet_wireframe = true;
          m_draw_isovalues = true;
          m_draw_state_changed = true;
          m_draw_level_set = true;
          m_draw_original_mesh = false;
        }
      }
    }

    //
    // Skeleton Straightening Options
    //
    if (isovals.rows() > 0) {
      if (ImGui::CollapsingHeader("Straightening Options",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderAngle("Up Angle", &m_current_angle, -360.0f, 360.0f)) {
          m_draw_state_changed = true;
        }

        if (ImGui::DragInt("Level Set", &m_current_level_set, 1.0f, 0, m_num_skel_constraints-2)) {
          update_current_level_set();
          m_draw_state_changed = true;
        }

        if (ImGui::Button("Set Orientation Constraint", ImVec2(-1, 0))) {
          m_double_buf_lock.lock();
          m_constraints_lock.lock();
          update_orientation_constraint();
          m_constraints_lock.unlock();
          m_double_buf_lock.unlock();
          m_draw_state_changed = true;
        }

        if(ImGui::Checkbox("Only Tets and Ends", &m_only_ends_and_tets)) {
          m_constraints_lock.lock();
          m_constraints_changed = true;
          m_constraints_lock.unlock();
          m_draw_state_changed = true;
        }

      }
    }

    //
    // Drawing options interface
    //
    if (ImGui::CollapsingHeader("Drawing Options",
                                ImGuiTreeNodeFlags_DefaultOpen)) {

      if (ImGui::DragFloat("Line Width", &m_overlay_linewidth, 0.1f, 1.0f, 10.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::DragFloat("Vector Scale", &m_vfield_scale, 1.0f, 10.0f, 1000.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::DragFloat("Overaly Point Size", &m_overlay_point_size, 1.0f, 1.0f, 10.0f)) {
        m_draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Mesh Surface", &m_draw_surface)) {
        m_draw_state_changed = true;
      }
      if (ImGui::Checkbox("Draw Tetrahedral Wireframe", &m_draw_tet_wireframe)) {
        m_draw_state_changed = true;
      }

      if (isovals.rows() > 0) {
        if (ImGui::Checkbox("Draw Isovalues", &m_draw_isovalues)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Level Set", &m_draw_level_set)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Constraints", &m_draw_constraints)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Skeleton", &m_draw_skeleton)) {
          m_draw_state_changed = true;
        }
        if (ImGui::Checkbox("Draw Original Mesh", &m_draw_original_mesh)) {
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
    FishPreprocessingMenu menu("./data/sar2_.msh", viewer);
    viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
    return viewer.launch();
  } else {
    FishPreprocessingMenu menu(argv[1], viewer);
    viewer.core.background_color = Eigen::RowVector4f(0.9, 0.9, 1.0, 1.0);
    return viewer.launch();
  }
}
