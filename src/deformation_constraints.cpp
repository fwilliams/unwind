#include "deformation_constraints.h"

#include <iostream>
#include <unordered_set>

#include <Eigen/Geometry>

#include "utils.h"

bool DeformationConstraints::validate_endpoint_pairs(const std::vector<std::array<int, 2>>& endpoints, const Eigen::VectorXi& components) {
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

std::pair<Eigen::Matrix3d, Eigen::RowVector3d> DeformationConstraints::frame_for_tet(
    const Eigen::MatrixXd& TV,
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


int DeformationConstraints::num_constraints() const {
  assert(m_bone_constraints_idx.size() == m_bone_constraints_pos.size());
  assert(m_orientation_constraints_idx.size() == m_orientation_constraints_pos.size());
  return m_bone_constraints_idx.size() + m_orientation_constraints_idx.size();
}


int DeformationConstraints::num_orientation_constraints() const {
  assert(m_orientation_constraints_idx.size() == m_orientation_constraints_pos.size());
  return m_orientation_constraints_idx.size();
}


int DeformationConstraints::num_bone_constraints() const {
  assert(m_bone_constraints_idx.size() == m_bone_constraints_pos.size());
  return m_bone_constraints_idx.size();
}


int DeformationConstraints:: num_constrainable_tets() const {
  return m_constrainable_tets_idx.size();
}


void DeformationConstraints::clear_orientation_constraints() {
  m_orientation_constraints_idx.clear();
  m_orientation_constraints_pos.clear();
  m_tet_constraints.clear();
}


double DeformationConstraints::one_pair_bone_constraints(
    const Eigen::MatrixXd& TV,
    const Eigen::MatrixXi& TT,
    const Eigen::VectorXd& isovals,
    const std::array<int, 2>& endpoints,
    int num_verts,
    const Eigen::RowVector3d& start_constraint,
    double isovalue_incr) {
  using namespace std;
  using namespace Eigen;

  MatrixXd LV;
  MatrixXi LF;

  unordered_set<int> vmap;
  vmap.max_load_factor(0.5);
  vmap.reserve(num_verts);

  const int num_endpoint_pairs = endpoints.size();
  assert(num_endpoint_pairs > 0);

  RowVector3d last_ctr = TV.row(endpoints[0]);
  m_bone_constraints_idx.push_back(endpoints[0]);
  m_bone_constraints_pos.push_back(start_constraint);

  last_ctr = TV.row(endpoints[0]);
  double dist = 0.0;
  double isovalue = isovals[endpoints[0]];
  for(int i = 1; i < num_verts; i++) {
    isovalue += isovalue_incr;
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);

    if (LV.rows() == 0) {
      cerr << "WARNING: Empty level set" << endl;
      continue;
    }

    RowVector3d ctr = LV.colwise().sum() / LV.rows();
    dist += (ctr - last_ctr).norm();
    last_ctr = ctr;

    const int tet = containing_tet(TV, TT, ctr);
    if (tet < 0) {
      cerr << "WARNING: Vertex not in tet" << endl;
      continue;
    }

    Matrix3d v;
    for (int k = 0; k < 3; k++) { v.row(k) = TV.row(TT(tet, k)); }
    int nv = TT(tet, nearest_vertex(v, ctr));

    if (vmap.find(nv) == vmap.end()) {
      vmap.insert(nv);
      m_bone_constraints_idx.push_back(nv);
      m_bone_constraints_pos.push_back(start_constraint + RowVector3d(0, 0, dist));
      m_constrainable_tets_idx.push_back(tet);
      m_level_set_distances.push_back(dist);
      m_level_set_isovalues.push_back(isovalue);
    }
  }

  dist += (TV.row(endpoints[1]) - last_ctr).norm();
  m_bone_constraints_idx.push_back(endpoints[1]);
  m_bone_constraints_pos.push_back(start_constraint + RowVector3d(0, 0, dist));
  return dist;
}

double DeformationConstraints::update_bone_constraints(
    const Eigen::MatrixXd& TV,
    const Eigen::MatrixXi& TT,
    const Eigen::VectorXd& isovals,
    const Eigen::VectorXd& components,
    const std::vector<std::array<int, 2>>& endpoints,
    int num_verts) {
  using namespace std;
  using namespace Eigen;

  double dist = 0.0;
  const double isovalue_increment = 1.0 / (num_verts+1);
  const int num_endpoint_pairs = endpoints.size();
  const int num_verts_per_segment = int(ceil(double(num_verts) / num_endpoint_pairs));
  assert(num_endpoint_pairs != 0);

  for (int i = 0; i < num_endpoint_pairs; i++) {
    dist += one_pair_bone_constraints(TV, TT, isovals, endpoints[i],
                                      num_verts_per_segment,
                                      RowVector3d(0, 0, dist),
                                      isovalue_increment);
  }
  return dist;
}


void DeformationConstraints::update_orientation_constraint(
    const Eigen::MatrixXd& TV,
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


void DeformationConstraints::slim_constraints(
    Eigen::VectorXi& slim_b,
    Eigen::MatrixXd& slim_bc,
    bool only_ends_and_tets) {
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
