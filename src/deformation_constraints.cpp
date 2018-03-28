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
    const Eigen::MatrixXd& TV_fat,
    const Eigen::MatrixXi& TT_fat,
    int idx, double angle,
    bool flip_x,
    int n_lookahead) {

  using namespace Eigen;

  RowVector3d fwd_dir;

  const int N_LOOKAHEAD = n_lookahead;
  if (idx < m_constrainable_tets_idx.size() - N_LOOKAHEAD) {
    RowVector3d c1 = TV_fat.row(m_bone_constraints_idx[idx]);
    RowVector3d c2 = TV_fat.row(m_bone_constraints_idx[idx+N_LOOKAHEAD]);

    fwd_dir = (c2 - c1).normalized();
  } else {
    RowVector3d c1 = TV_fat.row(m_bone_constraints_idx[idx-N_LOOKAHEAD]);
    RowVector3d c2 = TV_fat.row(m_bone_constraints_idx[idx]);

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

//  const RowVector3d tv1 = TV_fat.row(TT_fat(m_constrainable_tets_idx[idx], 0));
//  const RowVector3d tv2 = TV_fat.row(TT_fat(m_constrainable_tets_idx[idx], 1));
//  const RowVector3d tv3 = TV_fat.row(TT_fat(m_constrainable_tets_idx[idx], 2));
//  const RowVector3d tv4 = TV_fat.row(TT_fat(m_constrainable_tets_idx[idx], 3));

//  RowVectorXd ret_tet_ctr = (tv1 + tv2 + tv3 + tv4) / 4.0;

  return std::make_pair(ret_frame, TV_fat.row(m_bone_constraints_idx[idx]));
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
    const Eigen::MatrixXd& TV_fat,
    const Eigen::MatrixXi& TT_fat,
    const Eigen::MatrixXd& TV_thin,
    const Eigen::MatrixXi& TT_thin,
    const Eigen::VectorXd& geodesic_distances,
    const std::array<int, 2>& endpoints,
    const Eigen::RowVector3d& straight_dir,
    const Eigen::RowVector3d& straight_origin,
    int num_verts) {
  using namespace std;
  using namespace Eigen;

  MatrixXd LV;
  MatrixXi LF;

  unordered_set<int> vmap;
  vmap.max_load_factor(0.5);
  vmap.reserve(num_verts);

  const int num_endpoint_pairs = endpoints.size();
  assert(num_endpoint_pairs > 0);

  int ctr_idx = nearest_vertex(TV_fat, TV_thin.row(endpoints[0]));
  RowVector3d last_ctr = TV_fat.row(ctr_idx);
  m_bone_constraints_idx.push_back(ctr_idx);
  m_bone_constraints_pos.push_back(straight_origin);
  int tet_idx = -1;
  for (int i = 0; i < TT_fat.rows(); i++) {
    if (TT_fat(i, 0) == ctr_idx || TT_fat(i, 1) == ctr_idx || TT_fat(i, 2) == ctr_idx || TT_fat(i, 3) == ctr_idx) {
      tet_idx = i;
      break;
    }
  }
  assert(tet_idx >= 0);
  m_constrainable_tets_idx.push_back(tet_idx);


  double dist = 0.0;
  double isovalue = geodesic_distances[endpoints[0]];
  const double isovalue_incr = (geodesic_distances[endpoints[1]] - geodesic_distances[endpoints[0]]) / num_verts;

  for(int i = 1; i < num_verts; i++) {
    isovalue += isovalue_incr;
    igl::marching_tets(TV_thin, TT_thin, geodesic_distances, isovalue, LV, LF);

    if (LV.rows() == 0) {
      cerr << "WARNING: Empty level set" << endl;
      continue;
    }

    RowVector3d ctr = LV.colwise().sum() / LV.rows();
    dist += (ctr - last_ctr).norm();
    last_ctr = ctr;

    const int tet = containing_tet(TV_fat, TT_fat, ctr);
    if (tet < 0) {
      cerr << "WARNING: Vertex not in tet" << endl;
      continue;
    }

    Eigen::Matrix<double, 4, 3> v;
    for (int k = 0; k < 4; k++) { v.row(k) = TV_fat.row(TT_fat(tet, k)); }
    int nv = TT_fat(tet, nearest_vertex(v, ctr));

    if (vmap.find(nv) == vmap.end()) {
      vmap.insert(nv);
      m_bone_constraints_idx.push_back(nv);
      m_bone_constraints_pos.push_back(straight_origin + dist*straight_dir);
      m_constrainable_tets_idx.push_back(tet);
    }
  }

  ctr_idx = nearest_vertex(TV_fat, TV_thin.row(endpoints[1]));
  dist += (TV_fat.row(ctr_idx) - last_ctr).norm();
  m_bone_constraints_idx.push_back(ctr_idx);
  m_bone_constraints_pos.push_back(straight_origin + dist*straight_dir);
  tet_idx = -1;
  for (int i = 0; i < TT_fat.rows(); i++) {
    if (TT_fat(i, 0) == ctr_idx || TT_fat(i, 1) == ctr_idx || TT_fat(i, 2) == ctr_idx || TT_fat(i, 3) == ctr_idx) {
      tet_idx = i;
      break;
    }
  }
  assert(tet_idx >= 0);
  m_constrainable_tets_idx.push_back(tet_idx);
  return dist;
}

double DeformationConstraints::update_bone_constraints(
    const Eigen::MatrixXd& TV_fat,
    const Eigen::MatrixXi& TT_fat,
    const Eigen::MatrixXd& TV_thin,
    const Eigen::MatrixXi& TT_thin,
    const Eigen::VectorXd& geodesic_distances,
    const Eigen::VectorXi& components,
    const std::vector<std::array<int, 2>>& endpoints,
    int num_verts) {
  using namespace std;
  using namespace Eigen;

  double dist = 0.0;
  const int num_endpoint_pairs = endpoints.size();
  const int num_verts_per_segment = int(ceil(double(num_verts) / num_endpoint_pairs));
  assert(num_endpoint_pairs != 0);

  vector<MatrixXi> TTcomp;
  split_mesh_components(TT_thin, components, TTcomp);
  for (int i = 0; i < num_endpoint_pairs; i++) {
    dist += one_pair_bone_constraints(TV_fat, TT_fat, TV_thin, TTcomp[i], geodesic_distances, endpoints[i],
                                      RowVector3d(0, 0, 1),
                                      RowVector3d(0, 0, dist),
                                      num_verts_per_segment);
    if (i != num_endpoint_pairs-1) {
      dist += (TV_thin.row(endpoints[i+1][0]) - TV_thin.row(endpoints[i][1])).norm();
    }
  }
  return dist;
}


void DeformationConstraints::update_orientation_constraint(
    const Eigen::MatrixXd& TV_fat,
    const Eigen::MatrixXi& TT_fat,
    int idx, double angle, bool flipped_x) {
  using namespace Eigen;
  using namespace std;

  const int tt1 = TT_fat(m_constrainable_tets_idx[idx], 0);
  const int tt2 = TT_fat(m_constrainable_tets_idx[idx], 1);
  const int tt3 = TT_fat(m_constrainable_tets_idx[idx], 2);
  const int tt4 = TT_fat(m_constrainable_tets_idx[idx], 3);
  const RowVector3d tv1 = TV_fat.row(tt1);
  const RowVector3d tv2 = TV_fat.row(tt2);
  const RowVector3d tv3 = TV_fat.row(tt3);
  const RowVector3d tv4 = TV_fat.row(tt4);

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

  auto frame_ctr = frame_for_tet(TV_fat, TT_fat, idx, angle, flipped_x);
  const Matrix3d frame = frame_ctr.first;
  const RowVector3d tet_ctr = frame_ctr.second;

  m_orientation_constraints_pos[bc_idx+0] = (tv1 - tet_ctr) * frame.transpose() + tet_ctr;
  m_orientation_constraints_pos[bc_idx+1] = (tv2 - tet_ctr) * frame.transpose() + tet_ctr;
  m_orientation_constraints_pos[bc_idx+2] = (tv3 - tet_ctr) * frame.transpose() + tet_ctr;
  m_orientation_constraints_pos[bc_idx+3] = (tv4 - tet_ctr) * frame.transpose() + tet_ctr;
}


void DeformationConstraints::slim_constraints(
    Eigen::VectorXi& slim_b,
    Eigen::MatrixXd& slim_bc,
    bool only_ends_and_tets) {
  using namespace std;
  using namespace Eigen;
  vector<RowVector3d> scaled_bone_constraints;
  scale_bone_constraints(m_scale, scaled_bone_constraints);

  slim_b.resize(num_constraints());
  slim_bc.resize(num_constraints(), 3);
  int c_count = 0;
  if (!only_ends_and_tets) {
    for (int i = 0; i < m_bone_constraints_idx.size(); i++) {
      slim_b[c_count] = m_bone_constraints_idx[i];
      slim_bc.row(c_count) = scaled_bone_constraints[i];
      c_count += 1;
    }
  } else {
    slim_b[c_count] = m_bone_constraints_idx[0];
    slim_bc.row(c_count) = scaled_bone_constraints[0];
    c_count += 1;
    slim_b[c_count] = m_bone_constraints_idx.back();
    slim_bc.row(c_count) = scaled_bone_constraints.back();
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

void DeformationConstraints::scale_bone_constraints(double amount, std::vector<Eigen::RowVector3d>& scaled) {
  using namespace Eigen;
  for (int i = 0; i < m_bone_constraints_pos.size(); i++) {
    scaled.push_back((m_bone_constraints_pos[i] - m_bone_constraints_pos.front())*amount);
  }
}






//
// Fat only overloads
//
double DeformationConstraints::one_pair_bone_constraints(
    const Eigen::MatrixXd& TV,
    const Eigen::MatrixXi& TT,
    const Eigen::VectorXd& geodesic_distances,
    const std::array<int, 2>& endpoints,
    const Eigen::RowVector3d& straight_dir,
    const Eigen::RowVector3d& straight_origin,
    int num_verts) {
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
  m_bone_constraints_pos.push_back(straight_origin);
  int tet_idx = -1;
  for (int i = 0; i < TT.rows(); i++) {
    if (TT(i, 0) == endpoints[0] || TT(i, 1) == endpoints[0] || TT(i, 2) == endpoints[0] || TT(i, 3) == endpoints[0]) {
      tet_idx = i;
      break;
    }
  }
  assert(tet_idx >= 0);
  m_constrainable_tets_idx.push_back(tet_idx);


  double dist = 0.0;
  double isovalue = geodesic_distances[endpoints[0]];
  const double isovalue_incr = (geodesic_distances[endpoints[1]] - geodesic_distances[endpoints[0]]) / num_verts;

  for(int i = 1; i < num_verts; i++) {
    isovalue += isovalue_incr;
    igl::marching_tets(TV, TT, geodesic_distances, isovalue, LV, LF);

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
    for (int k = 0; k < 4; k++) { v.row(k) = TV.row(TT(tet, k)); }
    int nv = TT(tet, nearest_vertex(v, ctr));

    if (vmap.find(nv) == vmap.end()) {
      vmap.insert(nv);
      m_bone_constraints_idx.push_back(nv);
      m_bone_constraints_pos.push_back(straight_origin + dist*straight_dir);
      m_constrainable_tets_idx.push_back(tet);
    }
  }

  dist += (TV.row(endpoints[1]) - last_ctr).norm();
  m_bone_constraints_idx.push_back(endpoints[1]);
  m_bone_constraints_pos.push_back(straight_origin + dist*straight_dir);
  tet_idx = -1;
  for (int i = 0; i < TT.rows(); i++) {
    if (TT(i, 0) == endpoints[1] || TT(i, 1) == endpoints[1] || TT(i, 2) == endpoints[1] || TT(i, 3) == endpoints[1]) {
      tet_idx = i;
      break;
    }
  }
  assert(tet_idx >= 0);
  m_constrainable_tets_idx.push_back(tet_idx);
  return dist;
}


double DeformationConstraints::update_bone_constraints(
    const Eigen::MatrixXd& TV,
    const Eigen::MatrixXi& TT,
    const Eigen::VectorXd& geodesic_distances,
    const Eigen::VectorXi& components,
    const std::vector<std::array<int, 2>>& endpoints,
    int num_verts) {
  using namespace std;
  using namespace Eigen;

  double dist = 0.0;
  const int num_endpoint_pairs = endpoints.size();
  const int num_verts_per_segment = int(ceil(double(num_verts) / num_endpoint_pairs));
  assert(num_endpoint_pairs != 0);

  vector<MatrixXi> TTcomp;
  split_mesh_components(TT, components, TTcomp);
  for (int i = 0; i < num_endpoint_pairs; i++) {
    dist += one_pair_bone_constraints(TV, TTcomp[i], geodesic_distances, endpoints[i],
                                      RowVector3d(0, 0, 1),
                                      RowVector3d(0, 0, dist),
                                      num_verts_per_segment);
    if (i != num_endpoint_pairs-1) {
      dist += (TV.row(endpoints[i+1][0]) - TV.row(endpoints[i][1])).norm();
    }
  }
  return dist;
}



