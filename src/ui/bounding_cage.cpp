#include "bounding_cage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>

#include <iostream>


template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


BoundingCage::BoundingCage() {}


void BoundingCage::set_skeleton_vertices(const Eigen::MatrixXd& sv, unsigned smoothing_iters) {
  SV = sv;
  smooth_skeleton(smoothing_iters);

  root = make_bounding_cage_component(0, SV.rows()-2, 0 /* level */);
  if (!root) {
    // TODO: Use the bounding box of the mesh instead
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    assert(false);
  }

  make_bounding_cage_r(root);
}


bool BoundingCage::make_bounding_cage_r(std::shared_ptr<BoundingCageNode> root) {
  // If the node is in the cage, then we're done
  if (skeleton_in_cage(root->C, root->N, root->start, root->end)) {
    cage_components.push_back(root);
    return true;
  }

  // Otherwise split and recurse
  const int mid = root->start + (root->end - root->start) / 2;

  root->left = make_bounding_cage_component(root->start, mid, root->level+1);
  if (root->left) {
    make_bounding_cage_r(root->left);
  } else {
    root->left.reset();
    cage_components.push_back(root);
    return true;
  }

  root->right = make_bounding_cage_component(mid, root->end, root->level+1);
  if (root->right) {
    make_bounding_cage_r(root->right);
  } else {
    root->right.reset();
    cage_components.push_back(root);
    return true;
  }

  return true;
}


std::shared_ptr<BoundingCage::BoundingCageNode>
BoundingCage::make_bounding_cage_component(int v1, int v2, int level) {
  std::shared_ptr<BoundingCageNode> node = std::make_shared<BoundingCageNode>();
  if ((v2 - v1) <= 1) {
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

bool BoundingCage::skeleton_in_cage(
    const Eigen::MatrixXd& CC, const Eigen::MatrixXd& CN, int start, int end) {
  assert(start < end);
  if ((end - start) <= 1) {
    return false;
  }

  const int check_sign = sgn(CN.row(0).dot(SV_smooth.row(start+1)-CC.row(0)));
  for (int i = 0; i < end-start-1; i++) {
    const Eigen::RowVector3d V = SV_smooth.row(start+1+i);
    for (int j = 0; j < CN.rows(); j++) {
      const int sign = sgn(CN.row(j).dot(V-CC.row(j)));
      if (sign != check_sign) {
        return false;
      }
    }
  }

  return true;
}


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


std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> BoundingCage::plane_for_vertex(int vid, double radius) {
  Eigen::MatrixXd PV;
  Eigen::MatrixXi PF;
  Eigen::RowVector3d n1 = SV_smooth.row(vid+1) - SV_smooth.row(vid);
  n1.normalize();

  Eigen::RowVector3d right1(1, 0, 0);
  Eigen::RowVector3d up1 = right1.cross(n1);
  up1.normalize();
  right1 = up1.cross(n1);

  make_plane(n1, up1, SV_smooth.row(vid), radius, PV, PF);

  return std::make_tuple(PV, PF);
}

void BoundingCage::smooth_skeleton(int num_iters) {
  Eigen::MatrixXd SV_i = SV;
  SV_smooth.resize(SV_i.rows(), 3);
  for (int iter = 0; iter < num_iters; iter++) {
    SV_smooth.row(0) = SV_i.row(0);
    for (int i = 1; i < SV_i.rows()-1; i++) {
      SV_smooth.row(i) = 0.5 * (SV_i.row(i-1) + SV_i.row(i+1));
    }
    SV_smooth.row(SV_i.rows()-1) = SV_i.row(SV_i.rows()-1);
  }
}

