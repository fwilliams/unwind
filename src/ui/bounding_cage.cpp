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

  root = make_bounding_cage_component(0, SV.rows()-1, 0 /* level */);
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
  Eigen::RowVector3d n1;
  if (vid == SV.rows()-1) {
    n1 = SV_smooth.row(vid) - SV_smooth.row(vid-1);
  } else if (vid == 0) {
    n1 = SV_smooth.row(vid+1) - SV_smooth.row(vid);
  } else {
    n1 = 0.5 * (SV_smooth.row(vid+1) - SV_smooth.row(vid-1));
  }
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
    SV_i = SV_smooth;
  }
}




















std::shared_ptr<CageNode> CageNode::make_cage_node(std::shared_ptr<KeyFrame> front,
                                                   std::shared_ptr<KeyFrame> back,
                                                   std::shared_ptr<CageNode> prev,
                                                   std::shared_ptr<CageNode> next) {
  std::shared_ptr<CageNode> ret = std::make_shared<CageNode>();

  if (!front || !back) {
    ret.reset();
    return ret;
  }

  ret->left_keyframe = front;
  ret->right_keyframe = back;
  ret->prev = prev;
  ret->next = next;

  Eigen::MatrixXd CHV(front->points_2d().rows() + back->points_2d().rows(), 3);
  CHV.block(0, 0, front->points_2d().rows(), 3) = front->points_3d();
  CHV.block(front->points_2d().rows(), 0, back->points_2d().rows(), 3) = back->points_3d();

  igl::copyleft::cgal::convex_hull(CHV, ret->V, ret->F);
  if (CHV.rows() != ret->V.rows()) {
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    ret.reset();
    return ret;
  }

  igl::per_face_normals_stable(ret->V, ret->F, ret->N);
}

int CageNode::left_index() const {
  return left_keyframe->index();
}

int CageNode::right_index() const {
  return right_keyframe->index();
}

bool CageNode::split(std::shared_ptr<KeyFrame> key_frame) {
  if (key_frame->index() > right_index() || key_frame->index() < left_index()) {
    std::cout << "split: index out of range" << std::endl;
    std::cout << "split: left_keyframe index = " << left_index() << std::endl;
    std::cout << "split: new_keyframe index = " << key_frame->index() << std::endl;
    std::cout << "split: right_keyframe index = " << right_index() << std::endl;
    return false;
  }

  // This node has already been split by the keyframe
  if (key_frame->index() == right_index() || key_frame->index() == left_index()) {
    std::cout << "split: index already exists range" << std::endl;
    return true;
  }

  if (!left_child && !right_child) {
    std::cout << "split: is leaf, splitting that shit" << std::endl;
    std::cout << "split: left_keyframe index is " << left_keyframe->index() << std::endl;
    std::cout << "split: new_keyframe index is " << key_frame->index() << std::endl;
    std::cout << "split: right_keyframe index is " << right_keyframe->index() << std::endl;
    left_child = CageNode::make_cage_node(left_keyframe, key_frame);
    if (!left_child) {
      return false;
    }

    right_child = CageNode::make_cage_node(key_frame, right_keyframe);
    if (!right_child) {
      left_child.reset();
      return false;
    }

    left_child->prev = prev;
    left_child->next = right_child;
    right_child->prev = left_child;
    right_child->next = next;
    if (next) { next->prev = right_child; }
    if (prev) { prev->next = left_child; }

    next.reset();
    prev.reset();

    return true;
  } else if(key_frame->index() > left_child->left_index() && key_frame->index() < left_child->right_index()) {
    std::cout << "split: recurse left" << std::endl;
    return left_child->split(key_frame);
  } else if(key_frame->index() > right_child->left_index() && key_frame->index() < right_child->right_index()) {
    std::cout << "split: recurse right" << std::endl;
    return right_child->split(key_frame);
  }

  assert("BoundingCage tree is in a bad state" && false);
  assert("BoundingCage tree is in a bad state" && ((left_child && !right_child) || (right_child && !left_child)));
  return false;
}


Eigen::MatrixXd Cage::polygon_template() {
  Eigen::MatrixXd poly_template(4, 2);
  poly_template << -1, -1,
                     1, -1,
                     1,  1,
                    -1,  1;

  const double rad = 20.0;
  return rad * poly_template;
}

bool Cage::set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters) {
  auto smooth_skeleton = [&](int num_iters) {
    Eigen::MatrixXd SV_i = SV;
    SV_smooth.resize(SV_i.rows(), 3);
    for (int iter = 0; iter < num_iters; iter++) {
      SV_smooth.row(0) = SV_i.row(0);
      for (int i = 1; i < SV_i.rows()-1; i++) {
        SV_smooth.row(i) = 0.5 * (SV_i.row(i-1) + SV_i.row(i+1));
      }
      SV_smooth.row(SV_i.rows()-1) = SV_i.row(SV_i.rows()-1);
      SV_i = SV_smooth;
    }
  };

  SV = new_SV;

  if (SV.rows() <= 1) {
    return false;
  }

  smooth_skeleton(smoothing_iters);

  Eigen::MatrixXd poly_template = polygon_template();

  Eigen::RowVector3d front_normal = SV_smooth.row(1) - SV_smooth.row(0);
  Eigen::RowVector3d back_normal = SV_smooth.row(SV.rows()-1) - SV_smooth.row(SV.rows()-2);
  front_normal.normalize();
  back_normal.normalize();

  std::shared_ptr<KeyFrame> front_keyframe =
      std::make_shared<KeyFrame>(front_normal, SV.row(0), poly_template, 0);
  std::shared_ptr<KeyFrame> back_keyframe =
      std::make_shared<KeyFrame>(back_normal, SV.row(SV.rows()-1), poly_template, SV.rows()-1);

  root = CageNode::make_cage_node(front_keyframe, back_keyframe);
  if (!root) {
    // TODO: Use the bounding box of the mesh instead
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    assert(false);

    return false;
  }

  head = root;
  tail = head;

  fit_cage_r(root);

  return true;
}

bool Cage::skeleton_in_cage(std::shared_ptr<CageNode> node) {
  int start = node->left_index();
  int end = node->right_index();

  assert(start < end);
  if ((end - start) <= 1) {
    return false;
  }

  auto sgn = [](double val) -> int {
      return (double(0) < val) - (val < double(0));
  };

  Eigen::MatrixXd C(node->F.rows(), 3);
  for (int i = 0; i < node->F.rows(); i++) {
    C.row(i) = (node->V.row(node->F(i, 0)) +
                node->V.row(node->F(i, 1)) +
                node->V.row(node->F(i, 2))) / 3;
  }

  const int check_sign = sgn(node->N.row(0).dot(SV_smooth.row(start+1)-C.row(0)));
  for (int i = 0; i < end-start-1; i++) {
    const Eigen::RowVector3d V = SV_smooth.row(start+1+i);
    for (int j = 0; j < node->N.rows(); j++) {
      const int sign = sgn(node->N.row(j).dot(V-C.row(j)));
      if (sign != check_sign) {
        return false;
      }
    }
  }

  return true;
}

bool Cage::fit_cage_r(std::shared_ptr<CageNode> node) {
  // If all the skeleton vertices are in the cage node, then we're done
  if (skeleton_in_cage(node)) {
    std::cout << "Skeleton in cage!" << std::endl;
    return true;
  }

  std::cout << "fit_cage_r: left_idx = " << node->left_index() << ", right_idx = " << node->right_index() << std::endl;
  // Otherwise split the cage node and try again
  const int mid = node->left_index() + (node->right_index() - node->left_index()) / 2;
  if (mid == node->left_index() || mid == node->right_index()) {
    std::cout << "bailout!" << std::endl;
    return false;
  }

  Eigen::RowVector3d mid_normal = 0.5 * (SV_smooth.row(mid+1) - SV_smooth.row(mid-1));
  mid_normal.normalize();

  Eigen::MatrixXd pts = 0.5 * (node->left_keyframe->points_2d() + node->left_keyframe->points_2d());
  std::shared_ptr<KeyFrame> mid_keyframe =
      std::make_shared<KeyFrame>(mid_normal, SV_smooth.row(mid), pts, mid);

  if(node->split(mid_keyframe)) {
    if (node == head) {
      head = node->left_child;
    }
    if (node == tail) {
      tail = node->right_child;
    }

    std::cout << "splitting" << std::endl;
    bool ret = fit_cage_r(node->left_child);
    return ret && fit_cage_r(node->right_child);
  } else {
    std::cout << "failed to split" << std::endl;
    return false;
  }
}
