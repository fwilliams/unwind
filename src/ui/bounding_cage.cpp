#include "bounding_cage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>

#include <iostream>


KeyFrame::KeyFrame(const Eigen::RowVector3d& normal,
                   const Eigen::RowVector3d& center,
                   const Eigen::MatrixXd& pts,
                   double idx) {
  curve_index = idx;
  plane_normal = normal;
  plane_normal.normalize();
  plane_up = Eigen::RowVector3d(0, 1, 0);
  plane_right = Eigen::RowVector3d(1, 0, 0);
  plane_center = center;

  // If the up vector and the normal are about the same
  if (fabs(1.0 - plane_normal.dot(plane_up)) < 1e-8) {
    plane_up = plane_normal.cross(plane_right);
    plane_up.normalize();
    plane_right = plane_normal.cross(plane_up);
    plane_right.normalize();
  } else if (fabs(1.0 - plane_normal.dot(plane_right)) < 1e-8) {
    plane_right = plane_normal.cross(plane_up);
    plane_right.normalize();
    plane_up = plane_normal.cross(plane_right);
    plane_up.normalize();
  } else {
    plane_up = plane_normal.cross(plane_right);
    plane_up.normalize();
    plane_right = plane_normal.cross(plane_up);
    plane_right.normalize();
  }

  points2d = pts;
}

bool KeyFrame::move_point_2d(int i, Eigen::RowVector2d& newpos) {
  if (i < 0 || i >= points2d.rows()) {
    assert("i in move_vertex() was out of range" && false);
    return false;
  }
  Eigen::RowVector2d old_pt_2d = points2d.row(i);
  Eigen::RowVector3d old_pt_3d = points3d.row(i);
  points2d.row(i) = newpos;
  points3d.row(i) = plane_center + newpos[0]*plane_right + newpos[1]*plane_up;

  if (!(validate_points_2d() && validate_cage())) {
    points2d.row(i) = old_pt_2d;
    points3d.row(i) = old_pt_3d;
    return false;
  }
  return true;
}

bool KeyFrame::validate_points_2d() {
  if (points2d.rows() != points2d.rows()) {
    assert("points 2d has wrong number of rows" && false);
    return false;
  }
  if (points2d.cols() != 2) {
    assert("points 2d has wrong number of cols" && false);
    return false;
  }

  // TODO: Ensure there are no self intersections so we never get in a bad state
  return true;
}

bool KeyFrame::validate_cage() {
  bool ret = true;
  if (cage_nodes[0]) {
    ret = ret && cage_nodes[0]->update();
  }
  if(cage_nodes[1]) {
    ret = ret && cage_nodes[1]->update();
  }

  return ret;
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

double CageNode::left_index() const {
  return left_keyframe->index();
}

double CageNode::right_index() const {
  return right_keyframe->index();
}

bool CageNode::split(std::shared_ptr<KeyFrame> key_frame) {
  if (key_frame->index() > right_index() || key_frame->index() < left_index()) {
    return false;
  }

  // This node has already been split by the keyframe
  if (key_frame->index() == right_index() || key_frame->index() == left_index()) {
    return true;
  }

  if (!left_child && !right_child) {
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
    return left_child->split(key_frame);
  } else if(key_frame->index() > right_child->left_index() && key_frame->index() < right_child->right_index()) {
    return right_child->split(key_frame);
  }

  assert("BoundingCage tree is in a bad state" && false);
  assert("BoundingCage tree is in a bad state" && ((left_child && !right_child) || (right_child && !left_child)));
  return false;
}

std::shared_ptr<CageNode> BoundingCage::find_cage_node(std::shared_ptr<CageNode> node, double index) {
  if (!node || index < node->left_index() || index > node->right_index()) {
    return std::shared_ptr<CageNode>();
  } else if (!node->left_child && !node->right_child) {
    return node;
  } else if (node->left_child && index >= node->left_child->left_index() && index <= node->left_child->right_index()) {
    return find_cage_node(node->left_child, index);
  } else if (node->right_child && index >= node->right_child->left_index() && index <= node->right_child->right_index()) {
    return find_cage_node(node->right_child, index);
  } else {
    assert("This should never happen" && false);
  }
}

Eigen::MatrixXd BoundingCage::polygon_template() {
  Eigen::MatrixXd poly_template(4, 2);
  poly_template << -1, -1,
                     1, -1,
                     1,  1,
                    -1,  1;

  const double rad = 20.0;
  return rad * poly_template;
}

bool BoundingCage::set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters) {
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
    assert("TODO: Use bounding box" && false);

    return false;
  }

  head = root;
  tail = head;

  fit_cage_r(root);

  return true;
}

Eigen::MatrixXd BoundingCage::intersecting_plane(double index) {
  auto node = find_cage_node(root, index);
  if (!node) {
    return Eigen::MatrixXd();
  }

  double coeff = (index - node->left_index()) / (node->right_index() - node->left_index());

  return (1.0-coeff)*node->left_keyframe->points_3d() + coeff*node->right_keyframe->points_3d();
}

bool BoundingCage::skeleton_in_cage(std::shared_ptr<CageNode> node) {
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

bool BoundingCage::fit_cage_r(std::shared_ptr<CageNode> node) {
  // If all the skeleton vertices are in the cage node, then we're done
  if (skeleton_in_cage(node)) {
    return true;
  }

  // Otherwise split the cage node and try again
  const int mid = node->left_index() + (node->right_index() - node->left_index()) / 2;
  if (mid == node->left_index() || mid == node->right_index()) {
    return false;
  }

  assert("Bad mid index" && (mid > 0) && (mid_index < SV_smooth.rows()-1));

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

    bool ret = fit_cage_r(node->left_child);
    return ret && fit_cage_r(node->right_child);
  } else {
    return false;
  }
}
