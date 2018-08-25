#include "bounding_cage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>

#include <iostream>


BoundingCage::KeyFrame::KeyFrame(const Eigen::RowVector3d& normal,
                                 const Eigen::RowVector3d& center,
                                 const Eigen::MatrixXd& pts,
                                 double idx) {
  curve_index = idx;
  coord_system = local_coordinate_system(normal);
  plane_center = center;

  points2d = pts;
}


Eigen::Matrix3d BoundingCage::KeyFrame::local_coordinate_system(const Eigen::RowVector3d& normal) {
  Eigen::RowVector3d plane_normal = normal;
  plane_normal.normalize();
  Eigen::RowVector3d plane_up = Eigen::RowVector3d(0, 1, 0);
  Eigen::RowVector3d plane_right = Eigen::RowVector3d(1, 0, 0);

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

  Eigen::Matrix3d ret;
  ret.row(0) = plane_right;
  ret.row(1) = plane_up;
  ret.row(2) = plane_normal;

  return ret;
}

bool BoundingCage::KeyFrame::move_point_2d(int i, Eigen::RowVector2d& newpos) {
  if (i < 0 || i >= points2d.rows()) {
    assert("i in move_vertex() was out of range" && false);
    return false;
  }
  Eigen::RowVector2d old_pt_2d = points2d.row(i);
  Eigen::RowVector3d old_pt_3d = points3d.row(i);
  points2d.row(i) = newpos;
  points3d.row(i) = plane_center + newpos[0]*coord_system.row(0) + newpos[1]*coord_system.row(1);

  if (!(validate_points_2d() && validate_cage())) {
    points2d.row(i) = old_pt_2d;
    points3d.row(i) = old_pt_3d;
    return false;
  }
  return true;
}

bool BoundingCage::KeyFrame::validate_points_2d() {
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

bool BoundingCage::KeyFrame::validate_cage() {
  bool ret = true;
  std::shared_ptr<Cell> left_cell = cells[0].lock();
  std::shared_ptr<Cell> right_cell = cells[1].lock();

  if (left_cell) {
    ret = ret && left_cell->update();
  }
  if(right_cell) {
    ret = ret && right_cell->update();
  }

  return ret;
}



std::shared_ptr<BoundingCage::Cell> BoundingCage::Cell::make_cell(std::shared_ptr<BoundingCage::KeyFrame> left_kf,
                                                                  std::shared_ptr<BoundingCage::KeyFrame> right_kf,
                                                                  std::shared_ptr<BoundingCage::Cell> prev,
                                                                  std::shared_ptr<BoundingCage::Cell> next) {
  std::shared_ptr<Cell> ret(new Cell(left_kf, right_kf, prev, next));

  if (!left_kf || !right_kf) {
    ret.reset();
    return ret;
  }

  left_kf->cells[1] = ret;
  right_kf->cells[0] = ret;

  Eigen::MatrixXd CHV(left_kf->points_2d().rows() + right_kf->points_2d().rows(), 3);
  CHV.block(0, 0, left_kf->points_2d().rows(), 3) = left_kf->points_3d();
  CHV.block(left_kf->points_2d().rows(), 0, right_kf->points_2d().rows(), 3) = right_kf->points_3d();

  igl::copyleft::cgal::convex_hull(CHV, ret->V, ret->F);
  if (CHV.rows() != ret->V.rows()) {
    // TODO: Log here
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    std::cerr << CHV.rows() << ", " << ret->V.rows() << std::endl;
    ret.reset();
    return ret;
  }

  igl::per_face_normals_stable(ret->V, ret->F, ret->N);
}

std::shared_ptr<BoundingCage::KeyFrame> BoundingCage::Cell::split(std::shared_ptr<KeyFrame> key_frame) {
  // The index of the keyframe is out of range, since this method is called, internally,
  // this should fail
  if (key_frame->index() > max_index() || key_frame->index() < min_index()) {
    assert("split index is out of range" && false);
    return std::shared_ptr<KeyFrame>();
  }

  // This node has already been split by the keyframe
  if (key_frame->index() == max_index()) {
    return right_keyframe;
  }
  if (key_frame->index() == min_index()) {
    return left_keyframe;
  }

  // This node is a leaf node, so split it
  if (!left_child && !right_child) {

    // Initialize new Cells for the left and right children
    left_child = Cell::make_cell(left_keyframe, key_frame);
    if (!left_child) {
      return std::shared_ptr<KeyFrame>();
    }
    right_child = Cell::make_cell(key_frame, right_keyframe);
    if (!right_child) {
      left_child.reset();
      return std::shared_ptr<KeyFrame>();
    }

    // Fix the indices of the children to keep the leaf-list valid
    left_child->prev_cell = prev_cell;
    left_child->next_cell = right_child;
    right_child->prev_cell = left_child;
    right_child->next_cell = next_cell;
    if (next_cell) { next_cell->prev_cell = right_child; }
    if (prev_cell) { prev_cell->next_cell = left_child; }

    // Fix the cell pointers in the new keyframe
    left_keyframe->cells[1] = left_child;
    right_keyframe->cells[0] = right_child;
    key_frame->cells[0] = left_child;
    key_frame->cells[1] = right_child;

    // This cell is no longer a leaf, so clear its linked list pointers
    next_cell.reset();
    prev_cell.reset();

    return key_frame;

  // This node is not a leaf node, split one of the children
  } else if(key_frame->index() > left_child->min_index() && key_frame->index() < left_child->max_index()) {
    return left_child->split(key_frame);
  } else if(key_frame->index() > right_child->min_index() && key_frame->index() < right_child->max_index()) {
    return right_child->split(key_frame);
  } else {
    assert("BoundingCage tree is in a bad state" && false);
    assert("BoundingCage tree is in a bad state" && ((left_child && !right_child) || (right_child && !left_child)));
    return std::shared_ptr<KeyFrame>();;
  }
  assert(false);
}

std::shared_ptr<BoundingCage::Cell> BoundingCage::find_cell_r(std::shared_ptr<BoundingCage::Cell> node, double index) const {
  if (!node || index < node->min_index() || index > node->max_index()) {
    return std::shared_ptr<Cell>();
  } else if (!node->left_child && !node->right_child) {
    return node;
  } else if (node->left_child && index >= node->left_child->min_index() && index <= node->left_child->max_index()) {
    return find_cell_r(node->left_child, index);
  } else if (node->right_child && index >= node->right_child->min_index() && index <= node->right_child->max_index()) {
    return find_cell_r(node->right_child, index);
  } else {
    assert("This should never happen" && false);
  }
}

bool BoundingCage::set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters, const Eigen::MatrixXd &poly_template) {
  assert(cells.begin() == cells.end());
  assert(keyframes.begin() == keyframes.end());
  assert(cells.rbegin() == cells.rend());
  assert(keyframes.rbegin() == keyframes.rend());

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

  if(poly_template.rows() < 3 || poly_template.cols() != 2) {
    // TODO: Log here
    std::cerr << "Polygon template was invalid. Needs to have at least "
                 "3 points and exactly 2 columns. Got " << poly_template.rows() <<
                 "x" << poly_template.cols() << std::endl;
    return false;
  }

  Eigen::RowVector3d front_normal = SV_smooth.row(1) - SV_smooth.row(0);
  Eigen::RowVector3d back_normal = SV_smooth.row(SV.rows()-1) - SV_smooth.row(SV.rows()-2);
  front_normal.normalize();
  back_normal.normalize();

  std::shared_ptr<KeyFrame> front_keyframe(new KeyFrame(front_normal, SV.row(0), poly_template, 0));
  std::shared_ptr<KeyFrame> back_keyframe(new KeyFrame(back_normal, SV.row(SV.rows()-1), poly_template, SV.rows()-1));

  root = Cell::make_cell(front_keyframe, back_keyframe);
  if (!root) {
    // TODO: Use the bounding box of the mesh instead
    std::cerr << "*****THIS IS BAD UNTIL I FIX IT******" << std::endl;
    assert("TODO: Use bounding box" && false);

    return false;
  }

  cells.head = root;
  cells.tail = cells.head;

  // TODO: Log if we don't fit
  fit_cage_r(root);

  return true;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> BoundingCage::plane_for_index(double index) const {
  auto cell = find_cell_r(root, index);
  if (!cell) { return std::make_pair(Eigen::VectorXd(), Eigen::VectorXd()); }

  const double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
  Eigen::MatrixXd V = (1.0-coeff)*cell->left_keyframe->points_3d() + coeff*cell->right_keyframe->points_3d();
  Eigen::RowVector3d C = (1.0-coeff)*cell->left_keyframe->center() + coeff*cell->right_keyframe->center();

  V.rowwise() -= C;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeThinV);
  return std::make_pair(C, svd.matrixV().col(2));
}

Eigen::MatrixXd BoundingCage::vertices_3d_for_index(double index) const {
  auto cell = find_cell_r(root, index);
  if (!cell) {
    return Eigen::MatrixXd();
  }

  double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
  return (1.0-coeff)*cell->left_keyframe->points_3d() + coeff*cell->right_keyframe->points_3d();
}

Eigen::MatrixXd BoundingCage::vertices_2d_for_index(double index) const {
  auto cell = find_cell_r(root, index);
  if (!cell) {
    return Eigen::MatrixXd();
  }

  const double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
  Eigen::MatrixXd V = (1.0-coeff)*cell->left_keyframe->points_3d() + coeff*cell->right_keyframe->points_3d();
  Eigen::RowVector3d C = (1.0-coeff)*cell->left_keyframe->center() + coeff*cell->right_keyframe->center();

  Eigen::MatrixXd A = V.rowwise() - C;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::RowVector3d N = svd.matrixV().col(2).transpose();

  Eigen::Matrix3d coord = KeyFrame::local_coordinate_system(N);

  Eigen::MatrixXd points2d(V.rows(), 2);
  points2d.col(0) = A*coord.row(0).transpose();
  points2d.col(1) = A*coord.row(1).transpose();

  return points2d;
}

bool BoundingCage::skeleton_in_cage(std::shared_ptr<Cell> node) const {
  int start = node->min_index();
  int end = node->max_index();

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

bool BoundingCage::fit_cage_r(std::shared_ptr<Cell> node) {
  // If all the skeleton vertices are in the cage node, then we're done
  if (skeleton_in_cage(node)) {
    return true;
  }

  // Otherwise split the cage node and try again
  const int mid = node->min_index() + (node->max_index() - node->min_index()) / 2;
  if (mid == node->min_index() || mid == node->max_index()) {
    return false;
  }

  assert("Bad mid index" && (mid > 0) && (mid_index < SV_smooth.rows()-1));

  Eigen::RowVector3d mid_normal = 0.5 * (SV_smooth.row(mid+1) - SV_smooth.row(mid-1));
  mid_normal.normalize();

  Eigen::MatrixXd pts = 0.5 * (node->left_keyframe->points_2d() + node->left_keyframe->points_2d());
  std::shared_ptr<KeyFrame> mid_keyframe(new KeyFrame(mid_normal, SV_smooth.row(mid), pts, mid));

  if(node->split(mid_keyframe)) {
    if (node == cells.head) {
      cells.head = node->left_child;
    }
    if (node == cells.tail) {
      cells.tail = node->right_child;
    }

    bool ret = fit_cage_r(node->left_child);
    return ret && fit_cage_r(node->right_child);
  } else {
    return false;
  }
}

BoundingCage::KeyFrameIterator BoundingCage::split(double index) {
  auto cell = find_cell_r(root, index);
  if (!cell) {
    return KeyFrameIterator();
  }

  const double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
  Eigen::MatrixXd V = (1.0-coeff)*cell->left_keyframe->points_3d() + coeff*cell->right_keyframe->points_3d();
  Eigen::RowVector3d C = (1.0-coeff)*cell->left_keyframe->center() + coeff*cell->right_keyframe->center();

  Eigen::MatrixXd A = V.rowwise() - C;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::RowVector3d N = svd.matrixV().col(2).transpose();

  Eigen::Matrix3d coord = KeyFrame::local_coordinate_system(N);

  Eigen::MatrixXd points2d(V.rows(), 2);
  points2d.col(0) = A*coord.row(0).transpose();
  points2d.col(1) = A*coord.row(1).transpose();

  std::shared_ptr<KeyFrame> kf(new KeyFrame(N, C, points2d, index));
  KeyFrameIterator ret(cell->split(kf));

  if (cell == cells.head) {
    cells.head = ret.keyframe->cells[0].lock();
  } else if (cell == cells.tail) {
    cells.tail = ret.keyframe->cells[1].lock();
  }

  return ret;
}
