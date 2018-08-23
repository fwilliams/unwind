#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <iostream>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>
#include <igl/winding_number.h>

#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H

struct CageNode;
struct KeyFrame;
struct Cage;


struct CageNode {
  CageNode() {}

  static std::shared_ptr<CageNode> make_cage_node(std::shared_ptr<KeyFrame> front,
                                                  std::shared_ptr<KeyFrame> back,
                                                  std::shared_ptr<CageNode> prev=std::shared_ptr<CageNode>(),
                                                  std::shared_ptr<CageNode> next=std::shared_ptr<CageNode>());

  std::shared_ptr<CageNode> left_child;
  std::shared_ptr<CageNode> right_child;
  std::shared_ptr<CageNode> next;
  std::shared_ptr<CageNode> prev;

  std::shared_ptr<KeyFrame> left_keyframe;
  std::shared_ptr<KeyFrame> right_keyframe;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd N;

  bool split(std::shared_ptr<KeyFrame> key_frame);
  bool split();

  int left_index() const;

  int right_index() const;

private:
  friend class KeyFrame;

  bool update() {
    // Check that the edges in the two endplanes don't cross
    return true;
  }
};

struct KeyFrame {
  KeyFrame(const Eigen::RowVector3d& normal,
           const Eigen::RowVector3d& center,
           const Eigen::MatrixXd& pts,
           int idx);

  const Eigen::RowVector3d& normal() const {
    return plane_normal;
  }

  const Eigen::RowVector3d& center() const {
    return plane_center;
  }

  const Eigen::MatrixXd& points_2d() const {
    return points2d;
  }

  const Eigen::MatrixXd& points_3d() {
    // TODO: Recomputing this every frame might be a bit slow but we'll optimize it if we need to
    points3d.conservativeResize(points2d.rows(), 3);
    for (int i = 0; i < points2d.rows(); i++) {
      points3d.row(i) = plane_center + points2d(i, 0) * plane_right + points2d(i, 1) * plane_up;
    }
    return points3d;
  }

  const int index() const {
    return curve_index;
  }

  bool move_point_2d(int i, Eigen::RowVector2d& newpos);

private:
  // If a vertex is changed, these functions validate that we haven't
  // created any local self-interesections in the bounding cage.
  bool validate_points_2d();
  bool validate_cage();

  Eigen::RowVector3d plane_right;
  Eigen::RowVector3d plane_up;
  Eigen::RowVector3d plane_normal;
  Eigen::RowVector3d plane_center;
  Eigen::MatrixXd points2d;
  Eigen::MatrixXd points3d;
  int curve_index;

  std::array<std::shared_ptr<CageNode>, 2> cage_nodes;
};


struct Cage {

  Cage() {}

  // Root node of the cage tree
  std::shared_ptr<CageNode> root;

  // Linked list of the bottom level of the tree
  std::shared_ptr<CageNode> head;
  std::shared_ptr<CageNode> tail;

  static Eigen::MatrixXd polygon_template();

  bool set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters);

  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

private:
  bool fit_cage_r(std::shared_ptr<CageNode> node);
  bool skeleton_in_cage(std::shared_ptr<CageNode> node);

  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;
};

#endif // BOUNDING_CAGE_H
