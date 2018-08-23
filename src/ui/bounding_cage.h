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
           int idx) {
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
    update_points_3d();
  }

  const Eigen::RowVector3d& normal() const {
    return plane_normal;
  }

  const Eigen::RowVector3d& center() const {
    return plane_center;
  }

  const Eigen::MatrixXd& points_2d() const {
    return points2d;
  }

  const Eigen::MatrixXd& points_3d() const {
    return points3d;
  }

  const int index() const {
    return curve_index;
  }

  bool move_vertex(int i, Eigen::RowVector2d& point) {
    if (i < 0 || i >= points2d.rows()) {
      assert("i in move_vertex() was out of range" && false);
      return false;
    }
    Eigen::RowVector2d old_pt_2d = points2d.row(i);
    Eigen::RowVector3d old_pt_3d = points3d.row(i);
    points2d.row(i) = point;
    points3d.row(i) = plane_center + point[0]*plane_right + point[1]*plane_up;

    if (!(validate_points_2d() && validate_cage())) {
      points2d.row(i) = old_pt_2d;
      points3d.row(i) = old_pt_3d;
      return false;
    }
    return true;
  }

private:
  bool validate_points_2d() {
    if (points2d.rows() != points2d.rows()) {
      assert(false);
      return false;
    }
    if (points2d.cols() != 2) {
      assert(false);
      return false;
    }

    // TODO: Ensure there are no self intersections so we never get in a bad state
    return true;
  }

  bool validate_cage() {
    bool ret = true;
    if (cage_nodes[0]) {
      ret = ret && cage_nodes[0]->update();
    }
    if(cage_nodes[1]) {
      ret = ret && cage_nodes[1]->update();
    }

    return ret;
  }

  void update_points_3d() {
    points3d.conservativeResize(points2d.rows(), 3);
    for (int i = 0; i < points2d.rows(); i++) {
      points3d.row(i) = plane_center + points2d(i, 0) * plane_right + points2d(i, 1) * plane_up;
    }
  }

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

private:
  bool fit_cage_r(std::shared_ptr<CageNode> node);
  bool skeleton_in_cage(std::shared_ptr<CageNode> node);

  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;
};

class BoundingCage {
public:
  struct BoundingCageNode {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F; // Faces which index into CV
    Eigen::MatrixXd C; // Center positions of cage faces
    Eigen::MatrixXd N; // Normals of cage component faces

    int level = -1;

    int start = -1;
    int end = -1;

    std::shared_ptr<BoundingCageNode> left;
    std::shared_ptr<BoundingCageNode> right;

    bool is_leaf() const { return !left && !right; }
  };

  typedef std::shared_ptr<BoundingCageNode> CageNodePtr;
  typedef std::shared_ptr<KeyFrame> KeyFramePtr;

  BoundingCage();

  const std::vector<std::shared_ptr<BoundingCage::BoundingCageNode>>& components() const { return cage_components; }

  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

  void set_skeleton_vertices(const Eigen::MatrixXd& SV, unsigned smoothing_iters=0);

private:
  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;

  // Cage mesh
  Eigen::MatrixXd CV;
  Eigen::MatrixXd CF;

  void compute_cage_mesh();

  std::shared_ptr<BoundingCageNode> root;
  std::vector<std::shared_ptr<BoundingCageNode>> cage_components;

  bool split_node(std::shared_ptr<BoundingCageNode>, int idx);

  bool make_bounding_cage_r(std::shared_ptr<BoundingCageNode> root);
  bool skeleton_in_cage(const Eigen::MatrixXd& CC, const Eigen::MatrixXd& CN, int start, int end);
  std::shared_ptr<BoundingCageNode> make_bounding_cage_component(int v1, int v2, int level);

  Eigen::MatrixXd polygon_for_vertex(int vid, const Eigen::MatrixXd& P) {}

  void smooth_skeleton(int num_iters);

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> plane_for_vertex(int vid, double radius);
};

#endif // BOUNDING_CAGE_H
