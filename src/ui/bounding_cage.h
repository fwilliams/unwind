#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <iostream>

#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H

class BoundingCage {
public:
  class KeyFrame;
  class Cell;

private:
  // The initial polygon shape to use
  static Eigen::MatrixXd polygon_template();

  // Fit the inital cage to the skeleton
  bool fit_cage_r(std::shared_ptr<Cell> node);

  // Return true if a node contains its skeleton
  bool skeleton_in_cage(std::shared_ptr<Cell> node) const;

  // Find the cage node for a given index in the tree. If index is out of range, return null
  std::shared_ptr<Cell> find_cell_r(std::shared_ptr<Cell> node, double index) const;

  // Skeleton Vertices
  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;

  // Root node of the cage tree
  std::shared_ptr<Cell> root;

public:
  class Cell {
    friend class BoundingCage;

    static std::shared_ptr<Cell> make_cell(std::shared_ptr<BoundingCage::KeyFrame> front,
                                           std::shared_ptr<BoundingCage::KeyFrame> back,
                                           std::shared_ptr<Cell> prev_cell=std::shared_ptr<Cell>(),
                                           std::shared_ptr<Cell> next_cell=std::shared_ptr<Cell>());
    std::shared_ptr<Cell> left_child;
    std::shared_ptr<Cell> right_child;
    std::shared_ptr<Cell> next_cell;
    std::shared_ptr<Cell> prev_cell;

    std::shared_ptr<KeyFrame> left_keyframe;
    std::shared_ptr<KeyFrame> right_keyframe;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd N;

    bool split(std::shared_ptr<KeyFrame> key_frame);
    bool split();


    bool update() {
      // Check that the edges in the two endplanes don't cross
      return true;
    }

    Cell(std::shared_ptr<KeyFrame> left_kf,
         std::shared_ptr<KeyFrame> right_kf,
         std::shared_ptr<Cell> prev,
         std::shared_ptr<Cell> next) :
      left_keyframe(left_kf), right_keyframe(right_kf),
      prev_cell(prev), next_cell(next) {}

  public:
    const Eigen::MatrixXd& vertices() const { return V; }
    const Eigen::MatrixXi& faces() const { return F; }
    const Eigen::MatrixXd& normals() const { return N; }
    double min_index() const { return left_keyframe->index(); }
    double max_index() const { return right_keyframe->index(); }
  };

  class CellIterator {
    friend class BoundingCage;

    std::shared_ptr<Cell> cell;

    CellIterator(std::shared_ptr<Cell> c) : cell(c) {}

  public:
    CellIterator(const CellIterator& other) : cell(other.cell) {}
    CellIterator() : cell(std::shared_ptr<Cell>()) {}
    CellIterator& operator=(const CellIterator& other) { cell = other.cell; }

    CellIterator operator++() {
      if (cell) {
        cell = cell->next_cell;
      }
      return *this;
    }

    CellIterator operator++(int) {
      return operator++();
    }

    CellIterator operator--() {
      if (cell) {
        cell = cell->prev_cell;
      }
      return *this;
    }

    CellIterator operator--(int) {
      return operator--();
    }

    bool operator==(const CellIterator& other) const {
      return cell == other.cell;
    }

    bool operator!=(const CellIterator& other) const {
      return cell != other.cell;
    }

    std::shared_ptr<Cell> operator->() {
      return cell;
    }

    Cell& operator*() {
      return *cell;
    }
  };

  // Linked list representing all the cells in the cage.
  // These correspond to the leaf nodes of the cell tree.
  // This data structure can be used to iterate over the cage cells in order.
  class Cells {
    friend class BoundingCage;

    std::shared_ptr<Cell> head;
    std::shared_ptr<Cell> tail;
  public:
    CellIterator begin() const { return CellIterator(head); }
    CellIterator end() const { return CellIterator(); }
    CellIterator rbegin() const { return CellIterator(tail); }
    CellIterator rend() const { return CellIterator(); }
  } cells;

  class KeyFrame {
    friend class BoundingCage;

    KeyFrame(const Eigen::RowVector3d& normal,
             const Eigen::RowVector3d& center,
             const Eigen::MatrixXd& pts,
             double idx);

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
    double curve_index;

    std::array<std::weak_ptr<BoundingCage::Cell>, 2> cells;

  public:
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

    const double index() const {
      return curve_index;
    }

    bool move_point_2d(int i, Eigen::RowVector2d& newpos);
  };

  class KeyFrameIterator {
    friend class BoundingCage;

    std::shared_ptr<KeyFrame> keyframe;

    KeyFrameIterator(std::shared_ptr<KeyFrame> kf) : keyframe(kf) {}

  public:
    KeyFrameIterator(const KeyFrameIterator& other) : keyframe(other.keyframe) {}
    KeyFrameIterator() : keyframe(std::shared_ptr<KeyFrame>()) {}
    KeyFrameIterator& operator=(const KeyFrameIterator& other) { keyframe = other.keyframe; }

    KeyFrameIterator operator++() {
      std::shared_ptr<Cell> right_cell = keyframe->cells[1].lock();
      if (keyframe && right_cell) {
        keyframe = right_cell->right_keyframe;
      } else if(!right_cell) {
        keyframe.reset();
      }
      return *this;
    }

    KeyFrameIterator operator++(int) {
      return operator++();
    }

    KeyFrameIterator operator--() {
      std::shared_ptr<Cell> left_cell = keyframe->cells[0].lock();
      if (keyframe && left_cell) {
        keyframe = left_cell->left_keyframe;
      } else if(!left_cell) {
        keyframe.reset();
      }
      return *this;
    }

    KeyFrameIterator operator--(int) {
      return operator--();
    }

    bool operator==(const KeyFrameIterator& other) const {
      return keyframe == other.keyframe;
    }

    bool operator!=(const KeyFrameIterator& other) const {
      return keyframe != other.keyframe;
    }

    std::shared_ptr<KeyFrame> operator->() {
      return keyframe;
    }

    KeyFrame& operator*() {
      return *keyframe;
    }
  };

  // Linked list of keyframes built upon the linked list of cells.
  // This data structure can be used to iterate over the keyframes in order.
  class KeyFrames {
    friend class BoundingCage;

    const BoundingCage* cage;

  public:
    KeyFrameIterator begin() {
      if(cage->cells.head) {
        return KeyFrameIterator(cage->cells.head->left_keyframe);
      } else {
        return KeyFrameIterator();
      }
    }

    KeyFrameIterator end() {
      return KeyFrameIterator();
    }

    KeyFrameIterator rbegin() {
      if(cage->cells.tail) {
        return KeyFrameIterator(cage->cells.head->left_keyframe);
      } else {
        return KeyFrameIterator();
      }
    }

    KeyFrameIterator rend() {
      return KeyFrameIterator();
    }

  } keyframes;

  BoundingCage() { keyframes.cage = this; }

  // Set the skeleton vertices to whatever the user provides.
  // If the vertices are invalid, return false
  bool set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters);

  // Get the skeleton vertex positions
  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

  // Get the minimum keyframe index
  double min_index() const {
    if(!root) {
      return 0.0;
    }
    assert(root->min_index() == cells.head->min_index());
    return root->min_index();
  }

  // Get the maximum keyframe index
  double max_index() const {
    if(!root) {
      return 0.0;
    }
    assert(root->max_index() == cells.tail->max_index());
    return root->max_index();
  }

  // Get the 2d or 3d vertices for the plane cutting the cage at index.
  // Returns an empty matrix if the index is out of range
  Eigen::MatrixXd vertices_3d_for_index(double index) const;
  Eigen::MatrixXd vertices_2d_for_index(double index) const;

  // Get the plane cutting the cage at index
  std::pair<Eigen::RowVector3d, Eigen::RowVector3d> plane_for_index(double index) const;

  // Clear the bounding cage
  void clear() {
    root.reset();
    cells.head.reset();
    cells.tail.reset();
    SV.resize(0, 0);
    SV_smooth.resize(0, 0);
  }
};



#endif // BOUNDING_CAGE_H
