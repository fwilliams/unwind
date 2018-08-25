#include <Eigen/Core>

#include <memory>

#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H

class BoundingCage {
public:
  class KeyFrame;
  class Cell;

private:
  /// Fit the an initial BoundingCage to the skeleton. This will
  /// attempt to construct a series of prisms which fully enclose the skeleton
  /// vertices.
  ///
  /// If the resulting BoundingCage does not contain all the skeleton vertices,
  /// this method returns false.
  ///
  bool fit_cage_r(std::shared_ptr<Cell> node);

  /// Return true if a Cell contains the skeleton vertices corresponding
  /// to its index range, and false otherwise.
  ///
  bool skeleton_in_cage(std::shared_ptr<Cell> node) const;

  /// Find the Cell for a given index in the Cell tree.
  /// If index is out of range, this method returns a null pointer.
  ///
  std::shared_ptr<Cell> find_cell_r(std::shared_ptr<Cell> node, double index) const;

  /// Skeleton Vertices
  ///
  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;

  /// Root node of the Cell tree
  ///
  std::shared_ptr<Cell> root;

public:
  /// A Cell represents a prism whose bases are two keyframes which are indexed proportionally
  /// to their distance along the skeleton of the bounding cage. A Cell's "left" keyframe always
  /// has a smaller index than its "right" keyframe.
  ///
  /// A Cell can be split into two cells by adding a keyframe in the middle of the Cell whose
  /// index lies between the "left" and "right" keyframes.
  ///
  /// Cells are organized in a binary tree structure, so splitting a cell creates two
  /// child cells. The leaf nodes of the binary tree are the set of all prisms making up
  /// the bounding cage. These leaves are organized in a linked-list ordered by keyframe
  /// index.
  ///
  /// Cells do not expose any public methods which can mutate the class and thus, can be
  /// considered immutable.
  ///
  class Cell {
    friend class BoundingCage;

    /// Construct a new Cell wrapped in a shared_ptr. Internally, this method is used
    /// to create Cells in lieu of the constructor.
    static std::shared_ptr<Cell> make_cell(std::shared_ptr<BoundingCage::KeyFrame> left_kf,
                                           std::shared_ptr<BoundingCage::KeyFrame> right_kf,
                                           std::shared_ptr<Cell> prev_cell=std::shared_ptr<Cell>(),
                                           std::shared_ptr<Cell> next_cell=std::shared_ptr<Cell>());

    /// A Cell is a binary tree and can contain sub-Cells
    ///
    std::shared_ptr<Cell> left_child;
    std::shared_ptr<Cell> right_child;

    /// Cells which are leaves of the tree are linked together in KeyFrame index order.
    /// This linked list of leaf nodes is used to construct the mesh of the bounding cage
    ///
    std::shared_ptr<Cell> next_cell;
    std::shared_ptr<Cell> prev_cell;

    /// The "left" and "right" KeyFrames. The left has a smaller index than the right.
    ///
    std::shared_ptr<KeyFrame> left_keyframe;
    std::shared_ptr<KeyFrame> right_keyframe;

    /// Cached mesh information about this Cell
    ///
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd N;

    /// Split the Cell into two cells divided by key_frame.
    /// If the index of key_frame is outside the cell, this method
    /// returns false and the Cell remains unchanged.
    ///
    std::shared_ptr<KeyFrame> split(std::shared_ptr<KeyFrame> key_frame);

    /// This method gets called when one of the "left" or "right" KeyFrame
    /// changes. The update method propagates the change in state of the KeyFrame
    /// to the cell.
    ///
    /// If the change in KeyFrame results in local self-intersections, this method
    /// returns false and the cell remains unchanged. In this case, The calling
    /// KeyFrame reverts its state to before the change.
    ///
    bool update() {
      // TODO: Check that the edges in the two endplanes don't cross
      return true;
    }

    /// Construct a new Cell. Don't call this directly, instead use the factory
    /// function `make_cell()`.
    ///
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

  /// Bidirectional Iterator class used to traverse the linked list of leaf Cells
  /// in KeyFrame-index order.
  ///
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

  /// Linked list of Cell prisms which make up the bounding cage.
  /// These correspond to the keyframe-index ordered leaf nodes of the Cell tree.
  ///
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

    static Eigen::Matrix3d local_coordinate_system(const Eigen::RowVector3d& normal);

    KeyFrame(const Eigen::RowVector3d& normal,
             const Eigen::RowVector3d& center,
             const Eigen::MatrixXd& pts,
             double idx);

    /// When polygon vertex changes (via `set_point_2d()`), these methods
    /// validate that the change does not create self intersections.
    ///
    /// If the change is valid, then these methods propagate it to the rest
    /// of the bounding cage.
    ///
    bool validate_points_2d();
    bool validate_cage();

    /// State representing the plane for this KeyFrame.
    ///
    Eigen::Matrix3d coord_system;
    Eigen::RowVector3d plane_center;

    /// Cached positions of the bounding polygon for this KeyFrame in 2d
    /// and 3d.
    Eigen::MatrixXd points2d;
    Eigen::MatrixXd points3d;

    /// The index of this KeyFrame.
    double curve_index;

    std::array<std::weak_ptr<BoundingCage::Cell>, 2> cells;

  public:
    /// Get the normal of the plane of this KeyFrame.
    ///
    Eigen::RowVector3d normal() const {
      return coord_system.row(2);
    }

    /// Get the up basis vector of the coordinate system of this KeyFrame.
    ///
    Eigen::RowVector3d up() const {
      return coord_system.row(1);
    }

    /// Get the right basis vector of the coordinate system of this KeyFrame.
    ///
    Eigen::RowVector3d right() const {
      return coord_system.row(0);
    }

    /// Get the local coordinate system of this KeyFrame.
    /// 2d positions, (x, y), of this keyframe represent coefficients
    /// along the first and second rows of this system.
    ///
    const Eigen::Matrix3d& coordinate_system() const {
      return coord_system;
    }

    /// Get the center of the KeyFrame.
    ///
    const Eigen::RowVector3d& center() const {
      return plane_center;
    }

    /// Get the ordered 2d points of the KeyFrame polygon boundary.
    ///
    const Eigen::MatrixXd& points_2d() const {
      return points2d;
    }

    /// Get the ordered 3d points of the keyframe polygon boundary.
    /// These points are just the 2d points projected onto the KeyFrame plane.
    ///
    const Eigen::MatrixXd& points_3d() {
      // TODO: Recomputing this every frame might be a bit slow but we'll optimize it if we need to
      points3d.conservativeResize(points2d.rows(), 3);
      for (int i = 0; i < points2d.rows(); i++) {
        points3d.row(i) = plane_center + points2d(i, 0) *coord_system.row(0) + points2d(i, 1) *coord_system.row(1);
      }
      return points3d;
    }

    /// Get the index value of this KeyFrame.
    ///
    const double index() const {
      return curve_index;
    }

    /// Move the i^th point on the polygon boundary to the 2d position newpos. To get the
    /// ordered polygon points, call `points_2d()`.
    ///
    /// If the movement causes the bounding cage to self-intersect, no state gets changed,
    /// and this method returns false.
    ///
    bool move_point_2d(int i, Eigen::RowVector2d& newpos);
  };

  /// Bidirectional Iterator class used to traverse the linked list of KeyFrames
  /// in index order.
  ///
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

  /// Linked list of KeyFrames ordered by index.
  /// This list is built upon the Cell leaf-list described above.
  ///
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

  /// Set the skeleton vertices to whatever the user provides.
  /// There must be at least two vertices, if not the method returns false.
  /// Upon setting the vertices, The
  ///
  bool set_skeleton_vertices(const Eigen::MatrixXd& new_SV,
                             unsigned smoothing_iters,
                             const Eigen::MatrixXd& polygon_template);

  /// Clear the bounding cage and skeleton vertices
  ///
  void clear() {
    root.reset();
    cells.head.reset();
    cells.tail.reset();
    SV.resize(0, 0);
    SV_smooth.resize(0, 0);
  }

  /// Add a new KeyFrame at the given index in the bounding Cage.
  /// The new KeyFrame will have a shape which linearly interpolates the
  /// base KeyFrames of the Cell containing its index.
  ///
  KeyFrameIterator split(double index);

  /// Get the skeleton vertex positions
  ///
  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

  /// Get the minimum keyframe index
  ///
  double min_index() const {
    if(!root) {
      return 0.0;
    }
    assert(root->min_index() == cells.head->min_index());
    return root->min_index();
  }

  /// Get the maximum keyframe index
  ///
  double max_index() const {
    if(!root) {
      return 0.0;
    }
    assert(root->max_index() == cells.tail->max_index());
    return root->max_index();
  }

  /// Get the 2d or 3d vertices for the plane cutting the cage at index.
  /// These methods return an empty matrix if the index is out of range
  ///
  Eigen::MatrixXd vertices_3d_for_index(double index) const;
  Eigen::MatrixXd vertices_2d_for_index(double index) const;

  /// Get the plane cutting the cage at the given index
  ///
  std::pair<Eigen::VectorXd, Eigen::VectorXd> plane_for_index(double index) const;
};



#endif // BOUNDING_CAGE_H
