#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

#include <spdlog/spdlog.h>


#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H

extern const char* FISH_LOGGER_NAME;

class BoundingCage {
public:
  class KeyFrame;
  class Cell;

private:

  /// Return true if a Cell contains the skeleton vertices corresponding
  /// to its index range, and false otherwise.
  ///
  bool skeleton_in_cell(std::shared_ptr<Cell> node) const;

  /// Core method to split the bounding cage using the keyframe.
  ///
  std::shared_ptr<KeyFrame> split_internal(std::shared_ptr<KeyFrame> kf);

  /// Skeleton Vertices
  ///
  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;

  /// Root node of the Cell tree
  ///
  std::shared_ptr<Cell> root;

  /// Logger for this class
  /// By default this is the null logger
  std::shared_ptr<spdlog::logger> logger;

  /// Mesh for the whole bounding cage
  ///
  Eigen::MatrixXd CV;
  Eigen::VectorXi CV_refcount;
  std::vector<int> CV_free_list;
  int num_mesh_vertices = 0;

  bool update_vertex(int i, const Eigen::RowVector3d& v);
  bool add_vertices(const Eigen::MatrixXd& V, Eigen::VectorXi& VI);
  bool replace_vertices(const Eigen::MatrixXd& V, Eigen::VectorXi& VI);

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
  class Cell : public std::enable_shared_from_this<Cell> {
    friend class BoundingCage;

    /// Reference to the owning BoundingCage
    const BoundingCage* cage;

    /// A Cell is a binary tree and can contain sub-Cells
    ///
    std::shared_ptr<Cell> left_child;
    std::shared_ptr<Cell> right_child;

    /// Parent cell in the tree
    ///
    std::weak_ptr<Cell> parent_cell;


    /// The "left" and "right" KeyFrames. The left has a smaller index than the right.
    ///
    std::shared_ptr<KeyFrame> left_keyframe;
    std::shared_ptr<KeyFrame> right_keyframe;

    /// Indices of the boundary triangles in the
    /// mesh of the BoundingCage which owns this Cell
    ///
    Eigen::VectorXi mesh_face_indexes;
    Eigen::MatrixXi _mesh_faces;

    /// Split the Cell into two cells divided by key_frame.
    /// If the index of key_frame is outside the cell, this method
    /// returns false and the Cell remains unchanged.
    ///
    std::shared_ptr<KeyFrame> split(std::shared_ptr<KeyFrame> key_frame);

    /// Merge all the children of this cell into one cell
    ///
    bool merge();

    /// Find the cell for the given index
    ///
    std::shared_ptr<Cell> find(double index);

    /// Returns true if the cell is a leaf node
    ///
    bool is_leaf() const { return !left_child && !right_child; }

    /// Logger for this class
    ///
    std::shared_ptr<spdlog::logger> logger;

    /// This method initializes the BoundingCage mesh for this Cell, allocating new
    /// storage for the face information
    ///
    bool init_mesh();

    /// Update the mesh for this cell when the cell changes
    ///
    bool update_mesh();

    /// Construct a new Cell. Don't call this directly, instead use the factory
    /// function `make_cell()`.
    ///
    Cell(std::shared_ptr<KeyFrame> left_kf,
         std::shared_ptr<KeyFrame> right_kf,
         std::weak_ptr<Cell> parent,
         const BoundingCage* cage) :
      cage(cage), left_keyframe(left_kf), right_keyframe(right_kf), parent_cell(parent),
      logger(spdlog::get(FISH_LOGGER_NAME)) {}

    /// Construct a new Cell wrapped in a shared_ptr. Internally, this method is used
    /// to create Cells in lieu of the constructor.
    static std::shared_ptr<Cell> make_cell(std::shared_ptr<BoundingCage::KeyFrame> left_kf,
                                           std::shared_ptr<BoundingCage::KeyFrame> right_kf,
                                           const BoundingCage* cage,
                                           std::weak_ptr<Cell> parent_cell=std::shared_ptr<Cell>());

  public:
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
      if (cell && cell->right_keyframe) {
        cell = cell->right_keyframe->_right_cell.lock();
      }
      return *this;
    }

    CellIterator operator++(int) {
      return operator++();
    }

    CellIterator operator--() {
      if (cell && cell->left_keyframe) {
        cell = cell->left_keyframe->_left_cell.lock();
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

    /// Parallel transport constructor:
    /// The local coordinate frame in this KeyFrame is determined
    /// by transporting the frame from from_kf
    ///
    KeyFrame(const Eigen::RowVector3d& normal,
             const Eigen::RowVector3d& center,
             const KeyFrame& from_kf,
             const Eigen::MatrixXd& pts,
             std::shared_ptr<Cell> cell,
             double idx,
             const BoundingCage* _cage);

    /// Explicit constructor:
    /// The local coordinate frame for this KeyFrame is provided explicitly
    ///
    KeyFrame(const Eigen::RowVector3d& center,
             const Eigen::Matrix3d& coord_frame,
             const Eigen::MatrixXd& pts,
             std::shared_ptr<Cell> cell,
             double idx,
             const BoundingCage *_cage);

    /// When polygon vertex changes (via `set_point_2d()`), these methods
    /// validate that the change does not create *LOCAL* self intersections.
    ///
    bool validate_points_2d();
    bool validate_cage();

    /// Called internally to initialize the BoundingCage mesh with this KeyFrame's data
    /// If tesselate is true, the polygon for this KeyFrame will be triangulated and 
    /// included in the BoundingCage mesh. We use this for the caps of the cage mesh.
    ///
    bool init_mesh(bool tesellate=false, bool flip=false);

    /// Update the BoundingCage mesh when the KeyFrame changes. This is a no-op if
    /// the KeyFrame is not triangulated.
    ///
    bool update_mesh();

    /// Split the KeyFrame polygon along an edge starting at vertex i
    /// t in [0, 1] specifies where to split. i.e. v[i] + t*(v[i+1]-v[i])
    ///
    bool insert_vertex(unsigned i, double t);

    /// The BoundingCage which owns this KeyFrame
    ///
    const BoundingCage* _cage;

    /// State representing the plane for this KeyFrame.
    ///
    Eigen::Matrix3d _orientation;
    Eigen::RowVector3d _center;

    /// 2D positions of the boundary polygon of this KeyFrame
    ///
    Eigen::MatrixXd _vertices_2d;

    /// Indices of the 3D positions of the boundary polygon in the
    /// mesh of the BoundingCage which owns this KeyFrame
    ///
    Eigen::VectorXi _mesh_boundary_indices;
    Eigen::VectorXi _mesh_interior_indices;
    Eigen::MatrixXi _mesh_faces;

    /// The index of this KeyFrame.
    double _index;

    /// Pointers to the Cells bounidng this keyframe
    ///
    std::weak_ptr<Cell> _left_cell;
    std::weak_ptr<Cell> _right_cell;
    std::shared_ptr<Cell> left_cell() { return _left_cell.lock(); }
    std::shared_ptr<Cell> right_cell() { return _right_cell.lock(); }
    /// Logger for this class
    ///
    std::shared_ptr<spdlog::logger> logger;

    /// Set to true if this KeyFrame is triangulated
    ///
    bool _is_triangulated = false;
    /// Set to true if we need to flip the triangulation of the KeyFrame
    /// This is used to properly orient triangles on the front of the cage
    ///
    bool _is_triangulation_flipped = false;

  public:
    /// Returns true if this KeyFrame is part of the bounding cage
    ///
    bool in_bounding_cage() {
      return _mesh_boundary_indices.rows() != 0;
    }

    /// True if this KeyFrame is at one of the endpoints of its BoundingCage
    ///
    bool is_endpoint() {
      return _is_triangulated;
    }

    /// Get the normal of the plane of this KeyFrame.
    ///
    Eigen::RowVector3d normal() const {
      return _orientation.row(2);
    }

    /// Get the up basis vector of the coordinate system of this KeyFrame.
    ///
    Eigen::RowVector3d up() const {
      return _orientation.row(1);
    }

    /// Get the right basis vector of the coordinate system of this KeyFrame.
    ///
    Eigen::RowVector3d right() const {
      return _orientation.row(0);
    }

    /// Get the local coordinate system of this KeyFrame.
    /// 2d positions, (x, y), of this keyframe represent coefficients
    /// along the first and second rows of this system.
    ///
    const Eigen::Matrix3d& orientation() const {
      return _orientation;
    }

    /// Get the center of the KeyFrame.
    ///
    const Eigen::RowVector3d& center() const {
      return _center;
    }

    /// An affine transformation to map points to the local coordinates
    /// this KeyFrame
    ///
    Eigen::Matrix4d transform() const {
      Eigen::MatrixX4d ret;
      ret.setZero();
      ret.block<3, 3>(0, 0) = _orientation;
      ret.block<3, 1>(0, 3) = _center.transpose();
      ret(3, 3) = 1.0;
      return ret;
    }

    /// Get the ordered 2d points of the KeyFrame polygon boundary.
    ///
    const Eigen::MatrixXd& vertices_2d() const {
      return _vertices_2d;
    }

    /// Get the ordered 3d points of the keyframe polygon boundary.
    /// These points are just the 2d points projected onto the KeyFrame plane.
    ///
    Eigen::MatrixXd vertices_3d() const {
      Eigen::MatrixXd points3d(_vertices_2d.rows(), 3);
      for (int i = 0; i < _vertices_2d.rows(); i++) {
        points3d.row(i) = _center + _vertices_2d(i, 0) *_orientation.row(0) + _vertices_2d(i, 1) *_orientation.row(1);
      }
      return points3d;
    }

    /// Get the index value of this KeyFrame.
    ///
    const double index() const {
      return _index;
    }

    /// Move the i^th point on the polygon boundary to the 2d position newpos. To get the
    /// ordered polygon points, call `points_2d()`.
    ///
    /// If validate is set and the movement causes the bounding cage to self-intersect,
    /// no state gets changed, and this method returns false.
    ///
    /// Otherwise, if validate is unset, this method always returns true
    ///
    bool move_point_2d(int i, Eigen::RowVector2d& newpos, bool validate2d=true, bool validate_3d=false);
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
      if (!keyframe) {
        return *this;
      }

      std::shared_ptr<Cell> right_cell = keyframe->_right_cell.lock();
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
      if (!keyframe) {
        return *this;
      }

      std::shared_ptr<Cell> left_cell = keyframe->_left_cell.lock();
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
        return KeyFrameIterator(cage->cells.tail->right_keyframe);
      } else {
        return KeyFrameIterator();
      }
    }

    KeyFrameIterator rend() {
      return KeyFrameIterator();
    }

  } keyframes;

  BoundingCage() {
    keyframes.cage = this;
  }

  friend class KeyFrame;
  friend class Cell;

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
    CV.resize(0, 0);
    num_mesh_vertices = 0;
  }

  /// Add a new KeyFrame at the given index in the bounding Cage.
  /// The new KeyFrame will have a shape which linearly interpolates the
  /// base KeyFrames of the Cell containing its index.
  ///
  KeyFrameIterator insert_keyframe(double index);
  KeyFrameIterator insert_keyframe(KeyFrameIterator& it);

  /// Delete a KeyFrame in the BoundingCage.
  /// If the KeyFrame is not inserted, this method returns false
  ///
  bool remove_keyframe(KeyFrameIterator& it);

  /// Get the skeleton vertex positions
  ///
  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

  /// Get the mesh bounding this cage
  ///
  Eigen::MatrixXd vertices() const { return CV.block(0, 0, num_mesh_vertices, 3); }
  Eigen::MatrixXi faces() const {
    std::shared_ptr<KeyFrame> head_kf = cells.head->left_keyframe;
    std::shared_ptr<KeyFrame> tail_kf = cells.tail->right_keyframe;

    int num_faces = head_kf->_mesh_faces.rows() + tail_kf->_mesh_faces.rows();
    for (auto cell : cells) {
      num_faces += cell._mesh_faces.rows();
    }
    Eigen::MatrixXi CF(num_faces, 3);
    int start_idx = 0;
    CF.block(start_idx, 0, head_kf->_mesh_faces.rows(), 3) = head_kf->_mesh_faces;
    start_idx += head_kf->_mesh_faces.rows();
    for (auto cell : cells) {
      CF.block(start_idx, 0, cell._mesh_faces.rows(), 3) = cell._mesh_faces;
      start_idx += cell._mesh_faces.rows();
    }
    CF.block(start_idx, 0, tail_kf->_mesh_faces.rows(), 3) = tail_kf->_mesh_faces;

    return CF;
  }

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

  bool add_boundary_vertex(unsigned i, double t);

  /// Get a KeyFrame at the specified index.
  /// The KeyFrame may not yet be inserted into the bounding cage.
  /// To insert it, call split()
  ///
  KeyFrameIterator keyframe_for_index(double index) const;
};



#endif // BOUNDING_CAGE_H
