#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

#include <spdlog/spdlog.h>


class BoundingCage {
public:
    class KeyFrame;
    class Cell;
    class KeyFrameIterator;
    class CellIterator;

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

    /// Number of keyframes in the bounding cage
    int _num_keyframes = 0;

    Eigen::Vector4d _keyframe_bounding_box;

public:

    Eigen::Vector4d keyframe_bounding_box() const {
        return _keyframe_bounding_box;
    }

    bool set_keyframe_bounding_box(const Eigen::Vector4d& bbox) {
        if (bbox[0] >= bbox[1] || bbox[2] >= bbox[3]) {
            logger->error("Invalid intial bounding box. The input is (min_u, max_u, min_v, max_v) = ({}, {}, {}, {})."
                          "We require that min_u < max_u and min_v < max_v", bbox[0], bbox[1], bbox[2], bbox[3]);
            return false;
        }

        this->_keyframe_bounding_box = bbox;
        return true;
    }

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
        const BoundingCage* _cage;

        /// A Cell is a binary tree and can contain sub-Cells
        ///
        std::shared_ptr<Cell> _left_child;
        std::shared_ptr<Cell> _right_child;

        /// Parent cell in the tree
        ///
        std::weak_ptr<Cell> _parent_cell;


        /// The "left" and "right" KeyFrames. The left has a smaller index than the right.
        ///
        std::shared_ptr<KeyFrame> _left_keyframe;
        std::shared_ptr<KeyFrame> _right_keyframe;

        /// Split the Cell into two cells divided by key_frame.
        /// If the index of key_frame is outside the cell, this method
        /// returns false and the Cell remains unchanged.
        ///
        std::shared_ptr<KeyFrame> split(std::shared_ptr<KeyFrame> key_frame);
        /// Find the cell for the given index
        ///
        std::shared_ptr<Cell> find(double index);

        /// Returns true if the cell is a leaf node
        ///
        bool is_leaf() const { return !_left_child && !_right_child; }

        /// Logger for this class
        ///
        std::shared_ptr<spdlog::logger> logger;

        /// Construct a new Cell. Don't call this directly, instead use the factory
        /// function `make_cell()`.
        ///
        Cell(std::shared_ptr<KeyFrame> left_kf, std::shared_ptr<KeyFrame> right_kf,
             std::weak_ptr<Cell> parent, const BoundingCage* cage);

        /// Construct a new Cell wrapped in a shared_ptr. Internally, this method is used
        /// to create Cells in lieu of the constructor.
        static std::shared_ptr<Cell> make_cell(std::shared_ptr<BoundingCage::KeyFrame> left_kf,
                                               std::shared_ptr<BoundingCage::KeyFrame> right_kf,
                                               const BoundingCage* _cage,
                                               std::weak_ptr<Cell> _parent_cell=std::shared_ptr<Cell>());

    public:
        const Eigen::MatrixXi mesh_faces() const;
        const Eigen::MatrixXd mesh_vertices() const;

        const KeyFrameIterator left_keyframe() const { return KeyFrameIterator(_left_keyframe); }
        const KeyFrameIterator right_keyframe() const { return KeyFrameIterator(_right_keyframe); }
        double min_index() const { return _left_keyframe->index(); }
        double max_index() const { return _right_keyframe->index(); }
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
            if (cell && cell->_right_keyframe) {
                cell = cell->_right_keyframe->_right_cell.lock();
            }
            return *this;
        }

        CellIterator operator++(int) {
            return operator++();
        }

        CellIterator operator--() {
            if (cell && cell->_left_keyframe) {
                cell = cell->_left_keyframe->_left_cell.lock();
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
                 const BoundingCage::KeyFrame& from_kf, const double angle,
                 const Eigen::MatrixXd& pts,
                 const Eigen::RowVector2d& centroid,
                 std::shared_ptr<BoundingCage::Cell> cell,
                 double idx,
                 BoundingCage* cage);

        /// Explicit constructor:
        /// The local coordinate frame for this KeyFrame is provided explicitly
        ///
        KeyFrame(const Eigen::RowVector3d& origin,
                 const Eigen::Matrix3d& coord_frame, const double angle,
                 const Eigen::MatrixXd& pts,
                 const Eigen::RowVector2d& centroid,
                 std::shared_ptr<Cell> cell,
                 double idx,
                 BoundingCage *_cage);


        /// The BoundingCage which owns this KeyFrame
        ///
        BoundingCage* _cage;

        /// State representing the plane for this KeyFrame.
        ///
        Eigen::Matrix3d _orientation;
        Eigen::RowVector3d _origin;


        /// 2D positions of the boundary polygon of this KeyFrame
        ///
        Eigen::MatrixXd _vertices_2d;

        /// Centroid position in 2d coordinates
        Eigen::RowVector2d _centroid_2d;

        /// True if this keyframe is part of the bounding cage
        bool _in_cage = false;

        /// The index of this KeyFrame.
        double _index;

        /// Store a torsion angle from -pi/2 to pi/2 radians which we use to interpolate
        double _angle = 0.0;

        /// Pointers to the Cells bounidng this keyframe
        ///
        std::weak_ptr<Cell> _left_cell;
        std::weak_ptr<Cell> _right_cell;
        std::shared_ptr<Cell> left_cell() const { return _left_cell.lock(); }
        std::shared_ptr<Cell> right_cell() const { return _right_cell.lock(); }

        /// Logger for this class
        ///
        std::shared_ptr<spdlog::logger> logger;

    public:

        /// Returns true if this KeyFrame is part of the bounding cage
        ///
        bool in_bounding_cage() const {
            return _in_cage;
        }

        /// True if this KeyFrame is at one of the endpoints of its BoundingCage
        ///
        bool is_endpoint() const {
            return in_bounding_cage() && (!left_cell() || !right_cell());
        }

        /// Get the normal of the plane of this KeyFrame.
        /// This is independent of the torsion angle since torsion is applied
        /// in the plane defined by this normal
        ///
        Eigen::RowVector3d normal() const {
            return _orientation.row(2);
        }

        /// Get the right of the coordinate system with torsion rotation applied.
        ///
        Eigen::RowVector3d right_rotated_3d() const {
            return orientation_rotated().row(0);
        }

        /// Get the up vector of the coordinate system with torsion rotation applied.
        ///
        Eigen::RowVector3d up_rotated_3d() const {
            return orientation_rotated().row(1);
        }

        /// Get the right vector of the coordinate system of this KeyFrame without
        /// the torsion rotation applied.
        ///
        Eigen::RowVector3d right_3d() const {
            return orientation().row(0);
        }

        /// Get the up vector of the coordinate system of this KeyFrame without
        /// the torsion rotation applied.
        ///
        Eigen::RowVector3d up_3d() const {
            return orientation().row(1);
        }

        /// Get the right vector of the KeyFrame in 2D after applying the torsion rotation
        ///
        Eigen::RowVector2d right_rotated_2d() const {
            return Eigen::Rotation2Dd(_angle) * Eigen::Vector2d(1.0, -.0);
        }

        /// Get the up vector of the KeyFrame in 2D after applying the torsion rotation
        ///
        Eigen::RowVector2d up_rotated_2d() const {
            return Eigen::Rotation2Dd(_angle) * Eigen::Vector2d(0.0, 1.0);
        }

        /// Return the torsion angle applied to this keyframe
        ///
        double angle() const {
            return _angle;
        }

        /// Get the local coordinate system of this KeyFrame.
        /// 2d positions, (x, y), of this keyframe represent coefficients
        /// along the first and second rows of this system.
        ///
        const Eigen::Matrix3d orientation_rotated() const {
            Eigen::AngleAxisd R(-_angle, _orientation.row(2));
            return (R *_orientation.transpose()).transpose();
        }

        /// Get the coordinate frame of this keyframe without the torsion
        /// rotation applies. Each row of the returned matrix is a basis vector
        /// for coordinate system. The first row is the "right" direction, the
        /// second row is the "up" direction, and the third is the "normal" direction.
        ///
        const Eigen::Matrix3d orientation() const {
            return _orientation;
        }

        /// Get the center of the KeyFrame in 3D
        ///
        const Eigen::RowVector3d& origin() const {
            return _origin;
        }

        /// Get the ordered 2d points of the KeyFrame polygon boundary.
        /// TODO: Kill this!
        const Eigen::MatrixXd& vertices_2d() const {
            return _vertices_2d;
        }

        /// Get the 2d position of the centroid of this KeyFrame
        ///
        const Eigen::RowVector2d centroid_2d() const {
            return _centroid_2d;
        }

        /// Get the 2d position of the centroid of this KeyFrame
        ///
        const Eigen::RowVector3d centroid_3d() const {
            return _origin + _orientation.row(0)*_centroid_2d[0] + _orientation.row(1)*_centroid_2d[1];
        }

        /// Get the 3d positions of the bounding box for this keyframe without
        /// applying the torsion rotation
        ///
        const Eigen::MatrixXd bounding_box_vertices_3d() const {
            Eigen::MatrixXd ret(4, 3);
            Eigen::RowVector4d bbox = _cage->keyframe_bounding_box();
            double min_u = bbox[0], max_u = bbox[1], min_v = bbox[2], max_v = bbox[3];
            ret.row(0) = right_rotated_3d()*min_u + up_rotated_3d()*min_v;
            ret.row(1) = right_rotated_3d()*max_u + up_rotated_3d()*min_v;
            ret.row(2) = right_rotated_3d()*max_u + up_rotated_3d()*max_v;
            ret.row(3) = right_rotated_3d()*min_u + up_rotated_3d()*max_v;

            ret.rowwise() += centroid_3d();

            return ret;
        }

        /// Get the 2d positions of the bounding box for this keyframe
        /// without applying the torsion rotation
        /// TODO: Maybe kill this
        const Eigen::MatrixXd bounding_box_vertices_2d() const {
            Eigen::MatrixXd ret(4, 2);
            const Eigen::RowVector4d bbox = _cage->keyframe_bounding_box();
            const double min_u = bbox[0], max_u = bbox[1], min_v = bbox[2], max_v = bbox[3];
            ret << min_u, min_v,
                   max_u, min_v,
                   max_u, max_v,
                   min_u, max_v;

            ret.rowwise() += centroid_2d();
            return ret;
        }

        /// Get the 2d positions of the bounding box for this keyframe rotated
        /// by the torsion angle
        ///
        const Eigen::MatrixXd bounding_box_vertices_rotated_2d() const {
            Eigen::MatrixXd ret(4, 2);
            const Eigen::RowVector2d r = right_rotated_2d();
            const Eigen::RowVector2d u = up_rotated_2d();

            Eigen::RowVector4d bbox = _cage->keyframe_bounding_box();
            double min_u = bbox[0], max_u = bbox[1], min_v = bbox[2], max_v = bbox[3];
            ret.row(0) = r*min_u + u*min_v + centroid_2d();
            ret.row(1) = r*max_u + u*min_v + centroid_2d();
            ret.row(2) = r*max_u + u*max_v + centroid_2d();
            ret.row(3) = r*min_u + u*max_v + centroid_2d();

            return ret;
        }

        /// Return the position of the vector v rotated by the torsion angle
        ///
        const Eigen::RowVector2d rotated_coords(const Eigen::RowVector2d& v) const {
            return Eigen::RowVector2d(right_rotated_2d().dot(v), up_rotated_2d().dot(v));
        }

        /// Get the index value of this KeyFrame.
        ///
        const double index() const {
            return _index;
        }

        /// Move the center point of the KeyFrame in its xy plane. If relative is set, then the
        /// polygon points preserve their relative vectors with respect to the center
        ///
        bool move_centroid_2d(const Eigen::RowVector2d& new_center);

        /// Rotate the coordinate frame counter-clockwise about the normal axis
        ///
        bool rotate_torsion_frame(double d_angle);

        /// Set the rotation of the coordinate frame
        ///
        bool set_angle(double angle);
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
        KeyFrameIterator& operator=(const KeyFrameIterator& other) { if (&other != this) { keyframe = other.keyframe; } return *this; }

        KeyFrameIterator operator++() {
            if (!keyframe) {
                return *this;
            }

            std::shared_ptr<Cell> right_cell = keyframe->_right_cell.lock();
            if (keyframe && right_cell) {
                keyframe = right_cell->_right_keyframe;
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
                keyframe = left_cell->_left_keyframe;
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

        std::shared_ptr<KeyFrame> operator->() const {
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
                return KeyFrameIterator(cage->cells.head->_left_keyframe);
            } else {
                return KeyFrameIterator();
            }
        }

        KeyFrameIterator end() {
            return KeyFrameIterator();
        }

        KeyFrameIterator rbegin() {
            if(cage->cells.tail) {
                return KeyFrameIterator(cage->cells.tail->_right_keyframe);
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

    const int num_keyframes() const {
        return _num_keyframes;
    }

    const int num_cells() const {
        return _num_keyframes - 1;
    }

    /// Set the skeleton vertices to whatever the user provides.
    /// There must be at least two vertices, if not the method returns false.
    /// Upon setting the vertices, The
    ///
    bool set_skeleton_vertices(const Eigen::MatrixXd& new_SV,
                               unsigned smoothing_iters,
                               const Eigen::Vector4d& bounding_box);

    /// Clear the bounding cage and skeleton vertices
    ///
    void clear() {
        _num_keyframes = 0;
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
    KeyFrameIterator insert_keyframe(double index);
    KeyFrameIterator insert_keyframe(KeyFrameIterator& it);

    /// Delete a KeyFrame in the BoundingCage.
    /// If the KeyFrame is not inserted, this method returns false
    ///
    bool delete_keyframe(KeyFrameIterator& it);

    /// Get the skeleton vertex positions
    ///
    const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
    const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

    /// Get a buffer of vertices for the mesh of the BoundingCage
    /// To get the faces, iterate over the cells and call mesh_faces for each cell.
    /// Each cell's face buffer indexes into mesh_vertices()
    ///
    const Eigen::MatrixXd mesh_vertices();
    const Eigen::MatrixXi mesh_faces() const;

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

    /// Get a KeyFrame at the specified index.
    /// The KeyFrame may not yet be inserted into the bounding cage.
    /// To insert it, call split()
    ///
    KeyFrameIterator keyframe_for_index(double index) const;
};



#endif // BOUNDING_CAGE_H
