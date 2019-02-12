#include "bounding_cage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>
#include <igl/triangle/triangulate.h>
#include <igl/segment_segment_intersect.h>

// Parallel transport a coordinate system from a KeyFrame along a curve to a point with normal, to_n
//static Eigen::Matrix3d parallel_transport(const BoundingCage::KeyFrame& from_kf, Eigen::RowVector3d to_n) {
//    Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(from_kf.normal().normalized(), to_n.normalized()).matrix();
//    Eigen::Matrix3d coord_system = (R*from_kf.orientation().transpose()).transpose();
//    for (int i = 0; i < coord_system.rows(); i++) { coord_system.row(i) /= coord_system.row(i).norm(); }
//    return coord_system;
//}


static Eigen::Matrix3d parallel_transport(const Eigen::MatrixXd& kf_coord_frame, Eigen::RowVector3d to_n) {
    Eigen::RowVector3d normal = kf_coord_frame.row(2);
    Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(normal.normalized(), to_n.normalized()).matrix();
    Eigen::Matrix3d coord_system = (R*kf_coord_frame.transpose()).transpose();
    for (int i = 0; i < coord_system.rows(); i++) { coord_system.row(i) /= coord_system.row(i).norm(); }
    return coord_system;
}

// Construct a 3x3 rotation matrix whose 3rd row is normal
static Eigen::Matrix3d local_coordinate_system(const Eigen::RowVector3d& normal) {
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

    ret.row(0).normalize();
    ret.row(1).normalize();
    ret.row(2).normalize();

    return ret;
}





// |--------------------------------| //
// |                                | //
// | BoundingCage::KeyFrame methods | //
// |                                | //
// |================================| //

BoundingCage::KeyFrame::KeyFrame(const Eigen::RowVector3d& normal,
                                 const Eigen::RowVector3d& center,
                                 const BoundingCage::KeyFrame& from_kf,
                                 const double angle,
                                 const Eigen::MatrixXd& pts,
                                 const Eigen::RowVector2d& centroid,
                                 std::shared_ptr<BoundingCage::Cell> cell,
                                 double idx,
                                 BoundingCage* cage) {
    _cage = cage;
    _index = idx;
    _orientation = parallel_transport(from_kf._orientation, normal);
    _angle = angle;
    _centroid_2d = centroid;
    _origin = center;
    _vertices_2d = pts;

    _left_cell = cell;
    _right_cell = cell;

    logger = _cage->logger;
}

BoundingCage::KeyFrame::KeyFrame(const Eigen::RowVector3d& center,
                                 const Eigen::Matrix3d& coord_frame,
                                 const double angle,
                                 const Eigen::MatrixXd& pts,
                                 const Eigen::RowVector2d& centroid,
                                 std::shared_ptr<BoundingCage::Cell> cell,
                                 double idx,
                                 BoundingCage *cage) {
    _cage = cage;
    _index = idx;
    _angle = angle;
    _orientation = coord_frame;
    _origin = center;
    _vertices_2d = pts;
    _centroid_2d = centroid;
    _left_cell = cell;
    _right_cell = cell;

    logger = _cage->logger;
}


bool BoundingCage::KeyFrame::move_centroid_2d(const Eigen::RowVector2d& new_centroid_2d) {
    if (!in_bounding_cage()) {
        logger->warn("Cannot move KeyFrame centroid if KeyFrame is not in bounding cage");
        return false;
    }

    _centroid_2d = new_centroid_2d;
    return true;
}

bool BoundingCage::KeyFrame::rotate_torsion_frame(double d_angle) {
    if (!in_bounding_cage()) {
        logger->warn("Cannot move KeyFrame centroid if KeyFrame is not in bounding cage");
        return false;
    }

    _angle += d_angle;
}

bool BoundingCage::KeyFrame::set_angle(double angle) {
    if (!in_bounding_cage()) {
        logger->warn("Cannot move KeyFrame centroid if KeyFrame is not in bounding cage");
        return false;
    }

    _angle = angle;
}



// |----------------------------| //
// |                            | //
// | BoundingCage::Cell methods | //
// |                            | //
// |============================| //

BoundingCage::Cell::Cell(std::shared_ptr<KeyFrame> left_kf,
                         std::shared_ptr<KeyFrame> right_kf, std::weak_ptr<Cell> parent,
                         const BoundingCage* cage)
    : _cage(cage)
    , _left_keyframe(left_kf)
    , _right_keyframe(right_kf)
    , _parent_cell(parent)
    , logger(cage->logger) {}

std::shared_ptr<BoundingCage::Cell> BoundingCage::Cell::make_cell(std::shared_ptr<KeyFrame> left_kf,
                                                                  std::shared_ptr<KeyFrame> right_kf,
                                                                  const BoundingCage* cage,
                                                                  std::weak_ptr<Cell> parent) {
    std::shared_ptr<Cell> ret(new Cell(left_kf, right_kf, parent, cage));

    if (!left_kf || !right_kf) {
        std::shared_ptr<spdlog::logger> logger = ret->logger;
        logger->warn("Cell::make_cell failed to construct Cell. "
                     "One or both of left_kf or right_kf was null.");
        ret.reset();
        return ret;
    }

    left_kf->_right_cell = ret;
    right_kf->_left_cell = ret;

    // TODO: AABB tree Self intersection test

    return ret;
}

const Eigen::MatrixXi BoundingCage::Cell::mesh_faces() const {
    Eigen::MatrixXi F(12, 3);

    F << 3, 0, 1,
         3, 1, 2,
         5, 4, 7,
         5, 7, 6,
         0, 3, 7,
         0, 7, 4,
         3, 6, 7,
         6, 3, 2,
         2, 5, 6,
         5, 2, 1,
         1, 0, 5,
         5, 0, 4;

    return F;
}

const Eigen::MatrixXd BoundingCage::Cell::mesh_vertices() const {

    Eigen::MatrixXd lV = _left_keyframe->bounding_box_vertices_3d();
    Eigen::MatrixXd rV = _right_keyframe->bounding_box_vertices_3d();

    Eigen::MatrixXd V(8, 3);

    V << lV(0, 0), lV(0, 1), lV(0, 2),
         lV(1, 0), lV(1, 1), lV(1, 2),
         lV(2, 0), lV(2, 1), lV(2, 2),
         lV(3, 0), lV(3, 1), lV(3, 2),
         rV(0, 0), rV(0, 1), rV(0, 2),
         rV(1, 0), rV(1, 1), rV(1, 2),
         rV(2, 0), rV(2, 1), rV(2, 2),
         rV(3, 0), rV(3, 1), rV(3, 2);

    return V;
}

std::shared_ptr<BoundingCage::KeyFrame> BoundingCage::Cell::split(std::shared_ptr<KeyFrame> keyframe) {
    // The index of the keyframe is out of range, since this method is called, internally,
    // this should fail
    if (keyframe->index() > max_index() || keyframe->index() < min_index()) {
        assert("split index is out of range" && false);
        logger->error("index of keyframe ({}) in split(), was out of range ({}, {})",
                      keyframe->index(), min_index(), max_index());
        return std::shared_ptr<KeyFrame>();
    }

    // This node has already been split by the keyframe
    if (keyframe->index() == max_index()) {
        return _right_keyframe;
    }
    if (keyframe->index() == min_index()) {
        return _left_keyframe;
    }

    // This node is a leaf node, so split it
    if (is_leaf()) {
        // Initialize new Cells for the left and right children
        _left_child = Cell::make_cell(_left_keyframe, keyframe, _cage, shared_from_this() /* parent */);
        if (!_left_child) {
            logger->warn("Cell::split() failed to construct left child.");
            return std::shared_ptr<KeyFrame>();
        }
        _right_child = Cell::make_cell(keyframe, _right_keyframe, _cage, shared_from_this() /* parent */);
        if (!_right_child) {
            logger->warn("Cell::split() failed to construct right child.");
            _left_child.reset();
            return std::shared_ptr<KeyFrame>();
        }


        // Fix the cell pointers in the new keyframe
        _left_keyframe->_right_cell = _left_child;
        _right_keyframe->_left_cell = _right_child;
        keyframe->_left_cell = _left_child;
        keyframe->_right_cell = _right_child;

        // If we added a cell with a different normal than the left key-frame, we need to recompute the
        // the local coordinate frame of the right keyframe
        _right_keyframe->_orientation = parallel_transport(_right_child->_left_keyframe->_orientation, _right_keyframe->_orientation.row(2));

        return keyframe;

        // This node is not a leaf node, split one of the children
    } else if(keyframe->index() > _left_child->min_index() && keyframe->index() < _left_child->max_index()) {
        return _left_child->split(keyframe);
    } else if(keyframe->index() > _right_child->min_index() && keyframe->index() < _right_child->max_index()) {
        return _right_child->split(keyframe);
    } else {
        logger->error("BoundingCage tree is in an incorrect state. split() failed."
                      "Cell split index = {}, left_keyframe_index = {}, "
                      "right_keyframe_index = {}, "
                      "left_child = {0:x}, right_child = {0:x}.",
                      keyframe->index(), _left_keyframe->index(), _right_keyframe->index(),
                      (std::ptrdiff_t)_left_child.get(), (std::ptrdiff_t)_right_child.get());
        assert("BoundingCage tree is in a bad state" && false);
        assert("BoundingCage tree is in a bad state" && ((_left_child && !_right_child) || (_right_child && !_left_child)));
        return std::shared_ptr<KeyFrame>();;
    }
    assert(false);
}

std::shared_ptr<BoundingCage::Cell> BoundingCage::Cell::find(double index) {
    if (index < min_index() || index > max_index()) {
        logger->error("find_cell_r(cell, index) index, {} was out of range ({}, {})",
                      index, min_index(), max_index());
        return std::shared_ptr<Cell>();
    } else if (is_leaf()) {
        return shared_from_this();
    } else if (_left_child && index >= _left_child->min_index() && index <= _left_child->max_index()) {
        return _left_child->find(index);
    } else if (_right_child && index >= _right_child->min_index() && index <= _right_child->max_index()) {
        return _right_child->find(index);
    } else {
        logger->error("BUG!!! This branch in find_cell_r should never execute.");
        assert("This should never happen" && false);
        return std::shared_ptr<Cell>();
    }
}




// |----------------------| //
// |                      | //
// | BoundingCage methods | //
// |                      | //
// |======================| //

bool BoundingCage::set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters, const Eigen::Vector4d& bounding_box) {
    assert(cells.begin() == cells.end());
    assert(keyframes.begin() == keyframes.end());
    assert(cells.rbegin() == cells.rend());
    assert(keyframes.rbegin() == keyframes.rend());

    std::function<void(int)> smooth_skeleton = [&](int num_iters) {
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

    // Fit the an initial BoundingCage to the skeleton. This will
    // attempt to construct a series of prisms which fully enclose the skeleton
    // vertices.
    //
    // If the resulting BoundingCage does not contain all the skeleton vertices,
    // this method returns false.
    std::function<bool(std::shared_ptr<Cell>)> fit_cage_rec = [&](std::shared_ptr<Cell> cell) -> bool {
        // If all the skeleton vertices are in the cage node, then we're done
        if (skeleton_in_cell(cell)) {
            return true;
        }

        // Otherwise split the cage node and try again
        const int mid = cell->min_index() + (cell->max_index() - cell->min_index()) / 2;
        if (mid == cell->min_index() || mid == cell->max_index()) {
            logger->info("mid value, {}, equalled boundary ({}, {}), "
                         "while splitting cage cell in fit_cage_rec",
                         mid, cell->min_index(), cell->max_index());
            return false;
        }
        assert("Bad mid index" && (mid > 0) && (mid < SV_smooth.rows()-1));

        Eigen::RowVector2d mid_centroid = 0.5 * (cell->_left_keyframe->centroid_2d() + cell->_right_keyframe->centroid_2d());
        Eigen::RowVector3d mid_normal = 0.5 * (SV_smooth.row(mid+1) - SV_smooth.row(mid-1)).normalized();
        Eigen::MatrixXd mid_pts_2d = 0.5 * (cell->_left_keyframe->vertices_2d() + cell->_left_keyframe->vertices_2d());
        std::shared_ptr<KeyFrame> mid_keyframe(new KeyFrame(mid_normal, SV_smooth.row(mid), *cell->_left_keyframe, 0.0, mid_pts_2d, mid_centroid, cell, mid, this));
        assert("Parallel transport bug" && (1.0-fabs(mid_keyframe->normal().dot(mid_normal))) < 1e-6);

        if(split_internal(mid_keyframe)) {
            bool ret = fit_cage_rec(cell->_left_child);
            return ret && fit_cage_rec(cell->_right_child);
        } else {
            return false;
        }
    };


    clear();

    logger->info("Constructing intial bounding cage for skeleton.");
    SV = new_SV;

    if (SV.rows() <= 1) {
        logger->error("Skeleton vertices contains {} vertices which is fewer than 1. Need 1 or more vertices.", SV.rows());
        return false;
    }

    smooth_skeleton(smoothing_iters);

    Eigen::RowVector3d front_normal = SV_smooth.row(1) - SV_smooth.row(0);
    Eigen::RowVector3d back_normal = SV_smooth.row(SV.rows()-1) - SV_smooth.row(SV.rows()-2);
    front_normal.normalize();
    back_normal.normalize();

    double min_u = bounding_box[0], max_u = bounding_box[1],
           min_v = bounding_box[2], max_v = bounding_box[3];
    if (!set_keyframe_bounding_box(bounding_box)) {
        return false;
    }

    Eigen::MatrixXd poly_template(4, 2);
    poly_template << min_u, min_v,
                     max_u, min_v,
                     max_u, max_v,
                     min_u, max_v;

    Eigen::RowVector2d centroid = poly_template.colwise().mean();

    Eigen::Matrix3d front_coord_system = local_coordinate_system(front_normal);
    std::shared_ptr<KeyFrame> front_keyframe(new KeyFrame(SV.row(0), front_coord_system, 0.0, poly_template, centroid,
                                                          std::shared_ptr<Cell>(), 0, this));
    std::shared_ptr<KeyFrame> back_keyframe(new KeyFrame(back_normal, SV.row(SV.rows()-1), *front_keyframe, 0.0,
                                                         poly_template, centroid, std::shared_ptr<Cell>(),
                                                         SV.rows()-1, this));
    front_keyframe->_in_cage = true;
    back_keyframe->_in_cage = true;
    logger->debug("1-<bn, transport(fn)> = {}", 1.0-fabs(back_keyframe->normal().dot(back_normal)));

    assert("Parallel transport bug" && (1.0-fabs(back_keyframe->normal().dot(back_normal))) < 1e-6);

    root = Cell::make_cell(front_keyframe, back_keyframe, this);
    if (!root) {
        // TODO: Use the bounding box of the mesh instead
        logger->warn("set_skeleton_vertices was unable to fit the first Cell. Using bounding box instead.");
        assert("TODO: Use bounding box" && false);
        return false;
    }

    cells.head = root;
    cells.tail = cells.head;

    if (!fit_cage_rec(root)) {
        logger->info("Initial bounding cage does not contain all the skeleton vertices.");
    } else {
        logger->info("Successfully fit all skeleton vertices inside BoundingCage.");
    }

    logger->info("Done constructing initial cage for skeleton.");
    _num_keyframes += 2; // Head and tail keyframes
    return true;
}

BoundingCage::KeyFrameIterator BoundingCage::keyframe_for_index(double index) const {
    std::shared_ptr<Cell> cell = root->find(index);

    if (!cell) {
        logger->error("vertices_2d_for_index() could not find cell at index {}", index);
        return KeyFrameIterator();
    }

    // If the index matches one of the cell boundaries, return the KeyFrame on that boundary
    if (cell->min_index() == index) {
        return KeyFrameIterator(cell->_left_keyframe);
    } else if (cell->max_index() == index) {
        return KeyFrameIterator(cell->_right_keyframe);
    }

    const double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
    Eigen::MatrixXd V = (1.0-coeff)*cell->_left_keyframe->bounding_box_vertices_3d() + coeff*cell->_right_keyframe->bounding_box_vertices_3d();
    Eigen::RowVector3d origin = (1.0-coeff)*cell->_left_keyframe->origin() + coeff*cell->_right_keyframe->origin();


    Eigen::MatrixXd A = V.rowwise() - origin;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    Eigen::RowVector3d n = svd.matrixV().col(2).transpose();
    double sign = 1.0;
    if (n.dot(cell->_left_keyframe->normal()) < 0.0 || n.dot(cell->_right_keyframe->normal()) < 0.0) {
        sign = -1.0;
    }
    n *= sign;

    double angle = (1.0-coeff)*cell->_left_keyframe->angle() + coeff*cell->_right_keyframe->angle();

    Eigen::Matrix3d coord_frame = parallel_transport(cell->_left_keyframe->_orientation, n);

    Eigen::MatrixXd points2d(V.rows(), 2);
    points2d.col(0) = A*coord_frame.row(0).transpose();
    points2d.col(1) = A*coord_frame.row(1).transpose();

    Eigen::RowVector2d centroid2d = (1.0-coeff)*cell->_left_keyframe->centroid_2d() + coeff*cell->_right_keyframe->centroid_2d();
    std::shared_ptr<KeyFrame> kf(new KeyFrame(origin, coord_frame, angle, points2d, centroid2d, cell, index, (BoundingCage*)this));
    return KeyFrameIterator(kf);
}

bool BoundingCage::skeleton_in_cell(std::shared_ptr<Cell> cell) const {
    int start = cell->min_index();
    int end = cell->max_index();
    std::shared_ptr<KeyFrame> left_kf = cell->_left_keyframe, right_kf = cell->_right_keyframe;

    assert(left_kf->bounding_box_vertices_2d().rows() == right_kf->bounding_box_vertices_2d().rows());
    Eigen::MatrixXd CHV(left_kf->bounding_box_vertices_2d().rows() + right_kf->bounding_box_vertices_2d().rows(), 3);
    CHV.block(0, 0, left_kf->bounding_box_vertices_2d().rows(), 3) = left_kf->bounding_box_vertices_3d();
    CHV.block(left_kf->bounding_box_vertices_2d().rows(), 0, right_kf->bounding_box_vertices_2d().rows(), 3) = right_kf->bounding_box_vertices_3d();

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::copyleft::cgal::convex_hull(CHV, V, F);
    if (CHV.rows() != V.rows()) {
        logger->warn("Convex Hull of keyframes for Cell does not enclose all its points. "
                     "There were {} input points but the convex hull had {} points.",
                     CHV.rows(), V.rows());
    }

    assert(start < end);
    if ((end - start) <= 1) {
        return false;
    }

    auto sgn = [](double val) -> int {
        return (double(0) < val) - (val < double(0));
    };

    Eigen::MatrixXd C(F.rows(), 3);
    for (int i = 0; i < F.rows(); i++) {
        C.row(i) = (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2))) / 3;
    }

    // Computing the normals here is fine since this function only gets called
    // on construction when the cage is very simple
    Eigen::MatrixXd N;
    igl::per_face_normals_stable(V, F, N);
    const int check_sign = sgn(N.row(0).dot(SV_smooth.row(start+1)-C.row(0)));
    for (int i = 0; i < end-start-1; i++) {
        const Eigen::RowVector3d V = SV_smooth.row(start+1+i);
        for (int j = 0; j < N.rows(); j++) {
            const int sign = sgn(N.row(j).dot(V-C.row(j)));
            if (sign != check_sign) {
                return false;
            }
        }
    }

    return true;
}

std::shared_ptr<BoundingCage::KeyFrame> BoundingCage::split_internal(std::shared_ptr<BoundingCage::KeyFrame> split_kf) {
    std::shared_ptr<Cell> left_cell = split_kf->left_cell();
    std::shared_ptr<Cell> right_cell = split_kf->right_cell();

    // A KeyFrame should never have both left and right Cell pointers set to null
    if(!left_cell && !right_cell) {
        logger->error("KeyFrame left and right cell are both null.");
        assert(false);
        return std::shared_ptr<KeyFrame>();
    }

    // If a KeyFrame is already part of the cage, we can just return it
    if (split_kf->in_bounding_cage()) {
        return split_kf;
    }

    // Otherwise, the KeyFrame has not been inserted
    // into the BoundingCage, so insert it.
    assert("KeyFrame cells not equal" && split_kf->left_cell() == split_kf->right_cell());
    std::shared_ptr<Cell> cell(left_cell);
    if (!cell->split(split_kf)) {
        logger->error("Failed to split Cell with KeyFrame.");
        assert(false);
        return std::shared_ptr<KeyFrame>();
    }

    // If we split the head or tail of the Cell list, then update those pointers
    if (left_cell == cells.head) {
        cells.head = left_cell->_left_child;
    }
    if (right_cell == cells.tail) {
        cells.tail = right_cell->_right_child;
    }

    _num_keyframes += 1;
    split_kf->_in_cage = true;
    return split_kf;
}

BoundingCage::KeyFrameIterator BoundingCage::insert_keyframe(double index) {
    return KeyFrameIterator(split_internal(keyframe_for_index(index).keyframe));
}

BoundingCage::KeyFrameIterator BoundingCage::insert_keyframe(BoundingCage::KeyFrameIterator& split_kf) {
    return KeyFrameIterator(split_internal(split_kf.keyframe));
}

bool BoundingCage::delete_keyframe(KeyFrameIterator& it) {
    std::shared_ptr<KeyFrame> kf = it.keyframe;
    if (!kf->in_bounding_cage()) {
        logger->warn("Cannot remove keyframe at index {} which is not contained in BoundingCage", kf->index());
        return false;
    }

    if (kf->is_endpoint()) {
        logger->warn("Cannot remove enpoint KeyFrame");
        return false;
    }

    std::shared_ptr<Cell> next_cell = kf->right_cell();
    std::shared_ptr<Cell> prev_cell = kf->left_cell();
    if (!next_cell || !prev_cell) {
        logger->error("next cell or prev cell was null");
        return false;
    }

    std::shared_ptr<KeyFrame> next_keyframe = next_cell->_right_keyframe;
    assert(next_cell);
    assert(prev_cell);

    prev_cell->_right_keyframe = next_keyframe;
    next_keyframe->_left_cell = prev_cell;

    std::shared_ptr<Cell> next_parent = next_cell->_parent_cell.lock();

    std::shared_ptr<Cell> update = next_parent;
    while (update && update->_left_keyframe == kf) {
        update->_left_keyframe = next_keyframe;
        update = update->_parent_cell.lock();
    }

    update = prev_cell->_parent_cell.lock();
    while(update && update->_right_keyframe == kf) {
        update->_right_keyframe = next_keyframe;
        update = update->_parent_cell.lock();
    }

    kf->_in_cage = false;
    _num_keyframes -= 1;
    return true;
}


const Eigen::MatrixXd BoundingCage::mesh_vertices() {
    int num_vertices = this->num_keyframes()*4;
    Eigen::MatrixXd ret (num_vertices, 3);

    int count = 0;
    for (KeyFrame& kf : this->keyframes) {
        Eigen::MatrixXd bbox_v = kf.bounding_box_vertices_3d();

        ret.row(count++) = bbox_v.row(0);
        ret.row(count++) = bbox_v.row(1);
        ret.row(count++) = bbox_v.row(2);
        ret.row(count++) = bbox_v.row(3);
    }

    return ret;
}

const Eigen::MatrixXi BoundingCage::mesh_faces() const {
    int num_faces = 4 + 8*(this->num_keyframes()-1);
    int num_vertices = this->num_keyframes()*4;

    Eigen::MatrixXi ret(num_faces, 3);

    int count = 0;
    ret.row(count++) = Eigen::RowVector3i(0, 1, 2);
    ret.row(count++) = Eigen::RowVector3i(0, 2, 3);
    ret.row(count++) = Eigen::RowVector3i(num_vertices-3, num_vertices-4, num_vertices-2);
    ret.row(count++) = Eigen::RowVector3i(num_vertices-4, num_vertices-1, num_vertices-2);

    for (int kf_i = 0; kf_i < this->num_keyframes()-1; kf_i++) {
        int kf1[4] = { kf_i*4+0, kf_i*4+1, kf_i*4+2, kf_i*4+3 };
        int kf2[4] = { (kf_i+1)*4+0, (kf_i+1)*4+1, (kf_i+1)*4+2, (kf_i+1)*4+3 };

        for (int vi = 0; vi < 4; vi++) {
            int v1 = kf1[vi], v2 = kf2[(vi+1)%4], v3 = kf1[(vi+1)%4];
            ret.row(count++) = Eigen::RowVector3i(v1, v2, v3);

            int v4 = kf1[vi], v5 = kf2[vi], v6 = kf2[(vi+1)%4];
            ret.row(count++) = Eigen::RowVector3i(v4, v5, v6);
        }
    }

    return ret;
}
