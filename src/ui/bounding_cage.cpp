#include "bounding_cage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/per_face_normals.h>
#include <igl/triangle/triangulate.h>
#include <igl/segment_segment_intersect.h>

// Parallel transport a coordinate system from a KeyFrame along a curve to a point with normal, to_n
static Eigen::Matrix3d parallel_transport(const BoundingCage::KeyFrame& from_kf, Eigen::RowVector3d to_n) {
  Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(from_kf.normal().normalized(), to_n.normalized()).matrix();
  Eigen::Matrix3d coord_system = (R*from_kf.orientation().transpose()).transpose();
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
                                 const Eigen::MatrixXd& pts,
                                 std::shared_ptr<BoundingCage::Cell> cell,
                                 double idx,
                                 const BoundingCage* cage) {
  _cage = cage;
  _index = idx;
  _orientation = parallel_transport(from_kf, normal);
  _center = center;
  _vertices_2d = pts;

  _cells[0] = cell;
  _cells[1] = cell;

  logger = spdlog::get(FISH_LOGGER_NAME);
}

BoundingCage::KeyFrame::KeyFrame(const Eigen::RowVector3d& center,
                                 const Eigen::Matrix3d& coord_frame,
                                 const Eigen::MatrixXd& pts,
                                  std::shared_ptr<BoundingCage::Cell> cell,
                                 double idx,
                                 const BoundingCage *cage) {
  _cage = cage;
  _index = idx;
  _orientation = coord_frame;
  _center = center;
  _vertices_2d = pts;

  _cells[0] = cell;
  _cells[1] = cell;

  logger = spdlog::get(FISH_LOGGER_NAME);
}

bool BoundingCage::KeyFrame::move_point_2d(int i, Eigen::RowVector2d& newpos, bool validate_2d, bool validate_3d) {
  if (i < 0 || i >= _vertices_2d.rows()) {
    assert("i in move_vertex() was out of range" && false);
    logger->error("move_point_2d(i, newpos), index i={} was out of range, ({}, {})",
                  i, 0, _vertices_2d.rows()-1);
    return false;
  }

  if (!in_bounding_cage()) {
    logger->warn("Cannot move KeyFrame vertex if KeyFrame is not in bounding cage");
    return false;
  }

  Eigen::RowVector2d old_pt_2d = _vertices_2d.row(i);
  Eigen::RowVector3d old_pt_3d = _cage->CV.row(_mesh_boundary_indices[i]);


  _vertices_2d.row(i) = newpos;
  ((BoundingCage*) _cage)->CV.row(_mesh_boundary_indices[i]) = _center + newpos[0]*right()+ newpos[1]*up();

  if (validate_2d && !validate_points_2d() || validate_3d && !validate_cage()) {
      _vertices_2d.row(i) = old_pt_2d;
      ((BoundingCage*) _cage)->CV.row(_mesh_boundary_indices[i]) = old_pt_3d;
      logger->debug("move_point_2d() would have created invalid cage, reverting");
      return false;
  }


  return update_mesh();
}

bool BoundingCage::KeyFrame::validate_points_2d() {
  const int num_v = _vertices_2d.rows();
  for (int i = 0; i < num_v; i++) {
    for (int j = i+2; j < num_v; j++) {
      double t, u;
      const int next_i = (i+1)%num_v;
      const int next_j = (j+1)%num_v;
      if (i == next_j) { continue; }

      Eigen::RowVector3d a1(_vertices_2d(i, 0), _vertices_2d(i, 1), 0);
      Eigen::RowVector3d a2(_vertices_2d(next_i, 0), _vertices_2d(next_i, 1), 0);
      Eigen::RowVector3d da = a2 - a1;

      Eigen::RowVector3d b1(_vertices_2d(j, 0), _vertices_2d(j, 1), 0);
      Eigen::RowVector3d b2(_vertices_2d(next_j, 0), _vertices_2d(next_j, 1), 0);
      Eigen::RowVector3d db = b2 - b1;

      if (igl::segments_intersect(a1, da, b1, db, t, u)) {
        if (t > 1.0 || u > 1.0) {
          continue;
        }
        return false;
      }
    }
  }
  return true;
}

bool BoundingCage::KeyFrame::validate_cage() {
  bool ret = true;
  std::shared_ptr<Cell> left_cell = _cells[0].lock();
  std::shared_ptr<Cell> right_cell = _cells[1].lock();

  if (left_cell) {
  }

  if(right_cell) {
  }

  return ret;
}

bool BoundingCage::KeyFrame::init_mesh(bool tesellate) {
  BoundingCage* unconst = (BoundingCage*) _cage;

  if (tesellate) {
    logger->trace("Triangulating KeyFrame...");
    Eigen::MatrixXi E(_vertices_2d.rows(), 2);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd H(0, 2);
    Eigen::VectorXi VMin(_vertices_2d.rows()), VMout;
    Eigen::VectorXi EMin, EMout;
    std::string args("Q");

    for (int i = 0; i < _vertices_2d.rows(); i++) {
      E.row(i) = Eigen::RowVector2i(i, (i+1)%_vertices_2d.rows());
      VMin[i] = i+1;
    }

    igl::triangle::triangulate(_vertices_2d, E, H, VMin, EMin, args, V, F, VMout, EMout);

    while (unconst->CV.rows() < unconst->num_mesh_vertices + V.rows()) {
      logger->trace("resizing CV!");
      unconst->CV.conservativeResize(2*unconst->CV.rows(), 3);
    }
    while (unconst->CF.rows() < unconst->num_mesh_faces + F.rows()) {
      logger->trace("resizing CF!");
      unconst->CF.conservativeResize(2*unconst->CF.rows(), 3);
    }

    logger->trace("Triangle outputted {} vertices and {} faces", V.rows(), F.rows());
    _mesh_boundary_indices.resize(_vertices_2d.rows());
    _mesh_interior_indices.resize(V.rows());

    int icount = 0;
    for (int i = 0; i < V.rows(); i++) {
      const int vidx = unconst->num_mesh_vertices + i;
      if (VMout[i] != 0) {
        _mesh_boundary_indices[VMout[i]-1] = vidx;
      } else {
        _mesh_interior_indices[icount++] = vidx;
      }
      unconst->CV.row(vidx) = V(i, 0)*right() + V(i, 1)*up() + center();
    }
    _mesh_interior_indices.conservativeResize(icount);

    for (int i = 0; i < F.rows(); i++) {
      unconst->CF.row(i + unconst->num_mesh_faces) = F.row(i) + Eigen::RowVector3i::Ones() * unconst->num_mesh_vertices;
    }
    _mesh_face_indices.resize(F.rows());
    _mesh_face_indices.setLinSpaced(unconst->num_mesh_faces, unconst->num_mesh_faces+F.rows()-1);

    unconst->num_mesh_faces += F.rows();
    unconst->num_mesh_vertices += V.rows();
    _is_triangulated = true;

  } else {
    int num_new_vertices = _vertices_2d.rows();
    while (unconst->CV.rows() < unconst->num_mesh_vertices + num_new_vertices) {
      logger->trace("resizing CV!");
      unconst->CV.conservativeResize(2*unconst->CV.rows(), 3);
    }
    unconst->CV.block(unconst->num_mesh_vertices, 0, num_new_vertices, 3) = vertices_3d();
    _mesh_interior_indices.resize(0, 0);
    _mesh_boundary_indices.resize(num_new_vertices);
    _mesh_boundary_indices.setLinSpaced(unconst->num_mesh_vertices,
                                 unconst->num_mesh_vertices+num_new_vertices-1);
    unconst->num_mesh_vertices += num_new_vertices;
  }

  return true;
}

bool BoundingCage::KeyFrame::update_mesh() {
  if (!_is_triangulated) {
    return true;
  }

  // Compute a new triangulation of the KeyFrame boundary
  logger->trace("Re-triangulating KeyFrame...");
  Eigen::MatrixXi E(_vertices_2d.rows(), 2);
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd H(0, 2);
  Eigen::VectorXi VMin(_vertices_2d.rows()), VMout;
  Eigen::VectorXi EMin, EMout;
  std::string args("Q");
  for (int i = 0; i < _vertices_2d.rows(); i++) {
    E.row(i) = Eigen::RowVector2i(i, (i+1)%_vertices_2d.rows());
    VMin[i] = i+1;
  }
  igl::triangle::triangulate(_vertices_2d, E, H, VMin, EMin, args, V, F, VMout, EMout);


  BoundingCage* bc = (BoundingCage*) _cage;

  Eigen::MatrixXd newV(V.rows(), 3);
  Eigen::VectorXi vmap(V.rows());
  int vcount = 0;
  for (int i = 0; i < V.rows(); i++) {
    Eigen::RowVector3d v = V(i, 0)*right() + V(i, 1)*up() + center();
    if (VMout[i] == 0) {
      // If not a boundary vertex, then we'll add a new vertex
      newV.row(vcount) = v;
      vmap[i] = vcount;
      vcount += 1;
    } else {
      // The number of boundary vertices should never change,
      // so we can just update the vertex directly
      const int vidx = _mesh_boundary_indices[VMout[i]-1];
      bc->update_vertex(vidx, v);
      vmap[i] = VMout[i]-1;
    }
  }
  newV.conservativeResize(vcount, 3);
  bc->remove_vertices(Eigen::Map<VectorXi>(_mesh_faces, _mesh_faces.size()));
  bc->add_vertices(newV, _mesh_interior_indices);

  for (int i = 0; i < F.rows(); i++) {
    for (int j = 0; j < 3; j++) {
      const int vid = F(i, j);
      if (VMout[vid] == 0) {
        _mesh_faces(i, j) = _mesh_interior_indices[vmap[vid]];
      } else {
        _mesh_faces(i, j) = _mesh_boundary_indices[vmap[vid]];
      }
    }
  }

  return true;
}



// |----------------------------| //
// |                            | //
// | BoundingCage::Cell methods | //
// |                            | //
// |============================| //

std::shared_ptr<BoundingCage::Cell> BoundingCage::Cell::make_cell(std::shared_ptr<BoundingCage::KeyFrame> left_kf,
                                                                  std::shared_ptr<BoundingCage::KeyFrame> right_kf,
                                                                  const BoundingCage* cage,
                                                                  std::shared_ptr<BoundingCage::Cell> prev,
                                                                  std::shared_ptr<BoundingCage::Cell> next) {
  std::shared_ptr<Cell> ret(new Cell(left_kf, right_kf, prev, next, cage));

  std::shared_ptr<spdlog::logger> logger = ret->logger;

  if (!left_kf || !right_kf) {
    logger->warn("Cell::make_cell failed to construct Cell. "
                 "One or both of left_kf or right_kf was null.");
    ret.reset();
    return ret;
  }

  left_kf->_cells[1] = ret;
  right_kf->_cells[0] = ret;

  assert(left_kf->vertices_2d().rows() == right_kf->vertices_2d().rows());
  Eigen::MatrixXd CHV(left_kf->vertices_2d().rows() + right_kf->vertices_2d().rows(), 3);
  CHV.block(0, 0, left_kf->vertices_2d().rows(), 3) = left_kf->vertices_3d();
  CHV.block(left_kf->vertices_2d().rows(), 0, right_kf->vertices_2d().rows(), 3) = right_kf->vertices_3d();

  Eigen::MatrixXd V;
  igl::copyleft::cgal::convex_hull(CHV, V, ret->F);
  ret->V = V;
  if (CHV.rows() != V.rows()) {
    logger->warn("Convex Hull of keyframes for Cell does not enclose all its points. "
                 "There were {} input points but the convex hull had {} points.",
                 CHV.rows(), V.rows());
    ret.reset();
    return ret;
  }

  return ret;
}

bool BoundingCage::Cell::init_mesh() {
  assert(left_kf->vertices_2d().rows() == right_kf->vertices_2d().rows());

  BoundingCage* unconst = (BoundingCage*) cage;
  int num_new_faces = 2 * left_keyframe->vertices_2d().rows();

  while (unconst->CF.rows() < unconst->num_mesh_faces + num_new_faces) {
    logger->trace("resizing CF!");
    logger->trace("old size is {}", unconst->CF.rows());
    unconst->CF.conservativeResize(2*unconst->CF.rows(), 3);
    logger->trace("new size is {}", unconst->CF.rows());
  }

  int fcount = cage->num_mesh_faces;
  for (int i = 0; i < left_keyframe->vertices_2d().rows(); i++) {
    int next_i =(i+1) % right_keyframe->_mesh_boundary_indices.rows();
    unconst->CF.row(fcount++) = Eigen::RowVector3i(
          left_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[next_i]);
    unconst->CF.row(fcount++) = Eigen::RowVector3i(
          left_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[next_i],
          left_keyframe->_mesh_boundary_indices[next_i]);
  }
  _mesh_faces.conservativeResize(num_new_faces);
  _mesh_faces.setLinSpaced(unconst->num_mesh_faces, unconst->num_mesh_faces+num_new_faces-1);
  unconst->num_mesh_faces += num_new_faces;
}

bool BoundingCage::Cell::init_mesh(Cell& parent) {
  assert(left_kf->vertices_2d().rows() == right_kf->vertices_2d().rows());

  BoundingCage* unconst = (BoundingCage*) cage;

  int fcount = 0;
  for (int i = 0; i < left_keyframe->vertices_2d().rows(); i++) {
    int next_i =(i+1) % right_keyframe->_mesh_boundary_indices.rows();
    unconst->CF.row(parent._mesh_faces[fcount++]) = Eigen::RowVector3i(
          left_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[next_i]);
    unconst->CF.row(parent._mesh_faces[fcount++]) = Eigen::RowVector3i(
          left_keyframe->_mesh_boundary_indices[i],
          right_keyframe->_mesh_boundary_indices[next_i],
          left_keyframe->_mesh_boundary_indices[next_i]);
  }
  _mesh_faces = parent._mesh_faces;
  parent._mesh_faces.conservativeResize(0, 0);
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
    return right_keyframe;
  }
  if (keyframe->index() == min_index()) {
    return left_keyframe;
  }

  // This node is a leaf node, so split it
  if (!left_child && !right_child) {

    // Initialize new Cells for the left and right children
    left_child = Cell::make_cell(left_keyframe, keyframe, cage);
    if (!left_child) {
      logger->warn("Cell::split() failed to construct left child.");
      return std::shared_ptr<KeyFrame>();
    }
    right_child = Cell::make_cell(keyframe, right_keyframe, cage);
    if (!right_child) {
      logger->warn("Cell::split() failed to construct right child.");
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
    left_keyframe->_cells[1] = left_child;
    right_keyframe->_cells[0] = right_child;
    keyframe->_cells[0] = left_child;
    keyframe->_cells[1] = right_child;

    // If we added a cell with a different normal than the left key-frame, we need to recompute the
    // the local coordinate frame of the right keyframe
    right_keyframe->_orientation = parallel_transport(*right_child->left_keyframe, right_keyframe->normal());

    // This cell is no longer a leaf, so clear its linked list pointers
    next_cell.reset();
    prev_cell.reset();

    // We've successfully inserted, update the BoundingCage mesh
    keyframe->init_mesh();
    left_child->init_mesh(*this);
    right_child->init_mesh();

    return keyframe;

  // This node is not a leaf node, split one of the children
  } else if(keyframe->index() > left_child->min_index() && keyframe->index() < left_child->max_index()) {
    return left_child->split(keyframe);
  } else if(keyframe->index() > right_child->min_index() && keyframe->index() < right_child->max_index()) {
    return right_child->split(keyframe);
  } else {
    logger->error("BoundingCage tree is in an incorrect state. split() failed."
                  "Cell split index = {}, left_keyframe_index = {}, "
                  "right_keyframe_index = {}, "
                  "left_child = {0:x}, right_child = {0:x}.",
                  keyframe->index(), left_keyframe->index(), right_keyframe->index(),
                  (std::ptrdiff_t)left_child.get(), (std::ptrdiff_t)right_child.get());
    assert("BoundingCage tree is in a bad state" && false);
    assert("BoundingCage tree is in a bad state" && ((left_child && !right_child) || (right_child && !left_child)));
    return std::shared_ptr<KeyFrame>();;
  }
  assert(false);
}






// |----------------------| //
// |                      | //
// | BoundingCage methods | //
// |                      | //
// |======================| //

bool BoundingCage::set_skeleton_vertices(const Eigen::MatrixXd& new_SV, unsigned smoothing_iters, const Eigen::MatrixXd &poly_template) {
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

    Eigen::RowVector3d mid_normal = 0.5 * (SV_smooth.row(mid+1) - SV_smooth.row(mid-1)).normalized();
    Eigen::MatrixXd mid_pts_2d = 0.5 * (cell->left_keyframe->vertices_2d() + cell->left_keyframe->vertices_2d());
    std::shared_ptr<KeyFrame> mid_keyframe(new KeyFrame(mid_normal, SV_smooth.row(mid), *cell->left_keyframe, mid_pts_2d, cell, mid, this));
    assert("Parallel transport bug" && (1.0-fabs(mid_keyframe->normal().dot(mid_normal))) < 1e-6);

    if(split_internal(mid_keyframe)) {
      bool ret = fit_cage_rec(cell->left_child);
      return ret && fit_cage_rec(cell->right_child);
    } else {
      return false;
    }
  };


  clear();
  CV.resize(poly_template.rows(), 3);
  CV_refcount.resize(CV.rows());

  CF.resize(CV.rows(), 3);

  logger = spdlog::get(FISH_LOGGER_NAME);

  logger->info("Constructing intial bounding cage for skeleton.");
  SV = new_SV;

  if (SV.rows() <= 1) {
    logger->error("Skeleton vertices contains {} vertices which is fewer than 1. Need 1 or more vertices.", SV.rows());
    return false;
  }

  smooth_skeleton(smoothing_iters);

  if(poly_template.rows() < 3 || poly_template.cols() != 2) {
    logger->error("Polygon template passed to set_skeleton_vertices was invalid. "
                  "It needs to contain at least 3 points of dimension 2."
                  "Got an input matrix of size {}x{}.", poly_template.rows(), poly_template.cols());
    return false;
  }

  { // Check that polygon template is convex
    Eigen::MatrixXd PTP(poly_template.rows(), 3);
    PTP.setZero();
    PTP.block(0, 0, poly_template.rows(), 2) = poly_template;
    Eigen::MatrixXd PTP_W;
    Eigen::MatrixXi PTP_G;
    igl::copyleft::cgal::convex_hull(PTP, PTP_W, PTP_G);

    if (PTP_W.rows() != PTP.rows()) {
      logger->error("Polygon template passed to set_skeleton_vertices was not convex. "
                    "It needs to be a convex polygon of dimension 2.");
      return false;
    }
  }

  Eigen::RowVector3d front_normal = SV_smooth.row(1) - SV_smooth.row(0);
  Eigen::RowVector3d back_normal = SV_smooth.row(SV.rows()-1) - SV_smooth.row(SV.rows()-2);
  front_normal.normalize();
  back_normal.normalize();

  Eigen::Matrix3d front_coord_system = local_coordinate_system(front_normal);
  std::shared_ptr<KeyFrame> front_keyframe(new KeyFrame(SV.row(0), front_coord_system, poly_template,
                                                        std::shared_ptr<Cell>(), 0, this));
  std::shared_ptr<KeyFrame> back_keyframe(new KeyFrame(back_normal, SV.row(SV.rows()-1), *front_keyframe,
                                                       poly_template, std::shared_ptr<Cell>(),
                                                       SV.rows()-1, this));
  assert("Parallel transport bug" && (1.0-fabs(back_keyframe->normal().dot(back_normal))) < 1e-6);

  root = Cell::make_cell(front_keyframe, back_keyframe, this);
  if (!root) {
    // TODO: Use the bounding box of the mesh instead
    logger->warn("set_skeleton_vertices was unable to fit the first Cell. Using bounding box instead.");
    assert("TODO: Use bounding box" && false);
    return false;
  }

  front_keyframe->init_mesh(true);
  back_keyframe->init_mesh(true);
  root->init_mesh();

  cells.head = root;
  cells.tail = cells.head;

  if (!fit_cage_rec(root)) {
    logger->info("Initial bounding cage does not contain all the skeleton vertices.");
  } else {
    logger->info("Successfully fit all skeleton vertices inside BoundingCage.");
  }

  logger->info("Done constructing initial cage for skeleton.");
  return true;
}

BoundingCage::KeyFrameIterator BoundingCage::keyframe_for_index(double index) const {
  // Recursively find the Cell for a given index in the Cell tree.
  // If index is out of range, this method returns a null pointer.
  std::function<std::shared_ptr<Cell>(std::shared_ptr<Cell>, double)> find_cell_rec =
      [&] (std::shared_ptr<Cell> cell, double index) ->std::shared_ptr<Cell> {
    if (!cell) {
      logger->error("find_cell_r(cell, index): cell is null.");
    } else if (cell && (index < cell->min_index() || index > cell->max_index())) {
      logger->error("find_cell_r(cell, index) index, {} was out of range ({}, {})",
                    index, cell->min_index(), cell->max_index());
      return std::shared_ptr<Cell>();
    } else if (!cell->left_child && !cell->right_child) {
      return cell;
    } else if (cell->left_child && index >= cell->left_child->min_index() && index <= cell->left_child->max_index()) {
      return find_cell_rec(cell->left_child, index);
    } else if (cell->right_child && index >= cell->right_child->min_index() && index <= cell->right_child->max_index()) {
      return find_cell_rec(cell->right_child, index);
    } else {
      logger->error("BUG!!! This branch in find_cell_r should never execute.");
      assert("This should never happen" && false);
      return std::shared_ptr<Cell>();
    }
  };


  auto cell = find_cell_rec(root, index);

  if (!cell) {
    logger->error("vertices_2d_for_index() could not find cell at index {}", index);
    return KeyFrameIterator();
  }

  // If the index matches one of the cell boundaries, return the KeyFrame on that boundary
  if (cell->min_index() == index) {
    return KeyFrameIterator(cell->left_keyframe);
  } else if (cell->max_index() == index) {
    return KeyFrameIterator(cell->right_keyframe);
  }

  const double coeff = (index - cell->min_index()) / (cell->max_index() - cell->min_index());
  Eigen::MatrixXd V = (1.0-coeff)*cell->left_keyframe->vertices_3d() + coeff*cell->right_keyframe->vertices_3d();
  Eigen::RowVector3d ctr = (1.0-coeff)*cell->left_keyframe->center() + coeff*cell->right_keyframe->center();

  Eigen::MatrixXd A = V.rowwise() - ctr;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::RowVector3d n = svd.matrixV().col(2).transpose();

  Eigen::Matrix3d coord_frame = parallel_transport(*cell->left_keyframe, n);

  Eigen::MatrixXd points2d(V.rows(), 2);
  points2d.col(0) = A*coord_frame.row(0).transpose();
  points2d.col(1) = A*coord_frame.row(1).transpose();

  std::shared_ptr<KeyFrame> kf(new KeyFrame(ctr, coord_frame, points2d, cell, index, this));
  return KeyFrameIterator(kf);
}

bool BoundingCage::skeleton_in_cell(std::shared_ptr<Cell> cell) const {
  int start = cell->min_index();
  int end = cell->max_index();

  assert(start < end);
  if ((end - start) <= 1) {
    return false;
  }

  auto sgn = [](double val) -> int {
      return (double(0) < val) - (val < double(0));
  };

  Eigen::MatrixXd C(cell->F.rows(), 3);
  for (int i = 0; i < cell->F.rows(); i++) {
    C.row(i) = (cell->V.row(cell->F(i, 0)) +
                cell->V.row(cell->F(i, 1)) +
                cell->V.row(cell->F(i, 2))) / 3;
  }

  // Computing the normals here is fine since this function only gets called
  // on construction when the cage is very simple
  Eigen::MatrixXd N;
  igl::per_face_normals(cell->vertices(), cell->faces(), N);
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
  std::shared_ptr<Cell> left_cell = split_kf->_cells[0].lock();
  std::shared_ptr<Cell> right_cell = split_kf->_cells[1].lock();

  // A KeyFrame should never have both left and right Cell pointers set to null
  if(!left_cell && !right_cell) {
    logger->error("KeyFrame left and right cell are both null.");
    assert(false);
    return std::shared_ptr<KeyFrame>();
  }

  // If a KeyFrame's left and right Cell pointers don't match, it
  // is already part of the cage, then don't match so we can just return it
  if (left_cell != right_cell) {
    return split_kf;
  }

  // Otherwise, the KeyFrame's Cell pointers match and the KeyFrame has not been inserted
  // into the BoundingCage, so insert it.
  assert("KeyFrame cells not equal" && split_kf->_cells[0] == split_kf->_cells[1]);
  if (!left_cell->split(split_kf)) {
    logger->error("Failed to split Cell with KeyFrame.");
    assert(false);
    return std::shared_ptr<KeyFrame>();
  }

  // If we split the head or tail of the Cell list, then update those pointers
  if (left_cell == cells.head) {
    cells.head = left_cell->left_child;
  }
  if (right_cell == cells.tail) {
    cells.tail = right_cell->right_child;
  }

  return split_kf;
}

BoundingCage::KeyFrameIterator BoundingCage::split(double index) {
  return KeyFrameIterator(split_internal(keyframe_for_index(index).keyframe));
}

BoundingCage::KeyFrameIterator BoundingCage::split(BoundingCage::KeyFrameIterator& split_kf) {
  return KeyFrameIterator(split_internal(split_kf.keyframe));
}

bool BoundingCage::update_vertex(int i, const Eigen::RowVector3d& v) {
  CV.row(i) = v;
}

bool BoundingCage::add_vertices(const Eigen::MatrixXd& V, Eigen::VectorXi& VI) {
  VI.conservativeResize(V.rows());

  if (CV_free_list.empty()) {
    while(CV.rows() < num_mesh_vertices + V.rows()) {
      logger->trace("Resizing CV");
      CV.conservativeResize(2*CV.rows(), 3);
    }
    CV.block(num_mesh_vertices, 0, V.rows(), 3) = V;
    CV_refcount.block(num_mesh_vertices, 0, V.rows(), 1).setZero();
    VI.setLinSpaced(num_mesh_vertices, num_mesh_vertices+V.rows()-1);
    num_mesh_vertices += V.rows();
    return true;
  }

  int vcount = 0;
  for (int i = 0; i < std::min((int)V.rows(), (int)CV_free_list.size()); i++) {
    int next_free = CV_free_list.back();
    CV_free_list.pop_back();
    VI[vcount] = next_free;
    CV.row(next_free) = V.row(vcount++);
  }
  const int num_overflow = V.rows()-vcount;
  if (num_overflow > 0) {
    while(CV.rows() < num_mesh_vertices + num_overflow) {
      logger->trace("Resizing CV");
      CV.conservativeResize(2*CV.rows(), 3);
    }
    CV.block(num_mesh_vertices, 0, num_overflow, 3) = V.block(vcount, 0, V.rows(), 3);
    CV_refcount.block(num_mesh_vertices, 0, num_overflow, 1).setZero();
    VI.block(vcount, 0, V.rows()-1, 1).setLinSpaced(num_mesh_vertices, num_mesh_vertices+num_overflow-1);
    num_mesh_vertices += num_overflow;
  }

  return true;
}

bool BoundingCage::remove_vertices(const Eigen::VectorXi& VI) {
  return true;
}

