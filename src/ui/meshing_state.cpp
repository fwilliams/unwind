#include "meshing_state.h"

#include <igl/copyleft/marching_cubes.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <Eigen/Core>

#include <vector>

#include "make_tet_mesh.h"
#include "make_signed_distance.h"
#include "trimesh.h"

Meshing_Menu::Meshing_Menu(State& state)
  : _state(state)
{}


void Meshing_Menu::draw_viewer_menu() {
  ImGui::Text("%s", "Please wait...");


  if (!_is_meshing) {

  }
}

void Meshing_Menu::extract_surface_mesh() {
  const int w = _state.volume_file.w;
  const int h = _state.volume_file.h;
  const int d = _state.volume_file.d;

  // Grid positions and scalar values
  Eigen::MatrixXd GP((w+2)*(h+2)*(d+2), 3);
  Eigen::VectorXd SV(GP.rows());

  int readcount = 0;
  int appendcount = 0;
  for (int zi = 0; zi < d+2; zi++) {
    for (int yi = 0; yi < h+2; yi++) {
      for (int xi = 0; xi < w+2; xi++) {
        if (xi == 0 || yi == 0 || zi == 0 ||
            xi == (w+1) || yi == (h+1) || zi == (d+1)) {
          SV[readcount] = -1.0;
        } else {
          SV[readcount] = _state.volume_data[appendcount];
          appendcount += 1;
        }
        GP.row(readcount) = Eigen::RowVector3d(xi, yi, zi);
        readcount += 1;
      }
    }
  }

  igl::copyleft::marching_cubes(SV, GP, w+2, h+2, d+2,
                                _state.extracted_surface.V,
                                _state.extracted_surface.F);

  // TODO: Dilate and handle small components

  if (_state.extracted_surface.V.rows() < 4 || _state.extracted_surface.F.rows() < 4) {
    // TODO: Raise an error
  }
}


void Meshing_Menu::tetrahedralize_surface_mesh() {
  using namespace std;

  const Eigen::MatrixXd& V = _state.extracted_surface.V;
  const Eigen::MatrixXi& F = _state.extracted_surface.F;

  std::vector<Vec3i> surf_tri;
  std::vector<Vec3f> surf_x;

  for (int i = 0; i < F.rows(); i++) {
    surf_tri.push_back(Vec3i(F.row(i)[0], F.row(i)[1], F.row(i)[2]));
  }

  for (int i = 0; i < V.rows(); i++) {
    surf_x.push_back(Vec3f(V.row(i)[0], V.row(i)[1], V.row(i)[2]));
  }

  const float dx = 0.8; // TODO: Play with this a little bit

  const Eigen::RowVector3d v_min = V.colwise().minCoeff();
  const Eigen::RowVector3d v_max = V.colwise().maxCoeff();

  // Compute the bounding box of the mesh
  Vec3f xmin(v_min[0], v_min[1], v_min[2]);
  Vec3f xmax(v_max[0], v_max[1], v_max[2]);

  // Build triangle mesh data structure
  TriMesh trimesh(surf_x, surf_tri);

  // Make the level set

  // Determining dimensions of voxel grid.
  // Round up to ensure voxel grid completely contains bounding box.
  // Also add padding of 2 grid points around the bounding box.
  // NOTE: We add 5 here so as to add 4 grid points of padding, as well as
  // 1 grid point at the maximal boundary of the bounding box
  // ie: (xmax-xmin)/dx + 1 grid points to cover one axis of the bounding box
  Vec3f origin=xmin-Vec3f(2*dx);
  int ni = (int)std::ceil((xmax[0]-xmin[0])/dx)+5,
      nj = (int)std::ceil((xmax[1]-xmin[1])/dx)+5,
      nk = (int)std::ceil((xmax[2]-xmin[2])/dx)+5;

  SDF sdf(origin, dx, ni, nj, nk); // Initialize signed distance field.
  std::printf("making %dx%dx%d level set\n", ni, nj, nk);
  make_signed_distance(surf_tri, surf_x, sdf);

  // Then the tet mesh
  TetMesh mesh;

  // Make tet mesh without features
  make_tet_mesh(mesh, sdf, false /* optimize */, false /* intermediate */, false /* unsafe */);

  _state.extracted_surface.TV.resize(mesh.verts().size(), 3);
  for (int i = 0; i < mesh.verts().size(); i++) {
    _state.extracted_surface.TV.row(i) =
        Eigen::Vector3d(mesh.verts()[i][0], mesh.verts()[i][1], mesh.verts()[i][2]);
  }
  _state.extracted_surface.TV.resize(mesh.tets().size(), 4);
  for (int i = 0; i < mesh.tets().size(); i++) {
    _state.extracted_surface.TT.row(i) =
        Eigen::Vector4i(mesh.tets()[i][0], mesh.tets()[i][1], mesh.tets()[i][2], mesh.tets()[i][3]);
  }
}
