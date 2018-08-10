#include "meshing_plugin.h"

#include <igl/copyleft/marching_cubes.h>
#include <igl/writeOBJ.h>
#include <igl/readOBJ.h>
#include <igl/boundary_facets.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <Eigen/Core>

#include <vector>
#include <thread>
#include <type_traits>
#include <cstdlib>

#ifndef WIN32
// dirname()
#include <libgen.h>
#endif

#include <vor3d/CompressedVolume.h>
#include <vor3d/VoronoiVorPower.h>

#include "make_tet_mesh.h"
#include "make_signed_distance.h"
#include "trimesh.h"


Meshing_Menu::Meshing_Menu(State& state)
  : _state(state)
{}


bool Meshing_Menu::post_draw() {
  if(_is_meshing) {
    bool _is_active = true;
    ImGui::Begin("Segmentation Settings", &_is_active,
                 ImGuiWindowFlags_NoSavedSettings |
                 ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_Modal);
    ImGui::Text("%s", "Please wait...");
    ImGui::End();
    ImGui::Render();
  }

  return true;
}

bool Meshing_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();

  if (_done_meshing) {
    viewer->data().set_mesh(_state.dilated_surface.V, _state.dilated_surface.F);
    viewer->core.align_camera_center(_state.dilated_surface.V, _state.dilated_surface.F);
    _done_meshing = false;

    std::cout << "extracted bb " << std::endl;
    std::cout << _state.extracted_surface.V.colwise().maxCoeff() << std::endl;
    std::cout << _state.extracted_surface.V.colwise().minCoeff() << std::endl;
    std::cout << "dilated bb " << std::endl;
    std::cout << _state.dilated_surface.V.colwise().maxCoeff() << std::endl;
    std::cout << _state.dilated_surface.V.colwise().minCoeff() << std::endl;
  }
  return ret;
}

void Meshing_Menu::initialize() {
  _done_meshing = false;

  auto thread_fun = [&]() {
    _is_meshing = true;
    extract_surface_mesh();
    if (_state.extracted_surface.V.rows() == 0) {
      std::cerr << "Empty mesh!" << std::endl;
      abort();
    }
    dilate_volume();
    tetrahedralize_surface_mesh();

    _is_meshing = false;
    _done_meshing = true;

  };

  bg_thread = std::thread(thread_fun);
  bg_thread.detach();
}

static std::string do_readlink(std::string const& path) {
    char buff[PATH_MAX];
    ssize_t len = ::readlink(path.c_str(), buff, sizeof(buff)-1);
    if (len != -1) {
      buff[len] = '\0';
      return std::string(buff);
    }
    /* handle error condition */
}


void volume_to_dexels(const Eigen::VectorXd& scalars, int w, int h, int d, vor3d::CompressedVolume& dexels) {
  dexels = vor3d::CompressedVolume(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(d, h, w), 1.0, 0.0);

  int start_idx = 0;
  for (int z = 0; z < d; z++) {
    for (int y = 0; y < h; y++) {
      bool outside = true;
      int seg_entry = 0;
      for (int x = 0; x < w; x++) {
        if (outside && scalars[start_idx] > 0.0) {
          seg_entry = x;
          outside = false;
        } else if (!outside && scalars[start_idx] <= 0.0) {
          dexels.appendSegment(z, y, seg_entry, x, -1);
          outside = true;
        }
        start_idx += 1;
      }
    }
  }

}

void dexels_to_mesh(int n_samples, const vor3d::CompressedVolume &dexels,
                    Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
  std::vector<Eigen::Vector3d> grid_pts;
  std::vector<double> grid_vals;

  double min_z = std::numeric_limits<double_t>::max();
  for (int x = 0; x < dexels.gridSize()[0]; x++) {
    for (int y = 0; y < dexels.gridSize()[1]; y++) {
      if (min_z > dexels.at(x, y)[0]) {
        min_z = dexels.at(x, y)[0];
      }
    }
  }

  for (int z = -1; z < n_samples+1; ++z) {
    for (int y = -1; y < dexels.gridSize()[1]+1; ++y) {
      for (int x = -1; x < dexels.gridSize()[0]+1; ++x) {
        if (x == -1 || y == -1 || z == -1 || x == dexels.gridSize()[0] || y == dexels.gridSize()[1] || z == n_samples) {
          Eigen::Vector3d grid_ctr;
          grid_ctr[0] = dexels.origin()[0] + (x+0.5) * (dexels.extent()[0]/dexels.gridSize()[0]);
          grid_ctr[1] = dexels.origin()[1] + (y+0.5) * (dexels.extent()[1]/dexels.gridSize()[1]);
          grid_ctr[2] = dexels.origin()[2] + (z+0.5) * (dexels.extent()[2]/n_samples);
          grid_pts.push_back(grid_ctr);
          grid_vals.push_back(1);
          continue;
        }
        Eigen::Vector3d grid_ctr;
        grid_ctr[0] = dexels.origin()[0] + (x+0.5) * (dexels.extent()[0]/dexels.gridSize()[0]);
        grid_ctr[1] = dexels.origin()[1] + (y+0.5) * (dexels.extent()[1]/dexels.gridSize()[1]);
        grid_ctr[2] = dexels.origin()[2] + (z+0.5) * (dexels.extent()[2]/n_samples);

        grid_pts.push_back(grid_ctr);

        const std::vector<vor3d::Scalar>& d = dexels.at(x, y);
        int idx = -1;
        for (int i = 0; i < d.size(); i++) {
          if (grid_ctr[2] < d[i]) {
            idx = i-1;
            break;
          }
        }

        if (idx < 0) {
          grid_vals.push_back(1);
          continue;
        }

        if (idx % 2 == 0) {
          grid_vals.push_back(-1);
        } else {
          grid_vals.push_back(1);
        }
      }
    }
  }

  Eigen::MatrixXd pts(grid_vals.size(), 3);
  Eigen::VectorXd vals(grid_vals.size());
  for (int i = 0; i < grid_vals.size(); i++) {
    pts.row(i) = grid_pts[i];
    vals[i] = grid_vals[i];
  }

  igl::copyleft::marching_cubes(vals, pts, dexels.gridSize()[0]+2, dexels.gridSize()[1]+2, n_samples+2, V, F);
}

void Meshing_Menu::dilate_volume() {
  vor3d::CompressedVolume input;
  volume_to_dexels(_state.skeleton_masking_volume, _state.volume_file.w, _state.volume_file.h, _state.volume_file.d, input);

  vor3d::CompressedVolume output;

  vor3d::VoronoiMorphoVorPower op = vor3d::VoronoiMorphoVorPower();
  double time_1, time_2;
  op.dilation(input, output, 3, time_1, time_2);

  dexels_to_mesh(2*_state.volume_file.w, output, _state.dilated_surface.V, _state.dilated_surface.F);

  igl::writeOBJ("fat.obj", _state.dilated_surface.V, _state.dilated_surface.F);
  igl::writeOBJ("thin.obj", _state.extracted_surface.V, _state.extracted_surface.F);
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
          SV[readcount] = _state.skeleton_masking_volume[appendcount];
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


  if (_state.extracted_surface.V.rows() < 4 || _state.extracted_surface.F.rows() < 4) {
    // TODO: Raise an error
  }
}


void Meshing_Menu::tetrahedralize_surface_mesh() {
  using namespace std;

  const Eigen::MatrixXd& V = _state.dilated_surface.V;
  const Eigen::MatrixXi& F = _state.dilated_surface.F;

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

  _state.dilated_surface.TV.resize(mesh.verts().size(), 3);
  for (int i = 0; i < mesh.verts().size(); i++) {
    _state.dilated_surface.TV.row(i) =
        Eigen::Vector3d(mesh.verts()[i][0], mesh.verts()[i][1], mesh.verts()[i][2]);
  }
  _state.dilated_surface.TV.resize(mesh.tets().size(), 4);
  for (int i = 0; i < mesh.tets().size(); i++) {
    _state.dilated_surface.TT.row(i) =
        Eigen::Vector4i(mesh.tets()[i][0], mesh.tets()[i][1], mesh.tets()[i][2], mesh.tets()[i][3]);
  }
}
