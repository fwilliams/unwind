#include "meshing_state.h"

#include <igl/copyleft/marching_cubes.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <Eigen/Core>


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
}


void Meshing_Menu::tetrahedralize_surface_mesh() {

}
