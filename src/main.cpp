#pragma optimize( "", off )

#include <string>

#include "ui/initial_file_selection_state.h"
#include "ui/selection_plugin.h"
#include "ui/meshing_plugin.h"
#include "ui/endpoint_selection_plugin.h"
#include "ui/bounding_polygon_state.h"
#include "ui/straightening_state.h"
#include "ui/rasterization_state.h"
#include "ui/state.h"

State _state;
Application_State previous_state;



//#define Debugging

Initial_File_Selection_Menu initial_file_selection(_state);
Selection_Menu selection_menu(_state);
Meshing_Menu meshing_menu(_state);
EndPoint_Selection_Menu endpoint_selection_menu(_state);
Bounding_Polygon_Menu bounding_polygon_menu(_state);
Straightening_Menu straightening_menu(_state);
Rasterization_Menu rasterization_menu(_state);





bool init(igl::opengl::glfw::Viewer& viewer) {
  initial_file_selection.init(&viewer);
  selection_menu.init(&viewer);
  meshing_menu.init(&viewer);
  endpoint_selection_menu.init(&viewer);

  //  bounding_polygon_menu.init(&viewer);
  //  straightening_menu.init(&viewer);
  //  rasterization_menu.init(&viewer);

  viewer.plugins.push_back(&initial_file_selection);

  return false;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer) {
  if (previous_state != _state.application_state) {
    viewer.plugins.clear();

    switch (_state.application_state) {
    case Application_State::Initial_File_Selection:
      viewer.plugins.push_back(&initial_file_selection);
      break;
    case Application_State::Segmentation:
      selection_menu.initialize();
      viewer.plugins.push_back(&selection_menu);
      break;
    case Application_State::Meshing:
      meshing_menu.initialize();
      viewer.plugins.push_back(&meshing_menu);
      break;
    case Application_State::EndPointSelection:
      endpoint_selection_menu.initialize();
      viewer.plugins.push_back(&endpoint_selection_menu);
      break;
    case Application_State::BoundingPolygon:
      viewer.plugins.push_back(&bounding_polygon_menu);
      break;
    case Application_State::Straightening:
      viewer.plugins.push_back(&straightening_menu);
      break;
    case Application_State::Rasterization:
      viewer.plugins.push_back(&rasterization_menu);
      break;
    }

    previous_state = _state.application_state;

    return true;
  }

  switch (_state.application_state) {
  case Application_State::Segmentation:
    selection_menu.draw_setup();
    break;
  }

  return false;
}

bool post_draw(igl::opengl::glfw::Viewer& viewer) {
  _state.frame_counter++;

  switch (_state.application_state) {
  case Application_State::Segmentation:
    selection_menu.draw();
    break;
  }

  return false;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) {
  switch (_state.application_state) {
  case Application_State::Segmentation:
    selection_menu.key_down(key, modifiers);
    break;
  }

  return false;
}

int main(int argc, char** argv) {
  igl::opengl::glfw::Viewer viewer;
  viewer.core.background_color = Eigen::Vector4f(0.1, 0.1, 0.1, 1.0);
  viewer.callback_init = init;
  viewer.callback_pre_draw = pre_draw;
  viewer.callback_post_draw = post_draw;
  viewer.callback_key_pressed = key_down;
  viewer.launch();

  return EXIT_SUCCESS;
}
