#pragma optimize( "", off )

#include <string>

#include <spdlog/sinks/stdout_color_sinks.h>

#include "ui/initial_file_selection_state.h"
#include "ui/selection_plugin.h"
#include "ui/meshing_plugin.h"
#include "ui/endpoint_selection_plugin.h"
#include "ui/bounding_polygon_plugin.h"
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




void log_opengl_debug(GLenum source, GLenum type, GLuint id,
                      GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
{
  if (id == 131185 || id == 7) {
    return;
  }
  if (source == GL_DEBUG_SOURCE_APPLICATION) {
    return;
  }
  std::stringstream ss;
  ss << "OpenGL Debug msg\nSource: " << source << "\nType: " << type << "\nId: "
            << id << "\nSeverity: " << severity << "\nMessage: " << std::string(message) << '\n';
  _state.logger->debug("{}", ss.str().c_str());
#ifdef WIN32
  DebugBreak();
#endif
}


bool init(igl::opengl::glfw::Viewer& viewer) {
  initial_file_selection.init(&viewer);
  selection_menu.init(&viewer);
  meshing_menu.init(&viewer);
  endpoint_selection_menu.init(&viewer);
  bounding_polygon_menu.init(&viewer);
  //  straightening_menu.init(&viewer);
  //  rasterization_menu.init(&viewer);

  viewer.plugins.push_back(&initial_file_selection);

  _state.logger = spdlog::stdout_color_mt("stdout logger");
  _state.logger->set_level(spdlog::level::debug);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  glDebugMessageCallback(log_opengl_debug, NULL);

  return false;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer) {
  if (previous_state != _state.get_application_state()) {
    viewer.plugins.clear();

    switch (_state.get_application_state()) {
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
      bounding_polygon_menu.initialize();
      viewer.plugins.push_back(&bounding_polygon_menu);
      break;
    case Application_State::Straightening:
      viewer.plugins.push_back(&straightening_menu);
      break;
    case Application_State::Rasterization:
      viewer.plugins.push_back(&rasterization_menu);
      break;
    }

    previous_state = _state.get_application_state();

    glfwPostEmptyEvent();
    return true;
  }

  switch (_state.get_application_state()) {
  case Application_State::Segmentation:
    selection_menu.draw_setup();
    break;
  }

  return false;
}

bool post_draw(igl::opengl::glfw::Viewer& viewer) {
  _state.frame_counter++;

  switch (_state.get_application_state()) {
  case Application_State::Segmentation:
    selection_menu.draw();
    break;
  }

  return false;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) {
  switch (_state.get_application_state()) {
  case Application_State::Segmentation:
    selection_menu.key_down(key, modifiers);
    break;
  }

  return false;
}


bool mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) {
    switch (_state.get_application_state()) {
        case Application_State::BoundingPolygon:
            return bounding_polygon_menu.mouse_move(mouse_x, mouse_y);
        default:
            return false;
    }
}

bool mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
    switch (_state.get_application_state()) {
        case Application_State::BoundingPolygon:
            return bounding_polygon_menu.mouse_down(button, modifier);
        default:
            return false;
    }
}

bool mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
    switch (_state.get_application_state()) {
        case Application_State::BoundingPolygon:
            return bounding_polygon_menu.mouse_up(button, modifier);
        default:
            return false;
    }
}

int main(int argc, char** argv) {
  igl::opengl::glfw::Viewer viewer;
  viewer.core.background_color = Eigen::Vector4f(0.1, 0.1, 0.1, 1.0);
  viewer.callback_init = init;
  viewer.callback_pre_draw = pre_draw;
  viewer.callback_post_draw = post_draw;
  viewer.callback_key_pressed = key_down;
  viewer.callback_mouse_move = mouse_move;
  viewer.callback_mouse_down = mouse_down;
  viewer.callback_mouse_up = mouse_up;
  viewer.launch();

  return EXIT_SUCCESS;
}
