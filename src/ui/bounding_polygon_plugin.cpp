#include "bounding_polygon_plugin.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

Bounding_Polygon_Menu::Bounding_Polygon_Menu(State& state)
  : _state(state)
{}


void Bounding_Polygon_Menu::initialize() {

}


bool Bounding_Polygon_Menu::post_draw() {
  bool ret = FishUIViewerPlugin::post_draw();
  return ret;
}


bool Bounding_Polygon_Menu::pre_draw() {
  bool ret = FishUIViewerPlugin::pre_draw();
  return ret;
}
