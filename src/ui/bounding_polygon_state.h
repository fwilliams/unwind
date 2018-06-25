#ifndef __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
#define __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"

class Bounding_Polygon_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    Bounding_Polygon_Menu(State& state);

    void draw_viewer_menu() override;

private:
    State& _state;
};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
