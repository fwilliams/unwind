#ifndef __FISH_DEFORMATION_RASTERIZATION_STATE__
#define __FISH_DEFORMATION_RASTERIZATION_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"

class Rasterization_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    Rasterization_Menu(State& state);

    void draw_viewer_menu() override;

private:
    State& _state;
};

#endif // __FISH_DEFORMATION_RASTERIZATION_STATE__
