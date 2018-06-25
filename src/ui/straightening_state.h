#ifndef __FISH_DEFORMATION_STRAIGHTENING_STATE__
#define __FISH_DEFORMATION_STRAIGHTENING_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"

class Straightening_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    Straightening_Menu(State& state);

    void draw_viewer_menu() override;

private:
    State& _state;
};

#endif // __FISH_DEFORMATION_STRAIGHTENING_STATE__
