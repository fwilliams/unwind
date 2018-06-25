#ifndef __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
#define __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"

class EndPoint_Selection_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    EndPoint_Selection_Menu(State& state);

    void draw_viewer_menu() override;

private:
    State& _state;
};

#endif // __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
