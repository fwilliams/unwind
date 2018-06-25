#ifndef __FISH_DEFORMATION_MESHING_STATE__
#define __FISH_DEFORMATION_MESHING_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"
#include <atomic>

class Meshing_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    Meshing_Menu(State& state);

    void draw_viewer_menu() override;

private:
    State& _state;

    bool _is_meshing = false;
    std::atomic_bool _is_finished_meshing = false;
};

#endif // __FISH_DEFORMATION_MESHING_STATE__
