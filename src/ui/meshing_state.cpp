#include "meshing_state.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

Meshing_Menu::Meshing_Menu(State& state)
    : _state(state)
{}


void Meshing_Menu::draw_viewer_menu() {
    ImGui::Text("%s", "Please wait...");


    if (!_is_meshing) {

    }

    if (_is_finished_meshing) {
        _state.application_state = Application_State::EndPointSelection;
    }
}
