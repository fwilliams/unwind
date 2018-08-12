#ifndef __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
#define __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__

#include <vector>
#include <array>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "state.h"
#include "fish_ui_viewer_plugin.h"

class EndPoint_Selection_Menu : public FishUIViewerPlugin {
public:
    EndPoint_Selection_Menu(State& state);

    virtual bool post_draw() override;
    virtual bool pre_draw() override;
    virtual bool mouse_down(int button, int modifier) override;
    void initialize();

private:
    bool selecting_endpoints;
    State& state;

    unsigned current_endpoint_idx = 0;
    std::array<int, 2> current_endpoints = { -1, -1 };
    std::vector<std::array<int, 2>> endpoint_pairs;

    int mesh_overlay_id;
    int points_overlay_id;
};

#endif // __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
