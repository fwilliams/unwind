#ifndef __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
#define __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__

#include "fish_ui_viewer_plugin.h"

#include <array>
#include <atomic>
#include <thread>

struct State;

class EndPoint_Selection_Menu : public FishUIViewerPlugin {
public:
    EndPoint_Selection_Menu(State& state);

    virtual bool post_draw() override;
    virtual bool pre_draw() override;
    virtual bool key_down(int key, int modifiers) override;
    void initialize();
    void deinitialize();

private:
    struct {
        bool drew_debug_state = false;
        bool enabled = false;
    } debug;
    void debug_draw_intermediate_state();


    State& state;

    Eigen::Vector4f old_viewport;

    float view_hsplit = 0.2f;

    bool selecting_endpoints = false;

    std::atomic_bool extracting_skeleton;
    std::atomic_bool done_extracting_skeleton;
    std::thread extract_skeleton_thread;


    bool bad_selection = false; // Flag set to true if user selects invalid endpoint pair
    std::string bad_selection_error_message;

    unsigned current_endpoint_idx = 0;
    std::array<int, 2> current_endpoints = { -1, -1 };

    int mesh_overlay_id;
    int points_overlay_id;

    void extract_skeleton();
};

#endif // __FISH_DEFORMATION_ENDPOINT_SELECTION_STATE__
