#ifndef __FISH_DEFORMATION_SELECTION_MENU__
#define __FISH_DEFORMATION_SELECTION_MENU__

#include "fish_ui_viewer_plugin.h"

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <utils/gl/selection_renderer.h>
#include <utils/timer.h>

struct State;

class Selection_Menu : public FishUIViewerPlugin {
public:
    Selection_Menu(State& state);

    void initialize();
    void deinitialize();

    bool key_down(int key, int modifiers) override;
    bool post_draw() override;

private:
    Eigen::RowVector4f old_viewport;
    float view_hsplit = 0.2f;

    void draw_selection_volume();

    State& _state;
    Timer _timer;
    int _num_mouse_clicks = 0;
    int _num_key_presses = 0;

    Parameters rendering_params;
    SelectionRenderer selection_renderer;

    glm::vec2 clicked_mouse_position = { 0.f, 0.f };
    bool is_currently_interacting = false;
    int current_interaction_index = -1;
    bool has_added_node_since_initial_click = false;

    bool number_features_is_dirty = true;
    bool transfer_function_dirty = true;

    std::vector<TfNode> transfer_function;
    int current_selected_feature = -1;
    bool color_by_id = true;

    // Keep in sync with volume_fragment_shader.h and Combobox code generation
    enum class Emphasis {
        None = 0,
        OnSelection = 1,
        OnNonSelection = 2
    };
    Emphasis emphasize_by_selection = Emphasis::OnSelection;
    float highlight_factor = 0.125f;

    bool show_error_popup = false;
    std::string error_title;
    std::string error_message;

    bool should_select = false;
    bool selection_list_is_dirty = true;

    glm::vec4 target_viewport_size = { -1.f, -1.f, -1.f, -1.f };
};

#endif // __FISH_DEFORMATION_SELECTION_MENU__
