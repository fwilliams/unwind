#ifndef __FISH_DEFORMATION_SELECTION_MENU__
#define __FISH_DEFORMATION_SELECTION_MENU__

#include "fish_ui_viewer_plugin.h"

struct State;

class Selection_Menu : public FishUIViewerPlugin {
public:
    Selection_Menu(State& state);

    void initialize();
    void draw_setup();
    void draw();

    void key_down(unsigned int key, int modifiers);
    bool post_draw() override;

private:
    void resize_framebuffer_textures(igl::opengl::ViewerCore& core);

    State& _state;

    float clicked_mouse_position[2] = { 0.f, 0.f };
    bool is_currently_interacting = false;
    int current_interaction_index = -1;
    bool has_added_node_since_initial_click = false;

    bool number_features_is_dirty = true;

    int current_selected_feature = -1;

    bool color_by_id = true;

    // Keep in sync with volume_fragment_shader.h and Combobox code generation
    enum class Emphasis {
        None = 0,
        OnSelection = 1,
        OnNonSelection = 2
    };
    Emphasis emphasize_by_selection = Emphasis::OnSelection;

    float highlight_factor = 0.05f;

    bool show_error_popup = false;
    std::string error_title;
    std::string error_message;
};

#endif // __FISH_DEFORMATION_SELECTION_MENU__
