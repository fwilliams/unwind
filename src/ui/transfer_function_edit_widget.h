#ifndef TRANSFER_FUNCTION_EDIT_WIDGET_H
#define TRANSFER_FUNCTION_EDIT_WIDGET_H

#include <vector>
#include <glm/glm.hpp>
#include "volume_rendering_2.h"

class TransferFunctionEditWidget
{
    std::vector<TfNode> transfer_function;

    bool transfer_function_dirty = true;
    bool is_currently_interacting = false;
    bool has_added_node_since_initial_click = false;
    glm::vec2 clicked_mouse_position = { 0.f, 0.f };
    int current_interaction_index = -1;

    bool color_edit_as_popup = false;
    bool color_popup_open = false;

public:
    const std::vector<TfNode>& tfunc() const { return transfer_function; }
    TransferFunctionEditWidget();

    bool tfdirty() const { return transfer_function_dirty; }
    void clear_dirty_bit() { transfer_function_dirty = false; }
    bool post_draw();
};

#endif // TRANSFER_FUNCTION_EDIT_WIDGET_H
