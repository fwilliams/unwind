#ifndef TRANSFER_FUNCTION_EDIT_WIDGET_H
#define TRANSFER_FUNCTION_EDIT_WIDGET_H

#include <vector>
#include <glm/glm.hpp>
#include <utils/gl/volume_renderer.h>

class TransferFunctionEditWidget
{
    std::vector<TfNode> _transfer_function;

    bool _transfer_function_dirty = true;
    bool _is_currently_interacting = false;
    bool _has_added_node_since_initial_click = false;
    glm::vec2 _clicked_mouse_position = { 0.f, 0.f };
    int _current_interaction_index = -1;

    bool _color_edit_as_popup = false;
    bool _color_popup_open = false;

    float _padding_width = 0.05;
    float _aspect_ratio = 150.f / 200.f; // height / width

    float _node_radius = 10.0f;

public:
    TransferFunctionEditWidget();

    const std::vector<TfNode>& transfer_function() const { return _transfer_function; }
    float padding_width() const { return _padding_width; }
    float aspect_ratio() const { return _aspect_ratio; }
    float node_radius() const { return _node_radius; }
    bool transfer_function_dirty() const { return _transfer_function_dirty; }

    void set_aspect_ratio(float ar) { _aspect_ratio = ar; }
    void set_padding_width(float width) { _padding_width = width; }
    void set_color_edit_as_popup(bool enabled) { _color_edit_as_popup = enabled; }
    void clear_dirty_bit() { _transfer_function_dirty = false; }
    bool post_draw();
};

#endif // TRANSFER_FUNCTION_EDIT_WIDGET_H
