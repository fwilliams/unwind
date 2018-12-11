#ifndef __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
#define __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__

#include "fish_ui_viewer_plugin.h"
#include "bounding_polygon_widget.h"

struct State;

class Bounding_Polygon_Menu : public FishUIViewerPlugin {
public:
    Bounding_Polygon_Menu(State& state);

    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_down(int button, int modifier) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_scroll(float delta_y) override;

    bool post_draw() override;
    bool pre_draw() override;

    void initialize();
    void deinitialize();

private:
    float view_hsplit = 0.5; // Horizontal split for the two menus (normalized distance
    float view_vsplit = 0.2; // Vertical split for the bottom menu (normalized distance from the bottom)
    Bounding_Polygon_Widget widget_2d;
    State& state;

    int mesh_overlay_id = 0;
    size_t cage_mesh_overlay_id = size_t(-1);
    size_t points_overlay_id = size_t(-1);

    float current_cut_index = 0;
    int current_vertex = 0;

    Eigen::Vector4f old_viewport;
    Eigen::Vector4f viewer_viewport;
};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
