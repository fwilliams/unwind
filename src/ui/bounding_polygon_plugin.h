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


    virtual bool post_draw() override;
    virtual bool pre_draw() override;
    void initialize();

private:
    Bounding_Polygon_Widget widget_2d;
    State& state;

    // Vertices of the current plane
    Eigen::MatrixXd PV;

    int mesh_overlay_id = 0;
    size_t cage_mesh_overlay_id = size_t(-1);
    size_t points_overlay_id = size_t(-1);

    float current_cut_index = 0;
    int current_vertex = 0;

    bool show_slice_view = false;
};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
