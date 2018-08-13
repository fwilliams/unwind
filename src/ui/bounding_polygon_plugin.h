#ifndef __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
#define __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__

#include "state.h"
#include "fish_ui_viewer_plugin.h"

class Bounding_Polygon_Menu : public FishUIViewerPlugin {
public:
    Bounding_Polygon_Menu(State& state);

    virtual bool post_draw() override;
    virtual bool pre_draw() override;
    void initialize();
private:
    State& state;

    int mesh_overlay_id;
    int points_overlay_id;

    int current_vertex_id = 0;
};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
