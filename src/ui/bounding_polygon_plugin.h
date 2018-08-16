#ifndef __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
#define __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__

#include "state.h"
#include "fish_ui_viewer_plugin.h"

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
    State& state;

    int mesh_overlay_id;
    int points_overlay_id;

    int current_vertex_id = 0;
    int current_slice_id = 0;
    bool is_currently_on_slice = false;

    Eigen::Vector2f* current_edit_element = nullptr;

    Eigen::MatrixXd PV;

    struct {
        GLuint vao = 0;
        //GLuint vbo = 0;
        GLuint program = 0;
        
        GLint window_size_location = -1;
        GLint ll_location = -1;
        GLint lr_location = -1;
        GLint ul_location = -1;
        GLint ur_location = -1;

        GLint texture_location = -1;
        GLint tf_location = -1;
    } plane;

    struct {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint program = 0;
        
        GLint window_size_location = -1;
        GLint color = -1;
    } polygon;
};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
