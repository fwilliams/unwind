#ifndef __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
#define __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__

#include "state.h"
#include "fish_ui_viewer_plugin.h"
#include "bounding_polygon_widget.h"

#include <tuple>


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
  struct BoundingCageNode {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::MatrixXd N;

    int level;

    int start;
    int end;

    std::shared_ptr<BoundingCageNode> left;
    std::shared_ptr<BoundingCageNode> right;
  };
  std::vector<std::shared_ptr<BoundingCageNode>> cage_components;

  bool skeleton_in_cage(const Eigen::MatrixXd& CC, const Eigen::MatrixXd& CN, int start, int end);
  std::shared_ptr<BoundingCageNode> make_bounding_cage_component(int v1, int v2, int level);
  bool make_bounding_cage();
  bool make_bounding_cage_r(std::shared_ptr<BoundingCageNode> root);

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> plane_for_vertex(int vid, double radius);

  Bounding_Polygon_Widget widget_2d;
  State& state;

  // Vertices of the current plane
  Eigen::MatrixXd PV;

  int mesh_overlay_id;
  int points_overlay_id;

  int current_vertex_id = 0;

  bool show_slice_view = false;


};

#endif // __FISH_DEFORMATION_BOUNDING_POLYGON_STATE__
