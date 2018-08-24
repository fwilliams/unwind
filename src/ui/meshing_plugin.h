#ifndef __FISH_DEFORMATION_MESHING_STATE__
#define __FISH_DEFORMATION_MESHING_STATE__

#include "fish_ui_viewer_plugin.h"
#include "state.h"

#include <atomic>
#include <thread>

class Meshing_Menu : public FishUIViewerPlugin {
public:
  Meshing_Menu(State& state);

  bool post_draw() override;
  bool pre_draw() override;

  void initialize();
private:

  // Intermediate state of the extracted surface
  struct ExtractedSurface {
    Eigen::MatrixXd V_thin;
    Eigen::MatrixXi F_thin;
    Eigen::MatrixXd V_fat;
    Eigen::MatrixXi F_fat;
  } extracted_surface;

  std::thread bg_thread;
  std::atomic_bool _is_meshing;
  std::atomic_bool _done_meshing;
  State& _state;

  Eigen::VectorXd export_selected_volume(const std::vector<uint32_t>& feature_list);
  void tetrahedralize_surface_mesh();
  void dilate_volume();
  void extract_surface_mesh();
};

#endif // __FISH_DEFORMATION_MESHING_STATE__
