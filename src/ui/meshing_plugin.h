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

  std::thread bg_thread;
  std::atomic_bool _is_meshing;
  std::atomic_bool _done_meshing;
  State& _state;

  void extract_surface_mesh();
  void tetrahedralize_surface_mesh();
};

#endif // __FISH_DEFORMATION_MESHING_STATE__
