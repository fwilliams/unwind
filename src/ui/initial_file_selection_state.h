#ifndef __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
#define __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__

#include "fish_ui_viewer_plugin.h"
#include "state.h"

#include <atomic>
#include <thread>


class Initial_File_Selection_Menu : public FishUIViewerPlugin {
public:
    Initial_File_Selection_Menu(State& state);

    bool post_draw() override;

private:
    State& _state;

    static const int Buffer_Size = 256;

    char folder_name[Buffer_Size] = {};
    char file_prefix[Buffer_Size] = {};
    char extension[Buffer_Size] = {};
    int start_index = 0;
    int end_index = 0;
    char output_folder[Buffer_Size] = {};
    char output_prefix[Buffer_Size] = {};
    int downsample_factor = 4;
    bool write_original = true;

    std::atomic_bool done_loading;
    std::atomic_bool is_loading;
    std::thread loading_thread;
};

#endif // __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
