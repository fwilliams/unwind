#ifndef __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
#define __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__

#include "fish_ui_viewer_plugin.h"

#include <atomic>
#include <thread>

struct State;

class Initial_File_Selection_Menu : public FishUIViewerPlugin {
public:
    Initial_File_Selection_Menu(State& state);

    void initialize();
    void deinitialize();
    bool post_draw() override;

private:
    State& _state;

#ifdef WIN32
    static constexpr int BufferSize = 4096;
#else
    static const int BufferSize = PATH_MAX;
#endif

    struct {
        char folder_name[BufferSize] = {};
        char file_prefix[BufferSize] = {};
        char extension[BufferSize] = {};
        int start_index = 0;
        int end_index = 0;
        char output_folder[BufferSize] = {};
        char output_prefix[BufferSize] = {};
        int downsample_factor = 4;
        bool write_original = true;
    } ui;

    bool load_textures_slice_by_slice = false;
    int loading_progress = -1;

    std::vector<uint8_t> byte_data;
    std::atomic_bool done_loading;
    std::atomic_bool is_loading;
    std::thread loading_thread;
};

#endif // __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
