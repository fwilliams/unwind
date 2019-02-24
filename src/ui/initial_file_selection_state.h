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

#ifdef _MSC_VER
    static constexpr int PATH_BUFFER_SIZE = 4*4096;
#else
    static const int PATH_BUFFER_SIZE = 4*PATH_MAX;
#endif

    char output_dir_path_buf[PATH_BUFFER_SIZE];
    char first_image_path_buf[PATH_BUFFER_SIZE] = {};
    char last_image_path_buf[PATH_BUFFER_SIZE] = {};
    char existing_project_path_buf[PATH_BUFFER_SIZE] = {};

    bool show_error_popup = false;
    std::string error_message;

    bool show_new_scan_menu = true;

    std::vector<uint8_t> byte_data;
    std::atomic_bool done_loading;
    std::atomic_bool is_loading;
    std::thread loading_thread;

    bool process_new_project_form();
};

#endif // __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
