#ifndef __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
#define __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "state.h"

class Initial_File_Selection_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    Initial_File_Selection_Menu(State& state);

    void draw_viewer_menu() override;

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
};

#endif // __FISH_DEFORMATION_INITIAL_FILE_SELECTION_STATE__
