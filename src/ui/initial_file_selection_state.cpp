#include "initial_file_selection_state.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include "preprocessing.hpp"

#include "state.h"

Initial_File_Selection_Menu::Initial_File_Selection_Menu(State& state) :
    _state(state)
{
    // @TEMP

    strcpy(folder_name, "D:/Fish_Deformation/Plagiotremus-tapinosoma");
    strcpy(file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
    strcpy(extension, "bmp");
    start_index = 2;
    end_index = 1798;
    strcpy(output_folder, "D:/Fish_Deformation/Plagiotremus-tapinosoma/output");
    strcpy(output_prefix, "Plaagiotremus_tapinosoma");

}

void Initial_File_Selection_Menu::draw_viewer_menu() {
    ImGui::InputText("Folder Name", folder_name, Buffer_Size);
    ImGui::InputText("File Prefix", file_prefix, Buffer_Size);
    ImGui::InputText("File extensions", extension, Buffer_Size);
    ImGui::InputInt("Start Index", &start_index);
    ImGui::InputInt("End Index", &end_index);
    ImGui::InputText("Output Folder", output_folder, Buffer_Size);
    ImGui::InputText("Output Prefix", output_prefix, Buffer_Size);
    ImGui::InputInt("Downsample Factor", &downsample_factor);
    ImGui::Checkbox("Write Original", &write_original);


    bool pressed = ImGui::Button("Next");

    if (pressed) {
        SamplingOutput op = ImageData::writeOutput(folder_name, file_prefix, start_index, end_index, extension,
            output_folder, output_prefix, downsample_factor, write_original);
        preProcessing(op.fileName, op.x, op.y, op.z);

        _state.volume_base_name = std::string(output_folder) + '/' + output_prefix + "-sample";
        _state.volume_file = DatFile(_state.volume_base_name + ".dat"); // low res version

        _state.topological_features.loadData(_state.volume_base_name);

        _state.application_state = Application_State::Segmentation;
    }
}
