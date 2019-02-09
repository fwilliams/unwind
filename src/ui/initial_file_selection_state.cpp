#include "initial_file_selection_state.h"

#include "state.h"
#include "preprocessing.hpp"

#include <imgui/imgui.h>
#include <GLFW/glfw3.h>
#include <utils/mkpath.h>

Initial_File_Selection_Menu::Initial_File_Selection_Menu(State& state) : _state(state) {
#ifdef WIN32
    strcpy(ui.folder_name, "C:/ab7512/Plagiotremus-tapinosoma");
    strcpy(ui.file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
    strcpy(ui.extension, "bmp");
    ui.start_index = 2;
    ui.end_index = 1798;
    strcpy(ui.output_folder, "C:/ab7512/Plagiotremus-tapinosoma-output");
    strcpy(ui.output_prefix, "Plaagiotremus_tapinosoma");
#else
    strcpy(ui.folder_name, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma");
    strcpy(ui.file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
    strcpy(ui.extension, "bmp");
    ui.start_index = 2;
    ui.end_index = 1798;
    strcpy(ui.output_folder, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma/output");
    strcpy(ui.output_prefix, "Plaagiotremus_tapinosoma");
#endif
}

bool Initial_File_Selection_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

    int width;
    int height;
    glfwGetWindowSize(viewer->window, &width, &height);
    float w = static_cast<float>(width);
    float h = static_cast<float>(height);
    ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiSetCond_Always);
    ImGui::Begin("Load a fish!", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    if (is_loading) {
        ImGui::OpenPopup("Loading CT Scan");
        ImGui::BeginPopupModal("Loading CT Scan");
        ImGui::Text("Loading CT Scan. Please wait as this can take a few seconds.");
        ImGui::NewLine();
        ImGui::Separator();
        if (ImGui::Button("Cancel")) {
            // TODO: Cancel button
        }
        ImGui::EndPopup();

        if (done_loading) {
            is_loading = false;
            done_loading = false;
            _state.set_application_state(Application_State::Segmentation);
        }
    }
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
    ImGui::InputText("Folder Name", ui.folder_name, BufferSize);
    ImGui::InputText("File Prefix", ui.file_prefix, BufferSize);
    ImGui::InputText("File extensions", ui.extension, BufferSize);
    ImGui::InputInt("Start Index", &ui.start_index);
    ImGui::InputInt("End Index", &ui.end_index);
    ImGui::InputText("Output Folder", ui.output_folder, BufferSize);
    ImGui::InputText("Output Prefix", ui.output_prefix, BufferSize);
    ImGui::InputInt("Downsample Factor", &ui.downsample_factor);
    ImGui::Checkbox("Write Original", &ui.write_original);
    ImGui::PopItemWidth();

    if (ImGui::Button("Next")) {
        auto thread_fun = [&]() {
            mkpath(ui.output_folder, 0777 /* mode */);

            SamplingOutput op = ImageData::writeOutput(ui.folder_name, ui.file_prefix,
                ui.start_index, ui.end_index, ui.extension, ui.output_folder,
                ui.output_prefix, ui.downsample_factor, ui.write_original);
            preProcessing(op.fileName, op.x, op.y, op.z);

            std::string volume_output_files_prefix = std::string(ui.output_folder) + '/' + ui.output_prefix + "-sample";

            // low res version
            _state.low_res_volume.metadata = DatFile(volume_output_files_prefix + ".dat");
            _state.topological_features.loadData(volume_output_files_prefix);
            
            load_rawfile(volume_output_files_prefix+ ".raw", _state.low_res_volume.dims(), _state.low_res_volume.volume_data, true);

            const size_t num_bytes = _state.low_res_volume.num_voxels() * sizeof(uint32_t);
            std::ifstream file;
            file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            file.open(volume_output_files_prefix + ".part.raw", std::ifstream::binary);

            typedef decltype(_state.low_res_volume.index_data) IndexType;
            _state.low_res_volume.index_data.resize(num_bytes / sizeof(IndexType::Scalar));
            file.read(reinterpret_cast<char*>(_state.low_res_volume.index_data.data()), num_bytes);

            done_loading = true;
            glfwPostEmptyEvent();
        };

        is_loading = true;
        done_loading = false;
        loading_thread = std::thread(thread_fun);
        loading_thread.detach();
    }

    ImGui::End();
    ImGui::Render();
    return ret;
}
