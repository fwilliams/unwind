#include "initial_file_selection_state.h"

#include "state.h"
#include "preprocessing.hpp"

#include <imgui/imgui.h>
#include <GLFW/glfw3.h>
#include <utils/mkpath.h>

Initial_File_Selection_Menu::Initial_File_Selection_Menu(State& state) : _state(state) {
#ifdef WIN32
    strcpy(ui.folder_name, "C:/Users/harishd/Desktop/Projects/Fish/data/Plagiotremus-tapinosoma");
    strcpy(ui.file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
    strcpy(ui.extension, "bmp");
    ui.start_index = 2;
    ui.end_index = 1798;
    strcpy(ui.output_folder, "C:/Users/harishd/Desktop/Projects/Fish/data/Plagiotremus-tapinosoma-output");
    strcpy(ui.output_prefix, "Plaagiotremus_tapinosoma");
#else
//    strcpy(ui.folder_name, "/home/francis/projects/fish_deformation/data/Sternopygus_arenatus"); //Plaagiotremus_tapinosoma");
//    strcpy(ui.file_prefix, "Sternopygus_arenatus_72um_1k"); //Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
//    strcpy(ui.extension, "bmp");
//    ui.start_index = 45; //2;
//    ui.end_index = 2534.;
//    strcpy(ui.output_folder, "/home/francis/projects/fish_deformation/data/Sternopygus_arenatus/output"); //Plaagiotremus_tapinosoma/output");
//    strcpy(ui.output_prefix, "Sternopygus_arenatus"); //Plaagiotremus_tapinosoma");

    strcpy(ui.folder_name, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma");
    strcpy(ui.file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
    strcpy(ui.extension, "bmp");
    ui.start_index = 2;
    ui.end_index = 1798;
    strcpy(ui.output_folder, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma/output");
    strcpy(ui.output_prefix, "Plaagiotremus_tapinosoma");
#endif
}

void Initial_File_Selection_Menu::initialize() {
    _state.logger->debug("Initializing File Selection View");
}

void Initial_File_Selection_Menu::deinitialize() {
    _state.logger->debug("De-Initializing File Selection View");
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
        ImGui::EndPopup();

        if (done_loading) {
            const Eigen::RowVector3i volume_dims = _state.low_res_volume.dims();

            if (loading_progress == -1) {
                // Volume texture
                if (_state.low_res_volume.volume_texture == 0) {
                    _state.logger->debug("Creating low resolution volume texture...");
                    glGenTextures(1, &_state.low_res_volume.volume_texture);
                }

                glBindTexture(GL_TEXTURE_3D, _state.low_res_volume.volume_texture);
                GLfloat transparent_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
                glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, transparent_color);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
                //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                //    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                if (load_textures_slice_by_slice) {
                    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, volume_dims[0], volume_dims[1], volume_dims[2], 0,
                                 GL_RED, GL_UNSIGNED_BYTE, nullptr);
                } else {
                    uint8_t* volume_data = byte_data.data();
                    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, volume_dims[0], volume_dims[1], volume_dims[2], 0,
                                 GL_RED, GL_UNSIGNED_BYTE, volume_data);
                }

                // Index texture
                if (_state.low_res_volume.index_texture == 0) {
                    _state.logger->debug("Creating low resolution index texture...");
                    glGenTextures(1, &_state.low_res_volume.index_texture);
                }
                glBindTexture(GL_TEXTURE_3D, _state.low_res_volume.index_texture);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
                if (load_textures_slice_by_slice) {
                    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, volume_dims[0], volume_dims[1], volume_dims[2],
                                 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
                } else {
                    uint32_t* index_data = _state.low_res_volume.index_data.data();
                    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, volume_dims[0], volume_dims[1], volume_dims[2],
                                 0, GL_RED_INTEGER, GL_UNSIGNED_INT, index_data);
                    loading_progress = volume_dims[2];
                }

                // reinterpret_cast<char*>(_state.low_res_volume.index_data.data())
            } else if (loading_progress < volume_dims[2]) {
                _state.logger->debug("Loading slice {}/{}", loading_progress+1, volume_dims[2]);
                const int w = volume_dims[0], h = volume_dims[1];
                const int size = w*h;

                uint8_t* volume_data = &byte_data.data()[loading_progress*size];
                glBindTexture(GL_TEXTURE_3D, _state.low_res_volume.volume_texture);
                glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, loading_progress, w, h, 1, GL_RED, GL_UNSIGNED_BYTE,
                                reinterpret_cast<void*>(volume_data));

                uint32_t* index_data = &_state.low_res_volume.index_data.data()[loading_progress*size];
                glBindTexture(GL_TEXTURE_3D, _state.low_res_volume.index_texture);
                glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, loading_progress, w, h, 1, GL_RED_INTEGER, GL_UNSIGNED_INT,
                                reinterpret_cast<void*>(index_data));

            } else {
                is_loading = false;
                done_loading = false;
                glBindTexture(GL_TEXTURE_3D, 0);
                _state.dirty_flags.file_loading_dirty = false;
                _state.set_application_state(Application_State::Segmentation);
                return ret;
            }

            loading_progress += 1;
        }
    }
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
    if (ImGui::InputText("Folder Name", ui.folder_name, BufferSize)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputText("File Prefix", ui.file_prefix, BufferSize)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputText("File extensions", ui.extension, BufferSize)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputInt("Start Index", &ui.start_index)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputInt("End Index", &ui.end_index)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputText("Output Folder", ui.output_folder, BufferSize)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputText("Output Prefix", ui.output_prefix, BufferSize)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::InputInt("Downsample Factor", &ui.downsample_factor)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    if (ImGui::Checkbox("Write Original", &ui.write_original)) {
        _state.dirty_flags.file_loading_dirty = true;
    }
    ImGui::PopItemWidth();

    ImGui::NewLine();
    ImGui::Separator();
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
            _state.segmented_features.topological_features.loadData(volume_output_files_prefix);
            
            load_rawfile(volume_output_files_prefix+ ".raw", _state.low_res_volume.dims(), _state.low_res_volume.volume_data, true);

            _state.low_res_volume.max_value = _state.low_res_volume.volume_data.maxCoeff();
            _state.low_res_volume.min_value = _state.low_res_volume.volume_data.minCoeff();

            const size_t num_bytes = _state.low_res_volume.num_voxels() * sizeof(uint32_t);
            std::ifstream file;
            file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            file.open(volume_output_files_prefix + ".part.raw", std::ifstream::binary);

            typedef decltype(_state.low_res_volume.index_data) IndexType;
            _state.low_res_volume.index_data.resize(num_bytes / sizeof(IndexType::Scalar));
            file.read(reinterpret_cast<char*>(_state.low_res_volume.index_data.data()), num_bytes);

            const Eigen::RowVector3i volume_dims = _state.low_res_volume.dims();
            const int size = volume_dims[0]*volume_dims[1]*volume_dims[2];
            byte_data.clear();
            byte_data.resize(size);

            const double min_value = _state.low_res_volume.min_value;
            const double value_range = _state.low_res_volume.max_value - _state.low_res_volume.min_value;
            float* texture_data = _state.low_res_volume.volume_data.data();
            std::transform(
                texture_data,
                texture_data + size,
                byte_data.begin(),
                [min_value, value_range](double d) {
                    return static_cast<uint8_t>(((d - min_value)/value_range) * std::numeric_limits<uint8_t>::max());
                }
            );

            _state.segmented_features.recompute_feature_map();

            done_loading = true;
            loading_progress = -1;
            glfwPostEmptyEvent();
        };

        if (!_state.dirty_flags.file_loading_dirty) {
            is_loading = false;
            done_loading = false;
            _state.set_application_state(Application_State::Segmentation);
            return ret;
        }

        is_loading = true;
        done_loading = false;
        loading_thread = std::thread(thread_fun);
        loading_thread.detach();
    }

    ImGui::End();
    ImGui::Render();
    return ret;
}
