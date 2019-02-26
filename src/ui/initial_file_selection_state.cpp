#include "initial_file_selection_state.h"

#include "state.h"
#include "preprocessing.hpp"

#include <imgui/imgui.h>
#include <GLFW/glfw3.h>
#include <utils/path_utils.h>
#include <utils/open_file_dialog.h>
#include <utils/string_utils.h>
#include <imgui/imgui_internal.h>

Initial_File_Selection_Menu::Initial_File_Selection_Menu(State& state) : _state(state) {}

namespace {

void fix_path(char* path) {
    std::string strpath(path);
    std::string trimmed_path = trim_copy(strpath);
    strcpy(path, trimmed_path.c_str());
}

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

    if (show_error_popup) {
        ImGui::OpenPopup("Invalid Selection");
        ImGui::BeginPopupModal("Invalid Selection");
        ImGui::Text("%s", error_message.c_str());
        ImGui::NewLine();
        ImGui::Separator();
        if (ImGui::Button("OK")) {
            show_error_popup = false;
        }
        ImGui::EndPopup();
    }

    if (is_loading) {
        ImGui::OpenPopup("Loading CT Scan");
        ImGui::BeginPopupModal("Loading CT Scan");
        ImGui::Text("Loading CT Scan. Please wait as this can take a few seconds.");
        ImGui::NewLine();
        ImGui::EndPopup();

        if (done_loading) {
            // Volume texture
            if (_state.low_res_volume.volume_texture == 0) {
                _state.logger->debug("Creating low resolution volume texture...");
                glGenTextures(1, &_state.low_res_volume.volume_texture);
            }
            _state.logger->debug("Creating low resolution volume texture...");
            _state.low_res_volume.load_gl_volume_texture(low_res_byte_data);

            _state.logger->debug("Creating low resolution index texture...");
            _state.low_res_volume.load_gl_index_texture();

            if (_state.hi_res_volume.volume_texture == 0) {
                _state.logger->debug("Creating high resolution volume texture...");
                glGenTextures(1, &_state.hi_res_volume.volume_texture);
            }
            _state.logger->debug("Creating high resolution volume texture...");
            _state.hi_res_volume.load_gl_volume_texture(high_res_byte_data);

            low_res_byte_data.clear();
            high_res_byte_data.clear();

            is_loading = false;
            done_loading = false;
            glBindTexture(GL_TEXTURE_3D, 0);
            _state.dirty_flags.file_loading_dirty = false;
            _state.set_application_state(Application_State::Segmentation);
            ImGui::End();
            ImGui::Render();
            return ret;
        }
    }

    ImGui::SetNextTreeNodeOpen(show_new_scan_menu);
    if (ImGui::CollapsingHeader("Create New Project")) {
        show_new_scan_menu = true;
        ImGui::Text("First Scan Image:");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
        if (ImGui::InputText("##First Scan", first_image_path_buf, PATH_BUFFER_SIZE)) {
            _state.dirty_flags.file_loading_dirty = true;
            fix_path(first_image_path_buf);
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.2f);
        if (ImGui::Button("Select##FirstScan")) {
            std::string first_scan = open_image_file_dialog();
            if (first_scan.size() > 0) {
                strcpy(first_image_path_buf, first_scan.c_str());
                _state.dirty_flags.file_loading_dirty = true;
                fix_path(first_image_path_buf);
                _state.logger->trace("First image {}", first_image_path_buf);
            }
        }


        ImGui::Spacing();
        ImGui::Text("Last Scan Image");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
        if (ImGui::InputText("##Last Scan", last_image_path_buf, PATH_BUFFER_SIZE)) {
            _state.dirty_flags.file_loading_dirty = true;
            fix_path(last_image_path_buf);
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.2f);
        if (ImGui::Button("Select##LastScan")) {
            std::string last_scan = open_image_file_dialog();
            if (last_scan.size() > 0) {
                strcpy(last_image_path_buf, last_scan.c_str());
                _state.dirty_flags.file_loading_dirty = true;
                fix_path(last_image_path_buf);
                _state.logger->trace("Last image {}", last_image_path_buf);
            }
        }

        ImGui::Spacing();
        ImGui::Text("Output Folder:");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
        if (ImGui::InputText("##Output Folder", output_dir_path_buf, PATH_BUFFER_SIZE)) {
            _state.dirty_flags.file_loading_dirty = true;
            fix_path(output_dir_path_buf);
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.2f);
        if (ImGui::Button("Select##Outfolder")) {
            std::string folder = open_folder_dialog();
            if (folder.size() > 0) {
                folder = folder + "/";
                strcpy(output_dir_path_buf, folder.c_str());
                _state.dirty_flags.file_loading_dirty = true;
                fix_path(output_dir_path_buf);
                _state.logger->trace("Selected output folder {}", output_dir_path_buf);
            }
        }
        ImGui::PopItemWidth();


        ImGui::Spacing();
        ImGui::Text("Downsampling Factor:");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
        if (ImGui::InputInt("##Downsample Factor", &_state.input_metadata.downsample_factor)) {
            _state.dirty_flags.file_loading_dirty = true;
        }
        ImGui::PopItemWidth();
        ImGui::NewLine();

    } else {
        show_new_scan_menu = false;
    }

    ImGui::SetNextTreeNodeOpen(!show_new_scan_menu);
    if (ImGui::CollapsingHeader("Load Existing Project")) {
        show_new_scan_menu = false;

        ImGui::Spacing();
        ImGui::Text("Existing Project File:");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
        if (ImGui::InputText("##Existing Project", existing_project_path_buf, PATH_BUFFER_SIZE)) {
            _state.dirty_flags.file_loading_dirty = true;
            fix_path(existing_project_path_buf);
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.2f);
        if (ImGui::Button("Select##ExistProj")) {
            std::string proj_path = open_load_file_dialog();
            if (proj_path.size() > 0) {
                strcpy(existing_project_path_buf, proj_path.c_str());
                _state.dirty_flags.file_loading_dirty = true;
                fix_path(existing_project_path_buf);
            }
        }


    } else {
        show_new_scan_menu = true;
    }

    ImGui::NewLine();
    ImGui::Separator();

    bool next_disabled = false;
    if (show_new_scan_menu) {
        size_t first_len = std::string(first_image_path_buf).size();
        size_t last_len = std::string(last_image_path_buf).size();
        size_t output_len = std::string(output_dir_path_buf).size();
        if (first_len == 0 || last_len == 0 || output_len == 0) {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            next_disabled = true;
        }
    } else {
        size_t proj_path_len = std::string(existing_project_path_buf).size();
        if (proj_path_len == 0) {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            next_disabled = true;
        }
    }

    if (ImGui::Button("Next")) {

        if (!_state.dirty_flags.file_loading_dirty) {
            is_loading = false;
            done_loading = false;
            _state.set_application_state(Application_State::Segmentation);
            ImGui::End();
            ImGui::Render();
            return ret;
        }

        auto thread_fun = [&]() {
            // load_volume_data clears _state.segmented_features.selected_features which we don't want to do
            // if we're deserializing. So we'll back it up and restore it.
            std::vector<uint32_t> selected_features_backup;

            if (show_new_scan_menu) {
                mkpath(_state.input_metadata.output_dir.c_str(), 0777 /* mode */);
                ImageData::writeOutput(_state.input_metadata.input_dir, _state.input_metadata.prefix,
                                       _state.input_metadata.start_index, _state.input_metadata.end_index,
                                       _state.input_metadata.file_extension, _state.input_metadata.output_dir,
                                       _state.input_metadata.full_res_prefix(),
                                       _state.input_metadata.low_res_prefix(),
                                       _state.input_metadata.downsample_factor,
                                       true /* write_original */);
            } else {
                if (!igl::deserialize(_state, "state", std::string(existing_project_path_buf))) {
                    show_error_popup = true;
                    error_message = "Existing project must be a valid project file";
                    return;
                }

                // Backup selected features
                selected_features_backup = _state.segmented_features.selected_features;

                // Set paths so we load from the loaded directory not the original one
                std::pair<std::string, std::string> existing_project_dbn = dir_and_base_name(existing_project_path_buf);
                _state.input_metadata.output_dir = existing_project_dbn.first;
                _state.input_metadata.input_dir = "";
                _state.input_metadata.file_extension = "";
            }

            _state.load_volume_data(_state.low_res_volume, _state.input_metadata.low_res_prefix(), true /* load topological features */);
            _state.low_res_volume.preprocess_volume_texture(low_res_byte_data);

            _state.hi_res_volume.metadata = DatFile(_state.input_metadata.full_res_path_prefix() + ".dat", _state.logger);
            load_rawfile(_state.input_metadata.full_res_path_prefix() + ".raw", _state.hi_res_volume.dims(), high_res_byte_data, _state.logger);

             if (!show_new_scan_menu) {
                 _state.segmented_features.selected_features = selected_features_backup;
             }

            done_loading = true;
            glfwPostEmptyEvent();
        };

        if (show_new_scan_menu) {
            if (!process_new_project_form()) {
                ImGui::End();
                ImGui::Render();
                return ret;
            }
        } else {
            if (get_file_type(existing_project_path_buf) != FT_REGULAR_FILE) {
                show_error_popup = true;
                error_message = "Existing project must be a valid project file";
                ImGui::End();
                ImGui::Render();
                return ret;
            }
        }

        is_loading = true;
        done_loading = false;
        loading_thread = std::thread(thread_fun);
        loading_thread.detach();

        if (show_new_scan_menu) {
            existing_project_path_buf[0] = '\0';
        } else {
            first_image_path_buf[0] = '\0';
            last_image_path_buf[0] = '\0';
            output_dir_path_buf[0] = '\0';
        }
    }
    if (next_disabled) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    ImGui::End();
    ImGui::Render();
    return ret;
}

bool Initial_File_Selection_Menu::process_new_project_form() {

    if (get_file_type(first_image_path_buf) != FT_REGULAR_FILE) {
        show_error_popup = true;
        error_message = "Error: First image is not a valid file.";
        return false;
    }
    if (get_file_type(last_image_path_buf) != FT_REGULAR_FILE) {
        show_error_popup = true;
        error_message = "Error: Last image is not a valid file.";
        return false;
    }

    const std::pair<std::string, std::string> first_path = dir_and_base_name(first_image_path_buf);
    const std::pair<std::string, std::string> last_path = dir_and_base_name(last_image_path_buf);
    if (first_path.first != last_path.first) {
        show_error_popup = true;
        error_message = "Error: Invalid scan images. The first and last images must be in the same directory.";
        return false;
    }
    const std::string directory = first_path.first;
    _state.logger->trace("Directories are {} {}", first_path.first, last_path.first);

    int prefix_end_idx = 0;
    const std::string first_file = first_path.second;
    const std::string last_file = last_path.second;
    for (prefix_end_idx = 0; prefix_end_idx < std::min(first_file.size(), last_file.size()); prefix_end_idx++) {
        if (first_file[prefix_end_idx] != last_file[prefix_end_idx]) {
            break;
        }
    }
    if (prefix_end_idx == 0) {
        show_error_popup = true;
        error_message = "Error: Invalid scan images.";
        return false;
    }
    const std::string prefix = first_file.substr(0, prefix_end_idx);
    _state.logger->trace("Prefix is {}", prefix);


    const std::string first_extension = first_file.substr(first_file.find_last_of(".") + 1);
    const std::string last_extension = last_file.substr(last_file.find_last_of(".") + 1);
    if (first_extension != last_extension) {
        show_error_popup = true;
        error_message = "Error: First and last scan images do not have the same file extension";
        return false;
    }
    const std::string file_extension = first_extension;
    _state.logger->trace("File extension is {}", file_extension);

    int start_index = prefix.size();
    int first_end_index = first_file.find_last_of(".");
    int last_end_index = last_file.find_last_of(".");

    std::string first_index_str = first_file.substr(start_index, first_end_index-start_index);
    std::string last_index_str = last_file.substr(start_index, last_end_index-start_index);

    int first_index = -1, last_index = -1;
    try {
        first_index = std::stoi(first_index_str);
        last_index = std::stoi(last_index_str);
    } catch(std::invalid_argument) {
        show_error_popup = true;
        error_message = "Error: Invalid scan image pair must be of the form <prefix><number>.<extension>.";
        return false;
    }

    if (last_index < first_index) {
        int swap_index = last_index;
        last_index = first_index;
        first_index = swap_index;
        _state.logger->info("First and last index order out of order. Swapping them.");
    }
    _state.logger->trace("Range {} {}", first_index, last_index);

    _state.input_metadata.input_dir = directory;
    _state.input_metadata.output_dir = std::string(output_dir_path_buf);
    _state.logger->trace("Output directory is {}", _state.input_metadata.output_dir);
    _state.input_metadata.file_extension = file_extension;
    _state.input_metadata.prefix = prefix;
    _state.input_metadata.start_index = first_index;
    _state.input_metadata.end_index = last_index;
    return true;
}
