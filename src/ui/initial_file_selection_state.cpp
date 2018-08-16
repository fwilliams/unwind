#include "initial_file_selection_state.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GLFW/glfw3.h>
#include <utils/mkpath.h>

#include "preprocessing.hpp"

#include "state.h"


Initial_File_Selection_Menu::Initial_File_Selection_Menu(State& state) :
  _state(state)
{
  // @TEMP

#ifdef WIN32
  strcpy(folder_name, "C:/Users/Alex/Desktop/Plagiotremus-tapinosoma");
  strcpy(file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
  strcpy(extension, "bmp");
  start_index = 2;
  end_index = 1798;
  strcpy(output_folder, "C:/Users/Alex/Desktop/Plagiotremus-tapinosoma-output");
  strcpy(output_prefix, "Plaagiotremus_tapinosoma");
#else
  strcpy(folder_name, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma");
  strcpy(file_prefix, "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra");
  strcpy(extension, "bmp");
  start_index = 2;
  end_index = 1798;
  strcpy(output_folder, "/home/francis/projects/fish_deformation/data/Plaagiotremus_tapinosoma/output");
  strcpy(output_prefix, "Plaagiotremus_tapinosoma");
#endif
}

bool Initial_File_Selection_Menu::post_draw() {
  bool ret = FishUIViewerPlugin::post_draw();

  int width;
  int height;
  glfwGetWindowSize(viewer->window, &width, &height);
  ImGui::SetNextWindowPos(ImVec2(.0f, .0f), ImGuiSetCond_Always);
  ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiSetCond_Always);
  bool _menu_visible = true;
  ImGui::Begin("Load a fish!", &_menu_visible,
               ImGuiWindowFlags_NoResize |
               ImGuiWindowFlags_NoMove |
               ImGuiWindowFlags_NoCollapse |
               ImGuiWindowFlags_NoTitleBar);

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
      _state.application_state = Application_State::Segmentation;
    }
  }
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.8f);
  ImGui::InputText("Folder Name", folder_name, Buffer_Size);
  ImGui::InputText("File Prefix", file_prefix, Buffer_Size);
  ImGui::InputText("File extensions", extension, Buffer_Size);
  ImGui::InputInt("Start Index", &start_index);
  ImGui::InputInt("End Index", &end_index);
  ImGui::InputText("Output Folder", output_folder, Buffer_Size);
  ImGui::InputText("Output Prefix", output_prefix, Buffer_Size);
  ImGui::InputInt("Downsample Factor", &downsample_factor);
  ImGui::Checkbox("Write Original", &write_original);
  ImGui::PopItemWidth();

  bool pressed = ImGui::Button("Next");

  if (pressed) {

    auto thread_fun = [&]() {
      mkpath(output_folder, 0777 /* mode */);

      SamplingOutput op = ImageData::writeOutput(folder_name, file_prefix, start_index, end_index, extension,
                                                 output_folder, output_prefix, downsample_factor, write_original);
      preProcessing(op.fileName, op.x, op.y, op.z);

      _state.volume_base_name = std::string(output_folder) + '/' + output_prefix + "-sample";
      _state.volume_file = DatFile(_state.volume_base_name + ".dat"); // low res version

      _state.topological_features.loadData(_state.volume_base_name);
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
