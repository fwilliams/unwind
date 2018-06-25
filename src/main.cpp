#pragma optimize( "", off )

#include "preprocessing.hpp"
#include "volume_rendering.h"
#include "utils/datfile.h"
#include <string>

#include <gl/GL.h>

#include <igl/opengl/glfw/Viewer.h>
#include "utils/utils.h"

#include "volume_fragment_shader.h"
#include "picking_fragment_shader.h"

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

// c:\ab7512\fish_deformation\external\libigl\include\igl\opengl\glfw\imgui\ImGuiMenu.cpp
// https://www.khronos.org/opengl/wiki/Buffer_Texture



using namespace igl::opengl;
using namespace volumerendering;

// need to call preprocessing using the data name
std::string ipFolder = "D:/Fish_Deformation/Plagiotremus-tapinosoma";
//std::string ipFolder= "/home/harishd/Desktop/Projects/Fish/data/straightening/OSF/Plagiotremus-tapinosoma";
std::string filePrefix = "Plaagiotremus_tapinosoma_9.9um_2k__rec_Tra";
std::string ext = "bmp";
int stCt = 2;
int enCt = 1798;
std::string opFolder = "D:/Fish_Deformation/Plagiotremus-tapinosoma/output";
std::string opPrefix = "Plaagiotremus_tapinosoma";
int downsampleFactor = 4;
bool writeOriginal = true;

struct Contour_Information {
    GLuint ssbo = 0;
};

struct UI_State {
    int number_features = 10;
    bool number_features_is_dirty = true;

    int active_feature = 0;

    bool color_by_id = true;
};

struct State {
    std::string volume_base_name;
    DatFile volume_file;

    GLuint index_volume = 0;
    struct {
        GLuint index_volume = 0;
        GLuint id = 0;
        GLuint color_by_identifier = 0;
    } uniform_locations_rendering;

    struct {
        GLuint index_volume = 0;
    } uniform_locations_picking;

    contourtree::TopologicalFeatures topological_features;
    Contour_Information contour_information;

    Volume_Rendering volume_rendering;

    //int selected_feature = -1;
    bool should_select = false;

    // Sorted list of selected features
    std::vector<int> selection_list;

    UI_State ui_state;
} g_state;


class Transfer_Function_Menu : public igl::opengl::glfw::imgui::ImGuiMenu {
    float clicked_mouse_position[2] = { 0.f, 0.f };
    bool is_currently_interacting = false;
    int current_interaction_index = -1;
    bool has_added_node_since_initial_click = false;

public:
    void draw_viewer_menu() override {
        if (ImGui::SliderInt("Number of features", &g_state.ui_state.number_features, 1, 100)) {
            g_state.ui_state.number_features_is_dirty = true;
        }
        ImGui::SliderInt("Active feature", &g_state.ui_state.active_feature, 0, g_state.ui_state.number_features - 1);

        ImGui::Checkbox("Color by feature id", &g_state.ui_state.color_by_id);


        std::string list = std::accumulate(g_state.selection_list.begin(),
            g_state.selection_list.end(), std::string(),
            [](std::string s, int i) { return s + std::to_string(i) + ", "; });
        list = list.substr(0, list.size() - 2);

        ImGui::Text("Selected features: %s", list.c_str());


        constexpr const float Radius = 10.f;

        volumerendering::Transfer_Function& tf = g_state.volume_rendering.transfer_function;

        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 1.5f);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        //ImVec2 canvas_size = { 640.f, 150.f };
        ImVec2 canvas_size = { 200.f, 150.f };

        draw_list->AddRectFilledMultiColor(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255),
            IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
        draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x,
            canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));
        ImGui::InvisibleButton("canvas", canvas_size);

        // First render the lines
        for (size_t i = 0; i < tf.nodes.size(); ++i) {
            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

            const float x = canvas_pos.x + canvas_size.x * node.t;
            const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

            if (i > 0) {
                volumerendering::Transfer_Function::Node& prev_node = tf.nodes[i - 1];

                const float prev_x = canvas_pos.x + canvas_size.x * prev_node.t;
                const float prev_y = canvas_pos.y + canvas_size.y * (1.f - prev_node.rgba[3]);
                draw_list->AddLine(ImVec2(prev_x, prev_y), ImVec2(x, y), IM_COL32(255, 255, 255, 255));
            }
        }

        for (size_t i = 0; i < tf.nodes.size(); ++i) {
            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

            const float x = canvas_pos.x + canvas_size.x * node.t;
            const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

            if (i == current_interaction_index) {
                draw_list->AddCircleFilled(ImVec2(x, y), Radius * 1.5, IM_COL32(255, 255, 255, 255));
            }
            else {
                draw_list->AddCircleFilled(ImVec2(x, y), Radius, IM_COL32(255, 255, 255, 255));
            }

            draw_list->AddCircleFilled(ImVec2(x, y), Radius,
                IM_COL32(node.rgba[0] * 255, node.rgba[1] * 255, node.rgba[2] * 255, 255));
        }

        // If the mouse button is pressed, we either have to add a new node or move an
        // existing one
        const bool mouse_in_tf_editor = ImGui::GetIO().MousePos.x >= canvas_pos.x &&
            ImGui::GetIO().MousePos.x <= (canvas_pos.x + canvas_size.x) &&
            ImGui::GetIO().MousePos.y >= canvas_pos.y &&
            ImGui::GetIO().MousePos.y <= (canvas_pos.y + canvas_size.y);

        if (mouse_in_tf_editor) {
            if (ImGui::IsMouseDown(0)) {
                for (size_t i = 0; i < tf.nodes.size(); ++i) {
                    volumerendering::Transfer_Function::Node& node = tf.nodes[i];
                    const float x = canvas_pos.x + canvas_size.x * node.t;
                    const float y = canvas_pos.y + canvas_size.y * (1.f - node.rgba[3]);

                    const float dx = ImGui::GetIO().MousePos.x - x;
                    const float dy = ImGui::GetIO().MousePos.y - y;

                    const float r = sqrt(dx * dx + dy * dy);

                    if (r <= Radius * 2.5) {
                        clicked_mouse_position[0] = ImGui::GetIO().MousePos.x;
                        clicked_mouse_position[1] = ImGui::GetIO().MousePos.y;
                        is_currently_interacting = true;
                        current_interaction_index = i;
                        break;
                    }
                }

                if (is_currently_interacting) {
                    const float dx = ImGui::GetIO().MousePos.x - clicked_mouse_position[0];
                    const float dy = ImGui::GetIO().MousePos.y - clicked_mouse_position[1];

                    const float r = sqrt(dx * dx + dy * dy);

                    float new_t = (ImGui::GetIO().MousePos.x - canvas_pos.x) / canvas_size.x;
                    if (new_t < 0.f) {
                        new_t = 0.f;
                    }
                    if (new_t > 1.f) {
                        new_t = 1.f;
                    }

                    float new_a = 1.f - (ImGui::GetIO().MousePos.y - canvas_pos.y) / canvas_size.y;
                    if (new_a < 0.f) {
                        new_a = 0.f;
                    }
                    if (new_a > 1.f) {
                        new_a = 1.f;
                    }
                    // We don't want to move the first or last value
                    if ((current_interaction_index != 0) && (current_interaction_index != tf.nodes.size() - 1)) {
                        tf.nodes[current_interaction_index].t = new_t;
                    }
                    tf.nodes[current_interaction_index].rgba[3] = new_a;

                    using N = volumerendering::Transfer_Function::Node;
                    std::sort(tf.nodes.begin(), tf.nodes.end(),
                        [](const N& lhs, const N& rhs) { return lhs.t < rhs.t; });
                    tf.is_dirty = true;
                }
                else {
                    current_interaction_index = -1;
                    // We want to only add one node per mouse click
                    if (!has_added_node_since_initial_click) {
                        // Didn't hit an existing node
                        const float t = (ImGui::GetIO().MousePos.x - canvas_pos.x) / canvas_size.x;
                        const float a = 1.f - ((ImGui::GetIO().MousePos.y - canvas_pos.y) / canvas_size.y);

                        for (size_t i = 0; i < tf.nodes.size(); ++i) {
                            volumerendering::Transfer_Function::Node& node = tf.nodes[i];

                            if (node.t > t) {
                                volumerendering::Transfer_Function::Node& prev = tf.nodes[i - 1];

                                const float t_prime = (t - prev.t) / (node.t - prev.t);

                                const float r = prev.rgba[0] * (1.f - t_prime) + node.rgba[0] * t_prime;
                                const float g = prev.rgba[1] * (1.f - t_prime) + node.rgba[1] * t_prime;
                                const float b = prev.rgba[2] * (1.f - t_prime) + node.rgba[2] * t_prime;
                                const float a = prev.rgba[3] * (1.f - t_prime) + node.rgba[3] * t_prime;

                                tf.nodes.insert(tf.nodes.begin() + i, { t, { r, g, b, a } });
                                has_added_node_since_initial_click = true;
                                tf.is_dirty = true;
                                break;
                            }
                        }
                    }
                }
            }
            else {
                clicked_mouse_position[0] = 0.f;
                clicked_mouse_position[1] = 0.f;
                is_currently_interacting = false;

                has_added_node_since_initial_click = false;
            }
        }

        if (ImGui::Button("Remove node")) {
            if (current_interaction_index != 0 && current_interaction_index != tf.nodes.size() - 1) {
                tf.nodes.erase(tf.nodes.begin() + current_interaction_index);
                current_interaction_index = -1;
                tf.is_dirty = true;
            }
        }

        if (current_interaction_index >= 1 && current_interaction_index <= tf.nodes.size() - 1) {
            if (ImGui::ColorPicker4("Change Color", tf.nodes[current_interaction_index].rgba)) {
                tf.is_dirty = true;
            }
        }
        else {
            float rgba[4];
            ImGui::ColorPicker4("Change Color", rgba);
        }
    }
};

Transfer_Function_Menu transfer_function_menu;

bool init(igl::opengl::glfw::Viewer& viewer) {
    viewer.plugins.push_back(&transfer_function_menu);

    initialize(g_state.volume_rendering, viewer.core.viewport, ContourTreeFragmentShader,
        ContourTreePickingFragmentShader);

    g_state.uniform_locations_rendering.index_volume = glGetUniformLocation(
        g_state.volume_rendering.program.program_object, "index_volume"
    );
    g_state.uniform_locations_rendering.id = glGetUniformLocation(
        g_state.volume_rendering.program.program_object, "id"
    );
    g_state.uniform_locations_rendering.color_by_identifier = glGetUniformLocation(
        g_state.volume_rendering.program.program_object, "color_by_identifier"
    );
    g_state.uniform_locations_picking.index_volume = glGetUniformLocation(
        g_state.volume_rendering.picking_program.program_object, "index_volume"
    );

    // SSBO
    glGenBuffers(1, &g_state.contour_information.ssbo);
    
    Eigen::VectorXd volume_data;
    Eigen::RowVector3i dims = { g_state.volume_file.w, g_state.volume_file.h, g_state.volume_file.d };
    g_state.volume_rendering.parameters.volume_dimensions = {
        GLuint(g_state.volume_file.w),
        GLuint(g_state.volume_file.h),
        GLuint(g_state.volume_file.d)
    };
    g_state.volume_rendering.parameters.volume_dimensions_rcp = {
        1.f / g_state.volume_rendering.parameters.volume_dimensions[0],
        1.f / g_state.volume_rendering.parameters.volume_dimensions[1],
        1.f / g_state.volume_rendering.parameters.volume_dimensions[2]
    };
    GLuint maxDim = *std::max_element(
        g_state.volume_rendering.parameters.volume_dimensions.begin(),
        g_state.volume_rendering.parameters.volume_dimensions.end());

    g_state.volume_rendering.parameters.normalized_volume_dimensions = {
        g_state.volume_rendering.parameters.volume_dimensions[0] / static_cast<float>(maxDim),
        g_state.volume_rendering.parameters.volume_dimensions[1] / static_cast<float>(maxDim),
        g_state.volume_rendering.parameters.volume_dimensions[2] / static_cast<float>(maxDim),
    };

    load_rawfile(g_state.volume_base_name + ".raw", dims, volume_data, true);
    upload_volume_data(g_state.volume_rendering.volume_texture, dims, volume_data);



    // Index volume
    glGenTextures(1, &g_state.index_volume);
    glBindTexture(GL_TEXTURE_3D, g_state.index_volume);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    const int num_bytes = g_state.volume_file.w * g_state.volume_file.h * g_state.volume_file.d * 4;
    std::vector<char> data(num_bytes);

    std::ifstream file(g_state.volume_base_name + ".part.raw", std::ifstream::binary);
    assert(file.good());

    file.read(data.data(), num_bytes);

    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, g_state.volume_file.w, g_state.volume_file.h,
        g_state.volume_file.d, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, data.data());
    glBindTexture(GL_TEXTURE_3D, 0);

    return false;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer) {
    if (g_state.ui_state.number_features_is_dirty) {
        std::vector<contourtree::Feature> features = g_state.topological_features.getFeatures(
            g_state.ui_state.number_features, 0.f);

        uint32_t size = g_state.topological_features.ctdata.noArcs;

        // Buffer contents:
        // [0]: number of features
        // [...]: A linearized map from voxel identifier -> feature number
        std::vector<uint32_t> buffer_data(size + 1 + 1, static_cast<uint32_t>(-1));
        buffer_data[0] = static_cast<uint32_t>(features.size());
        for (size_t i = 0; i < features.size(); ++i) {
            for (uint32_t j : features[i].arcs) {
                // +1 since the first value of the vector contains the number of features
                buffer_data[j + 1] = static_cast<uint32_t>(i);
            }
        }

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, g_state.contour_information.ssbo);
        glBufferData(
            GL_SHADER_STORAGE_BUFFER,
            sizeof(uint32_t) * buffer_data.size(),
            buffer_data.data(),
            GL_DYNAMIC_COPY
        );

        g_state.ui_state.number_features_is_dirty = false;
    }

    return false;
}

bool post_draw(igl::opengl::glfw::Viewer& viewer) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (g_state.volume_rendering.transfer_function.is_dirty) {
        update_transfer_function(g_state.volume_rendering.transfer_function);
        g_state.volume_rendering.transfer_function.is_dirty = false;
    }

    
    render_bounding_box(g_state.volume_rendering, viewer.core.model, viewer.core.view,
        viewer.core.proj);

    glUseProgram(g_state.volume_rendering.program.program_object);

    // The default volume renderer already uses GL_TEXTURE0 through GL_TEXTURE3
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, g_state.index_volume);
    glUniform1i(g_state.uniform_locations_rendering.index_volume, 4);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, g_state.contour_information.ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, g_state.contour_information.ssbo);

    glUniform1i(g_state.uniform_locations_rendering.id, g_state.ui_state.active_feature);

    glUniform1i(g_state.uniform_locations_rendering.color_by_identifier, g_state.ui_state.color_by_id ? 1 : 0);

    render_volume(g_state.volume_rendering, viewer.core.model, viewer.core.view,
        viewer.core.proj, viewer.core.light_position);

    glUseProgram(g_state.volume_rendering.picking_program.program_object);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_3D, g_state.index_volume);
    glUniform1i(g_state.uniform_locations_picking.index_volume, 4);

    Eigen::Vector3f picking = pick_volume_location(g_state.volume_rendering,
        viewer.core.model, viewer.core.view, viewer.core.proj,
        { viewer.current_mouse_x, viewer.core.viewport[3] - viewer.current_mouse_y });

    if (g_state.should_select) {
        assert(picking[0] == picking[1] && picking[0] == picking[2]);
        assert(std::is_sorted(g_state.selection_list.begin(), g_state.selection_list.end()));

        int selected_feature = static_cast<int>(picking[0]);

        if (selected_feature != 0) {
            auto it = std::lower_bound(g_state.selection_list.begin(), g_state.selection_list.end(), selected_feature);

            if (it == g_state.selection_list.end()) {
                // The index was not found
                g_state.selection_list.push_back(selected_feature);
            }
            else if (*it == selected_feature) {
                // We found the feature
                g_state.selection_list.erase(it);
            }
            else {
                // We did not find the feature
                g_state.selection_list.insert(it, selected_feature);
            }

            assert(std::is_sorted(g_state.selection_list.begin(), g_state.selection_list.end()));
        }

        g_state.should_select = false;
    }

    return false;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) {
    if (key == 32) { // SPACE
        g_state.should_select = true;
    }
    return false;
}

int main(int argc, char** argv) {
    SamplingOutput op = ImageData::writeOutput(ipFolder, filePrefix, stCt, enCt, ext,
        opFolder, opPrefix, downsampleFactor, writeOriginal);
    preProcessing(op.fileName, op.x, op.y, op.z);

    g_state.volume_base_name = opFolder + '/' + opPrefix + "-sample";
    g_state.volume_file = DatFile(g_state.volume_base_name + ".dat"); // low res version

    g_state.topological_features.loadData(g_state.volume_base_name);

    igl::opengl::glfw::Viewer viewer;
    viewer.callback_init = init;
    viewer.callback_pre_draw = pre_draw;
    viewer.callback_post_draw = post_draw;
    viewer.callback_key_pressed = key_down;
    viewer.launch();
    
    return EXIT_SUCCESS;
}
