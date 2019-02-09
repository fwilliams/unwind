#ifndef __FISH_DEFORMATION_STATE__
#define __FISH_DEFORMATION_STATE__

#include "bounding_cage.h"
#include "preprocessing.hpp"
#include "volume_rendering.h"
#include "utils/utils.h"
#include "utils/datfile.h"

#include <array>
#include <GLFW/glfw3.h>
#include <vector>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

constexpr const char* FishLoggerName = "fish_deformation_logger";

enum class Application_State {
    Initial_File_Selection = 0,
    Segmentation,
    Meshing,
    EndPointSelection,
    BoundingPolygon,
    Straightening,
    Rasterization
};

struct State {
    static constexpr bool Debugging = false;

    Application_State application_state = Application_State::Initial_File_Selection;

    void set_application_state(Application_State new_state) {
        application_state = new_state;
        glfwPostEmptyEvent();
    }

    std::string volume_base_name;
    DatFile volume_file;

    int num_features = 10;

    GLuint index_volume = 0;
    struct {
        GLuint index_volume = 0;
        GLuint color_by_identifier = 0;
        GLuint selection_emphasis_type = 0;
        GLuint highlight_factor = 0;
    } uniform_locations_rendering;

    struct {
        GLuint index_volume = 0;
    } uniform_locations_picking;

    contourtree::TopologicalFeatures topological_features;

    Eigen::VectorXd volume_data;
    std::vector<unsigned int> index_volume_data;
    volumerendering::Volume_Rendering volume_rendering;


    bool should_select = false;

    struct Fish_Status {
        std::vector<uint32_t> feature_list;
    };
    std::vector<Fish_Status> fishes;
    size_t current_fish = 0;

    // Sorted list of selected features
    std::vector<uint32_t> total_selection_list;
    bool selection_list_is_dirty = true;


    Eigen::VectorXd skeleton_masking_volume;

    glm::vec4 target_viewport_size = { -1.f, -1.f, -1.f, -1.f };
    uint64_t frame_counter = 0;
    const int Delta_Frame_Count_Until_Resize = 10;

    // Output of the dilation and tetrahedralization
    struct DilatedTetMesh {
        Eigen::MatrixXd TV;
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TF;
        Eigen::VectorXi connected_components;
    } dilated_tet_mesh;

    // Selected pairs of endpoints
    std::vector<std::pair<int, int>> endpoint_pairs;

    BoundingCage cage;

    // Geodesic distances
    Eigen::VectorXd geodesic_dists;

    std::shared_ptr<spdlog::logger> logger;
};

#endif // __FISH_DEFORMATION_STATE__
