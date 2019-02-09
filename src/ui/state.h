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

    DatFile volume_file;

    int num_features = 10;

    GLuint index_volume = 0;

    contourtree::TopologicalFeatures topological_features;


    Eigen::VectorXd volume_data;
    std::vector<unsigned int> index_volume_data;
    volumerendering::Volume_Rendering volume_rendering;


    struct Fish_Status {
        std::vector<uint32_t> feature_list;
    };
    std::vector<Fish_Status> fishes;
    size_t current_fish = 0;


    Eigen::VectorXd skeleton_masking_volume;

    // Output of the dilation and tetrahedralization
    struct DilatedTetMesh {
        Eigen::MatrixXd TV;
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TF;
        Eigen::VectorXi connected_components;

        // Geodesic distances stored at each tet vertex
        Eigen::VectorXd geodesic_dists;
    } dilated_tet_mesh;

    // Selected pairs of endpoints
    std::vector<std::pair<int, int>> endpoint_pairs;

    BoundingCage cage;



    std::shared_ptr<spdlog::logger> logger;
};

#endif // __FISH_DEFORMATION_STATE__
