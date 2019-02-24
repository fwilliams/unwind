#ifndef __FISH_DEFORMATION_STATE__
#define __FISH_DEFORMATION_STATE__

#include <preprocessing.hpp>
#include <utils/bounding_cage.h>
#include <utils/utils.h>
#include <utils/datfile.h>

#include <array>
#include <GLFW/glfw3.h>
#include <vector>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <igl/serialize.h>


const spdlog::level::level_enum FISH_LOGGER_LEVEL = spdlog::level::trace;
constexpr const char* FISH_LOGGER_NAME = "Fish Deformation";
const spdlog::level::level_enum CONTOURTREE_LOGGER_LEVEL = spdlog::level::trace;
constexpr const char* CONTOURTREE_LOGGER_NAME = "Contour Tree";

enum class Application_State {
    Initial_File_Selection = 0,
    Segmentation,
    Meshing,
    EndPointSelection,
    BoundingPolygon,
    NoState,
};

struct State {
    static constexpr bool Debugging = false;

    typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> VectorXui;

    struct DirtyFlags {
        bool file_loading_dirty = true;
        bool mesh_dirty = true;
        bool endpoints_dirty = true;
        bool bounding_cage_dirty = true;
    } dirty_flags;

    Application_State application_state = Application_State::Initial_File_Selection;

    void set_application_state(Application_State new_state) {
        application_state = new_state;
        glfwPostEmptyEvent();
    }

    std::shared_ptr<spdlog::logger> logger;

    struct LoadedVolume {
        DatFile metadata;
        VectorXui index_data;
        Eigen::VectorXf volume_data;

        GLuint volume_texture = 0;
        GLuint index_texture = 0;

        double min_value;
        double max_value;

        const Eigen::RowVector3i dims() const {
            return Eigen::RowVector3i(metadata.w, metadata.h, metadata.d);
        }

        const size_t num_voxels() const {
            return metadata.w*metadata.h*metadata.d;
        }

        void preprocess_volume_texture(std::vector<uint8_t>& byte_data);
        void load_gl_volume_texture(const std::vector<uint8_t> &byte_data);
        void load_gl_index_texture();
    };

    // Initial volume data loaded in the first screen
    LoadedVolume low_res_volume;
    LoadedVolume hi_res_volume;

    // Topological features
    struct SegmentedFeatures {
        std::vector<uint32_t> buffer_data;
        contourtree::TopologicalFeatures topological_features;
        std::vector<contourtree::Feature> features;

        std::vector<uint32_t> selected_features;
        int num_selected_features = 5;

        void recompute_feature_map();

    } segmented_features;

    struct ImageInput {
        std::string input_dir;
        std::string output_dir;
        std::string file_extension;
        std::string prefix;

        int downsample_factor = 8;
        int start_index;
        int end_index;

        std::string full_res_prefix() {
            std::string str = prefix + std::string("-") + std::to_string(start_index) + std::string("-") + std::to_string(end_index);
            return str;
        }

        std::string low_res_prefix() {
            std::string str = prefix + std::string("-") + std::to_string(start_index) + std::string("-") + std::to_string(end_index) + std::string("-") + std::to_string(downsample_factor);
            return str;
        }

        std::string low_res_path_prefx() {
            return output_dir + "/" + low_res_prefix();
        }
    } input_metadata;

    // Output of the dilation and tetrahedralization
    struct DilatedTetMesh {
        Eigen::MatrixXd TV;
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TF;
        Eigen::VectorXi connected_components;

        double dilation_radius = 3.0;
        double meshing_voxel_radius = 1.5;

        // Geodesic distances stored at each tet vertex
        Eigen::VectorXd geodesic_dists;

        void clear() {
            TV.resize(0, 0);
            TF.resize(0, 0);
            TT.resize(0, 0);
            connected_components.resize(0);
            geodesic_dists.resize(0);
        }
    } dilated_tet_mesh;

    struct SkeletonEstimationParameters {
        // Number of level sets in the skeleton
        int num_subdivisions = 100;

        // Number of smoothing iterations
        int num_smoothing_iters = 50;

        double cage_bbox_radius = 7.5;

        // Selected pairs of endpoints
        std::vector<std::pair<int, int>> endpoint_pairs;
    } skeleton_estimation_parameters;



    void load_volume_data(LoadedVolume& volume, bool load_topology);

    BoundingCage cage;

    void serialize(std::vector<char>& buffer) const;
    void deserialize(const std::vector<char>& buffer);
};

namespace igl {
namespace serialization {

template <> inline void serialize(const State& obj, std::vector<char>& buffer) {
    obj.serialize(buffer);
}
template <> inline void deserialize(State& obj, const std::vector<char>& buffer){
    obj.deserialize(buffer);
}

}
}

#endif // __FISH_DEFORMATION_STATE__
