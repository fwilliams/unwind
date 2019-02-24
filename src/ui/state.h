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

        GLuint volume_texture;
        GLuint index_texture;

        double min_value;
        double max_value;

        const Eigen::RowVector3i dims() const {
            return Eigen::RowVector3i(metadata.w, metadata.h, metadata.d);
        }

        const size_t num_voxels() const {
            return metadata.w*metadata.h*metadata.d;
        }
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

        void recompute_feature_map() {
            selected_features.clear();
            buffer_data.clear();
            features = topological_features.getFeatures(num_selected_features, 0.f);

            uint32_t size = topological_features.ctdata.noArcs;
            buffer_data.resize(size + 1 + 1, static_cast<uint32_t>(-1));
            buffer_data[0] = static_cast<uint32_t>(features.size());
            for (size_t i = 0; i < features.size(); ++i) {
                for (uint32_t j : features[i].arcs) {
                    // +1 since the first value of the vector contains the number of features
                    buffer_data[j + 1] = static_cast<uint32_t>(i);
                }
            }
        }
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



    void preprocess_lowres_volume_texture(std::vector<uint8_t>& byte_data) {
        // Pre-load and normalize data for the low res volume GL texture
        const Eigen::RowVector3i volume_dims = low_res_volume.dims();
        const int size = volume_dims[0]*volume_dims[1]*volume_dims[2];
        byte_data.clear();
        byte_data.resize(size);
        const double min_value = low_res_volume.min_value;
        const double value_range = low_res_volume.max_value - low_res_volume.min_value;
        float* texture_data = low_res_volume.volume_data.data();
        std::transform(
            texture_data,
            texture_data + size,
            byte_data.begin(),
            [min_value, value_range](double d) {
                return static_cast<uint8_t>(((d - min_value)/value_range) * std::numeric_limits<uint8_t>::max());
            }
        );

    }

    void load_data_from_filesystem() {

        // Load the low-res volume data
        low_res_volume.metadata = DatFile(input_metadata.low_res_path_prefx() + ".dat", logger);
        load_rawfile(input_metadata.low_res_path_prefx() + ".raw", low_res_volume.dims(),
                     low_res_volume.volume_data, logger, true /* normalize */);
        low_res_volume.max_value = low_res_volume.volume_data.maxCoeff();
        low_res_volume.min_value = low_res_volume.volume_data.minCoeff();

        // Compute the topological features
        Eigen::Vector3i lrv = low_res_volume.dims();
        preProcessing(input_metadata.low_res_path_prefx(), lrv[0], lrv[1], lrv[2]);
        segmented_features.topological_features.loadData(input_metadata.low_res_path_prefx());
        segmented_features.recompute_feature_map();

        // Load the low-res index data
        const size_t num_bytes = low_res_volume.num_voxels() * sizeof(uint32_t);
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        file.open(input_metadata.low_res_path_prefx() + ".part.raw", std::ifstream::binary);
        typedef decltype(low_res_volume.index_data) IndexType;
        low_res_volume.index_data.resize(num_bytes / sizeof(IndexType::Scalar));
        file.read(reinterpret_cast<char*>(low_res_volume.index_data.data()), num_bytes);
    }

    BoundingCage cage;
};

#endif // __FISH_DEFORMATION_STATE__
