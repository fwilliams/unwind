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

constexpr const char* FISH_LOGGER_NAME = "fish_deformation_logger";

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
    struct FeatureSegmentation {
        std::vector<uint32_t> buffer_data;
        contourtree::TopologicalFeatures topological_features;
        std::vector<contourtree::Feature> features;
        std::vector<uint32_t> selected_features;
        int num_selected_features = 5;

        void recompute_feature_map() {
            selected_features.clear();
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

    // Output of the dilation and tetrahedralization
    struct DilatedTetMesh {
        Eigen::MatrixXd TV;
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TF;
        Eigen::VectorXi connected_components;

        double dilation_radius = 3.0;

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

    // Selected pairs of endpoints
    std::vector<std::pair<int, int>> endpoint_pairs;

    BoundingCage cage;
};

#endif // __FISH_DEFORMATION_STATE__
