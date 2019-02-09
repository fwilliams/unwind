#include "meshing_plugin.h"

#include "make_tet_mesh.h"
#include "make_signed_distance.h"
#include "state.h"
#include "trimesh.h"

#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include <igl/boundary_facets.h>
#include <igl/components.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/copyleft/marching_cubes.h>
#include <imgui/imgui.h>
#include <vector>
#include <vor3d/CompressedVolume.h>
#include <vor3d/VoronoiVorPower.h>

namespace {

void volume_to_dexels(const Eigen::VectorXf& scalars, Eigen::RowVector3i volume_size,
                      vor3d::CompressedVolume& dexels)
{
    const int w = volume_size[0], h = volume_size[1], d = volume_size[2];
    dexels = vor3d::CompressedVolume(Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(d, h, w), 1.0, 0);

    int start_idx = 0;
    for (int z = 0; z < d; z++) {
        for (int y = 0; y < h; y++) {
            bool outside = true;
            int seg_entry = 0;
            for (int x = 0; x < w; x++) {
                if (outside && scalars[start_idx] > 0.0) {
                    seg_entry = x;
                    outside = false;
                }
                else if (!outside && scalars[start_idx] <= 0.0) {
                    dexels.appendSegment(z, y, seg_entry, x, -1);
                    outside = true;
                }
                start_idx += 1;
            }
        }
    }

}

void dexels_to_mesh(int n_samples, const vor3d::CompressedVolume& dexels,
                    Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::vector<Eigen::Vector3d> grid_pts;
    std::vector<double> grid_vals;

    for (int x = -1; x < dexels.gridSize()[0] + 1; ++x) { // z axis
        for (int y = -1; y < dexels.gridSize()[1] + 1; ++y) {   // y axis
            for (int z = -1; z < n_samples + 1; ++z) {                // x axis
                if (x == -1 || y == -1 || z == -1 || x == dexels.gridSize()[0] || y == dexels.gridSize()[1] || z == n_samples) {
                    Eigen::Vector3d grid_ctr;
                    grid_ctr[0] = dexels.origin()[2] + (z + 0.5) * (dexels.extent()[2] / n_samples);
                    grid_ctr[1] = dexels.origin()[1] + (y + 0.5) * (dexels.extent()[1] / dexels.gridSize()[1]);
                    grid_ctr[2] = dexels.origin()[0] + (x + 0.5) * (dexels.extent()[0] / dexels.gridSize()[0]);
                    grid_pts.push_back(grid_ctr);
                    grid_vals.push_back(1);
                    continue;
                }
                Eigen::Vector3d grid_ctr;
                grid_ctr[0] = dexels.origin()[2] + (z + 0.5) * (dexels.extent()[2] / n_samples);
                grid_ctr[1] = dexels.origin()[1] + (y + 0.5) * (dexels.extent()[1] / dexels.gridSize()[1]);
                grid_ctr[2] = dexels.origin()[0] + (x + 0.5) * (dexels.extent()[0] / dexels.gridSize()[0]);

                grid_pts.push_back(grid_ctr);

                const std::vector<vor3d::Scalar>& d = dexels.at(x, y);
                int idx = -1;
                for (int i = 0; i < d.size(); i++) {
                    if (grid_ctr[0] < d[i]) {
                        idx = i - 1;
                        break;
                    }
                }

                if (idx < 0) {
                    grid_vals.push_back(1);
                    continue;
                }

                if (idx % 2 == 0) {
                    grid_vals.push_back(-1);
                }
                else {
                    grid_vals.push_back(1);
                }
            }
        }
    }

    Eigen::MatrixXd pts(grid_vals.size(), 3);
    Eigen::VectorXd vals(grid_vals.size());
    for (int i = 0; i < grid_vals.size(); i++) {
        pts.row(i) = grid_pts[i];
        vals[i] = grid_vals[i];
    }

    igl::copyleft::marching_cubes(vals, pts, n_samples + 2, dexels.gridSize()[1] + 2,
        dexels.gridSize()[0] + 2, V, F);
}

} // namespace


Meshing_Menu::Meshing_Menu(State& state) : _state(state) {}


void Meshing_Menu::initialize() {
    done_meshing = false;

    auto thread_fun = [&]() {
        is_meshing = true;
        glfwPostEmptyEvent();

        std::vector<uint32_t> feature_list = _state.fishes[_state.current_fish].feature_list;
        // The feature list used in export_selected_volume uses a zero-based indexing, we use
        // 0 for the non-feature, so we have to convert into the zero-based indexing here
        std::transform(feature_list.begin(), feature_list.end(), feature_list.begin(),
            [](uint32_t v) { return v - 1; });
        export_selected_volume(feature_list);

        dilate_volume();
        if (extracted_surface.V_fat.rows() == 0) {
            _state.logger->error("Extracted empty mesh after dilation! Something went wrong!");
            abort();
        }
        tetrahedralize_surface_mesh();
        igl::components(_state.dilated_tet_mesh.TT, _state.dilated_tet_mesh.connected_components);

        is_meshing = false;
        done_meshing = true;

        _state.logger->info("Done meshing background thread.");
        glfwPostEmptyEvent();
    };

    extracted_surface.V_thin.resize(0, 0);
    extracted_surface.F_thin.resize(0, 0);
    extracted_surface.V_fat.resize(0, 0);
    extracted_surface.F_fat.resize(0, 0);

    _state.logger->info("Starting meshing background thread...");
    bg_thread = std::thread(thread_fun);
    bg_thread.detach();
}


bool Meshing_Menu::post_draw() {
    bool ret = FishUIViewerPlugin::post_draw();

    if (is_meshing) {
        int width;
        int height;
        glfwGetWindowSize(viewer->window, &width, &height);
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiSetCond_Always);
        float w = static_cast<float>(width);
        float h = static_cast<float>(height);
        ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiSetCond_Always);
        ImGui::Begin("", nullptr,
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoTitleBar);

        ImGui::OpenPopup("Processing Fish Segments");
        ImGui::BeginPopupModal("Processing Fish Segments");
        ImGui::Text("Processing Fish Segments. Please wait as this can take a few minutes.");
        ImGui::NewLine();
        ImGui::Separator();
        if (ImGui::Button("Cancel")) {
            // TODO: Cancel button
        }
        ImGui::EndPopup();
        ImGui::End();
    }

    if (done_meshing) {
        _state.set_application_state(Application_State::EndPointSelection);
        done_meshing = false;

        glfwPostEmptyEvent();
    }

    ImGui::Render();
    return ret;
}


void Meshing_Menu::dilate_volume() {
    vor3d::CompressedVolume input;
    volume_to_dexels(skeleton_masking_volume, _state.low_res_volume.dims(), input);

    vor3d::VoronoiMorphoVorPower op = vor3d::VoronoiMorphoVorPower();
    double time_1;
    double time_2;
    vor3d::CompressedVolume output;
    op.dilation(input, output, 3, time_1, time_2);

    dexels_to_mesh(2 * _state.low_res_volume.dims()[0], output, extracted_surface.V_fat, extracted_surface.F_fat);
}


void Meshing_Menu::tetrahedralize_surface_mesh() {
    const Eigen::MatrixXd& V = extracted_surface.V_fat;
    const Eigen::MatrixXi& F = extracted_surface.F_fat;

    std::vector<Vec3i> surf_tri(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        surf_tri[i] = Vec3i(F.row(i)[0], F.row(i)[1], F.row(i)[2]);
    }

    std::vector<Vec3f> surf_x(V.rows());
    for (int i = 0; i < V.rows(); i++) {
        surf_x[i] = Vec3f(
            static_cast<float>(V.row(i)[0]),
            static_cast<float>(V.row(i)[1]),
            static_cast<float>(V.row(i)[2]));
    }

    // Compute the bounding box of the mesh
    const Eigen::RowVector3d v_min = V.colwise().minCoeff();
    Vec3f xmin(static_cast<float>(v_min[0]), static_cast<float>(v_min[1]),
        static_cast<float>(v_min[2]));

    const Eigen::RowVector3d v_max = V.colwise().maxCoeff();
    Vec3f xmax(static_cast<float>(v_max[0]), static_cast<float>(v_max[1]),
        static_cast<float>(v_max[2]));

    // Build triangle mesh data structure
    TriMesh trimesh(surf_x, surf_tri);

    // Make the level set

    // Determining dimensions of voxel grid.
    // Round up to ensure voxel grid completely contains bounding box.
    // Also add padding of 2 grid points around the bounding box.
    // NOTE: We add 5 here so as to add 4 grid points of padding, as well as
    // 1 grid point at the maximal boundary of the bounding box
    // ie: (xmax-xmin)/dx + 1 grid points to cover one axis of the bounding box
    constexpr float dx = 0.8f; // TODO: Play with this a little bit
    Vec3f origin = xmin - Vec3f(2 * dx);
    int ni = static_cast<int>(std::ceil((xmax[0] - xmin[0]) / dx) + 5);
    int nj = static_cast<int>(std::ceil((xmax[1] - xmin[1]) / dx) + 5);
    int nk = static_cast<int>(std::ceil((xmax[2] - xmin[2]) / dx) + 5);

    SDF sdf(origin, dx, ni, nj, nk); // Initialize signed distance field.
    
    _state.logger->info("making {}x{}x{} level set", ni, nj, nk);
    make_signed_distance(surf_tri, surf_x, sdf);

    // Then the tet mesh
    TetMesh mesh;

    // Make tet mesh without features
    const bool optimize = false;
    const bool intermediate = false;
    const bool unsafe = false;
    make_tet_mesh(mesh, sdf, optimize, intermediate, unsafe);

    _state.dilated_tet_mesh.TV.resize(mesh.verts().size(), 3);
    for (int i = 0; i < mesh.verts().size(); i++) {
        _state.dilated_tet_mesh.TV.row(i) =
            Eigen::Vector3d(mesh.verts()[i][0], mesh.verts()[i][1], mesh.verts()[i][2]);
    }
    _state.dilated_tet_mesh.TT.resize(mesh.tets().size(), 4);
    for (int i = 0; i < mesh.tets().size(); i++) {
        _state.dilated_tet_mesh.TT.row(i) =
            Eigen::Vector4i(mesh.tets()[i][0], mesh.tets()[i][2], mesh.tets()[i][1], mesh.tets()[i][3]);
    }

    igl::boundary_facets(_state.dilated_tet_mesh.TT, _state.dilated_tet_mesh.TF);
}


void Meshing_Menu::extract_surface_mesh() {
    const Eigen::RowVector3i volume_dims = _state.low_res_volume.dims();
    const int w = volume_dims[0], h = volume_dims[1], d = volume_dims[2];

    // Grid positions and scalar values
    Eigen::MatrixXd GP((w + 2)*(h + 2)*(d + 2), 3);
    Eigen::VectorXd SV(GP.rows());

    int readcount = 0;
    int appendcount = 0;
    for (int zi = 0; zi < d + 2; zi++) {
        for (int yi = 0; yi < h + 2; yi++) {
            for (int xi = 0; xi < w + 2; xi++) {
                if (xi == 0 || yi == 0 || zi == 0 ||
                    xi == (w + 1) || yi == (h + 1) || zi == (d + 1)) {
                    SV[readcount] = -1.0;
                }
                else {
                    SV[readcount] = skeleton_masking_volume[appendcount];
                    appendcount += 1;
                }
                GP.row(readcount) = Eigen::RowVector3d(xi, yi, zi);
                readcount += 1;
            }
        }
    }

    igl::copyleft::marching_cubes(SV, GP, w + 2, h + 2, d + 2,
        extracted_surface.V_thin,
        extracted_surface.F_thin);


    if (extracted_surface.V_thin.rows() < 4 || extracted_surface.F_thin.rows() < 4) {
        _state.logger->error("Extracted mesh has too few tets, aborting!");
        exit(EXIT_FAILURE);
    }
}


void Meshing_Menu::export_selected_volume(const std::vector<uint32_t>& feature_list)
{
    _state.logger->debug("Feature list size: {}", feature_list.size());
    skeleton_masking_volume.resize(_state.low_res_volume.volume_data.size());

    std::cout << _state.num_selected_features << " vs " << _state.fishes[_state.current_fish].feature_list.size() << std::endl;
    std::vector<contourtree::Feature> features = _state.topological_features.getFeatures(_state.num_selected_features, 0.f);

    std::vector<uint32_t> good_arcs;
    for (uint32_t f : feature_list) {
        _state.logger->debug("Feature: {}", f);
        _state.logger->debug("Feature arcs size: {}", features[f].arcs.size());
        good_arcs.insert(good_arcs.end(), features[f].arcs.begin(), features[f].arcs.end());
        _state.logger->debug("Good arcs size: {}", good_arcs.size());
    }
    std::sort(good_arcs.begin(), good_arcs.end());

    for (int i = 0; i < skeleton_masking_volume.size(); ++i) {
        unsigned int idx = _state.low_res_volume.index_data[i];
        if (std::binary_search(good_arcs.begin(), good_arcs.end(), idx)) {
            skeleton_masking_volume[i] = 1.0;
        }
        else {
            skeleton_masking_volume[i] = -1.0;
        }
    }
}
