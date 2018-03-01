#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/colormap.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/harmonic.h>
#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/writeOFF.h>
#include <igl/components.h>

#include <iostream>
#include <utility>

#include "tetrahedralize.h"
#include "marching_tets.h"

typedef igl::opengl::glfw::Viewer Viewer;


void plotMeshDistance(Viewer& viewer, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::VectorXd& d, const double strip_size ) {
    // Rescale the function depending on the strip size
    Eigen::VectorXd f = (d/strip_size);

    // The function should be 1 on each integer coordinate
    f = (f*M_PI).array().sin().abs();

    // Compute per-vertex colors
    Eigen::MatrixXd C;
    igl::colormap(igl::COLOR_MAP_TYPE_JET, f, false, C);

    // Plot the mesh
    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
}

void color_components(const Eigen::MatrixXi& components, Eigen::MatrixXd& colors) {
    colors.resize(components.rows(), 3); for (int i = 0; i < components.rows(); i++) {
        if (components(i, 0) == 0) {
            colors.row(i) = Eigen::RowVector3d(1.0, 0.0, 0.0);
        } else if (components(i, 0) == 1) {
            colors.row(i) = Eigen::RowVector3d(0.0, 1.0, 0.0);
        } else if (components(i, 0) == 2) {
            colors.row(i) = Eigen::RowVector3d(0.0, 0.0, 1.0);
        } else {
            colors.row(i) = Eigen::RowVector3d(0.5, 0.5, 0.5);
        }
    }
}

void visualize_tets(Viewer& viewer, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT) {
    std::vector<std::pair<int, int>> edges;
    for (int i = 0; i < TT.rows(); i++) {
        int tf1 = TT(i, 0);
        int tf2 = TT(i, 1);
        int tf3 = TT(i, 2);
        int tf4 = TT(i, 2);
        edges.push_back(std::make_pair(tf1, tf2));
        edges.push_back(std::make_pair(tf1, tf3));
        edges.push_back(std::make_pair(tf1, tf4));
        edges.push_back(std::make_pair(tf2, tf3));
        edges.push_back(std::make_pair(tf2, tf4));
        edges.push_back(std::make_pair(tf3, tf4));
    }

    Eigen::MatrixXd v1(edges.size(), 3), v2(edges.size(), 3);
    for (int i = 0; i < edges.size(); i++) {
        v1.row(i) = TV.row(edges[i].first);
        v2.row(i) = TV.row(edges[i].second);
    }
    viewer.data().add_points(TV, Eigen::RowVector3d(0.0, 1.0, 0.0));
    viewer.data().point_size = 1.0;
    viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.5, 0.5, 0.5));
}

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace std;

    Eigen::MatrixXd TV, V;
    Eigen::MatrixXi TF, TT, F;

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    // Load a mesh in OFF format
    // igl::readOFF("../p-tapinosoma-100k.off", V, F);
    // igl::readOBJ("../armadillo.obj", V, F);
//    igl::readOBJ("./meshes/sphere.obj", V, F);
//    igl::writeOFF("./meshes/sphere.off", V, F);
    igl::readOFF("./meshes/sphere.off", V, F);
    // igl::readOFF("./meshes/p-tapinosoma-100k-watertight.off", V, F);
    cout << V.colwise().maxCoeff() - V.colwise().minCoeff() << endl;
    tetrahedralize_mesh("./meshes/sphere.off",
                        30,  // facet_angle
                        0.05, // facet_size
                        0.0125, // facet_distance
                        2,   // cell_radius_edge_ratio
                        0.05, // cell_size
                        TV, TF, TT);
    viewer.data().set_mesh(V, F);

    Eigen::MatrixXd colors;
    Eigen::MatrixXi components;
    igl::components(F, components);
    color_components(components, colors);
    cout << components.rows() << " " << components.cols() << endl;
    cout << colors.rows() << " " << colors.cols() << endl;
    cout << TV.rows() << " " << TV.cols() << endl;
    viewer.data().set_colors(colors);
    visualize_tets(viewer, TV, TT);

    int selected_end_coords[2] = {-1, -1};
    int current_idx = 0;
    bool done_harmonic = false;

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers)->bool {
        switch(key) {
        case 'R':
            std::cout << "RESET!" << std::endl;
            current_idx = 0;
            done_harmonic = false;
            color_components(components, colors);
            viewer.data().set_colors(colors);
            break;
        }

        return false;
    };

    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int, int)->bool {
        int fid;
        Eigen::Vector3f bc;
        // Cast a ray in the view direction starting from the mouse position
        double x = viewer.current_mouse_x;
        double y = viewer.core.viewport(3) - viewer.current_mouse_y;
        if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view * viewer.core.model,
                                    viewer.core.proj, viewer.core.viewport, V, F, fid, bc))
        {
            if (current_idx > 1 && !done_harmonic) {
                // Biharmonic operator
                Eigen::MatrixXi constraint_indices;
                Eigen::MatrixXd constraint_values;
                constraint_indices.resize(2, 1);
                constraint_values.resize(2, 1);

                constraint_indices(0, 0) = selected_end_coords[0];
                constraint_indices(1, 0) = selected_end_coords[1];
                constraint_values(0, 0) = 1.0;
                constraint_values(1, 0) = -1.0;

                Eigen::MatrixXd W;
                igl::harmonic(V, F, constraint_indices, constraint_values, 1, W);
                plotMeshDistance(viewer, V, F, W, 0.05);
                done_harmonic = true;
            } else if(current_idx <= 1) {
                int max;
                bc.maxCoeff(&max);
                int vid = F(fid,max);

                selected_end_coords[current_idx] = vid;
                current_idx += 1;

                color_components(components, colors);
                for (int i = 0; i < current_idx; i++) {
                    colors.row(selected_end_coords[i]) = Eigen::RowVector3d(1.0, 1.0, 1.0);
                }
                viewer.data().set_colors(colors);
            }
        }
        return false;
    };

    cout << "Press [r] to reset." << endl;;
    return viewer.launch();
}

