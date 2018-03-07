#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/writeOFF.h>
#include <igl/colormap.h>
#include <igl/marching_tets.h>

#include <iostream>
#include <utility>

#include "tetrahedralize.h"


typedef igl::opengl::glfw::Viewer Viewer;

void visualize_tets(Viewer& viewer, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd& isovals) {
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

    Eigen::MatrixXd C;
    igl::colormap(igl::COLOR_MAP_TYPE_MAGMA, isovals, false, C);

    viewer.data().add_points(TV, C);
    viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.1, 0.1, 0.1));
}

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace std;

    Eigen::MatrixXd TV, V, VMT;
    Eigen::MatrixXi TF, TT, F, FMT;
    Eigen::VectorXd isovals;

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    const std::string mesh_name = "./meshes/icosphere2.off";
    bool render_tet = false;

    // Load a mesh in OFF format
    igl::readOFF(mesh_name, V, F);

    Eigen::RowVector3d barycenter = V.colwise().sum() / V.rows();
    V = V.rowwise() - barycenter;

    Eigen::RowVector3d bb = V.colwise().maxCoeff() - V.colwise().minCoeff();
    double bb_size = bb.norm();
    V *= 100/bb_size;

    cout << "rescaled bb: " << V.colwise().maxCoeff() - V.colwise().minCoeff() << endl;

    viewer.data().set_mesh(V, F);

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers)->bool {
        cout << key << endl;
        switch (key) {
        case 'T':
            render_tet = !render_tet;
            if (render_tet) {
                if (TT.rows() == 0) {
                    tetrahedralize_mesh(V, F,
                                        30,  // facet_angle
                                        0.25, // facet_size
                                        0.5, // facet_distance
                                        2,   // cell_radius_edge_ratio
                                        0.1, // cell_size
                                        TV, TF, TT);
                    isovals.resize(TV.rows());
                    for (int i = 0; i < TV.rows(); i++) {
                        isovals[i] = TV(i, 0); // TV.row(i).norm();
                    }
                }
                cout << "isovalue avg: " << isovals.sum() / isovals.size() << endl;
                igl::marching_tets(TV, TT, isovals, isovals.sum() / isovals.size(), VMT, FMT);

                viewer.data().clear();
                viewer.data().point_size = 4.0;
                viewer.data().show_faces = true;
                visualize_tets(viewer, TV, TT, isovals / (isovals.maxCoeff() - isovals.minCoeff()));
                viewer.data().set_mesh(VMT, FMT);
            } else {
                viewer.data().clear();
                viewer.data().set_mesh(V, F);
            }
            break;
        }
        return false;
    };
    cout << "Press [r] to reset." << endl;;
    return viewer.launch();
}


