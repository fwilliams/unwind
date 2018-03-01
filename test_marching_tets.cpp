#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/per_face_normals.h>

#include <iostream>
#include <utility>

//#include "tetrahedralize.h"
#include "marching_tets.h"

typedef igl::opengl::glfw::Viewer Viewer;

void make_single_tet(Eigen::MatrixXd& TV, Eigen::MatrixXd& TN, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
    const double SQRT_3 = sqrt(3);
    TV.resize(4, 3);
    TF.resize(4, 3);
    TT.resize(1, 4);

    TV << 0.0, 0.0, 0.0,
          1.0, 0.0, 0.0,
          0.5, SQRT_3/2,  SQRT_3/2 - 1/SQRT_3,
          0.5, 0.0, SQRT_3/2;
    Eigen::RowVector3d ctr = TV.colwise().sum() / 4;
    for (int i = 0; i < 4; i++) {
        TV.row(i) -= ctr;
    }


    TF << 0, 1, 2,
          3, 0, 2,
          1, 3, 2,
          0, 3, 1;

    TT << 0, 1, 2, 3;

    Eigen::MatrixXd Nf;
    igl::per_face_normals(TV, TF, Nf);
    TN.resize(Nf.rows()*3, 3);
    for (int i = 0; i < Nf.rows(); i++) {
        TN.row(i*3) = -Nf.row(i);
        TN.row(i*3+1) = -Nf.row(i);
        TN.row(i*3+2) = -Nf.row(i);
    }
}

void mt_test_case(Viewer& viewer, int i, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                  Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    const int I =  1;
    const int O = -1;
    Eigen::MatrixXd isovals(16, 4);
    isovals << O, O, O, O, // 0a
               I, O, O, O, // 1a
               O, I, O, O, // 1b
               O, O, I, O, // 1c
               O, O, O, I, // 1d
               I, O, I, O, // 2a
               I, I, O, O, // 2b
               I, O, O, I, // 2c
               O, I, I, O, // 2d
               O, O, I, I, // 2e
               O, I, O, I, // 2f
               O, I, I, I, // 3a
               I, O, I, I, // 3b
               I, I, O, I, // 3c
               I, I, I, O, // 3d
               I, I, I, I; // 4a

    marching_tets(TV, isovals.row(i), TT, V, F);

    Eigen::MatrixXd e1(6, 3), e2(6, 3);
    e1.row(0) = TV.row(0); e2.row(0) = TV.row(1);
    e1.row(1) = TV.row(0); e2.row(1) = TV.row(2);
    e1.row(2) = TV.row(0); e2.row(2) = TV.row(3);
    e1.row(3) = TV.row(1); e2.row(3) = TV.row(2);
    e1.row(4) = TV.row(1); e2.row(4) = TV.row(3);
    e1.row(5) = TV.row(2); e2.row(5) = TV.row(3);

    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().add_edges(e1, e2, Eigen::RowVector3d(0.1, 0.1, 0.1));
    viewer.data().line_width = 2.0;
    viewer.data().point_size = 5.0;

    Eigen::MatrixXd colors(4, 3);
    for (int j = 0; j < 4; j++) {
        Eigen::RowVector3d color;
        if (isovals(i, j) > 0.0) {
            color = Eigen::RowVector3d(0.0, 1.0, 0.0);
        } else {
            color = Eigen::RowVector3d(1.0, 0.0, 0.0);
        }
        colors.row(j) = color;
    }
    viewer.data().add_points(TV, colors);
}

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace std;

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    Eigen::MatrixXd TV, TN, V;
    Eigen::MatrixXi TF, TT, F;
    make_single_tet(TV, TN, TF, TT);
    viewer.data().set_mesh(TV, TF);
    viewer.data().set_normals(TN);

    const int NUM_TEST_CASES = 16;
    unsigned test_case = NUM_TEST_CASES + 1;

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers)->bool {
        switch(key) {
        case 'N':
            if (test_case > NUM_TEST_CASES) {
                test_case = 0;
                mt_test_case(viewer, test_case, TV, TT, V, F);
            } else {
                test_case = (test_case + 1) % NUM_TEST_CASES;
                mt_test_case(viewer, test_case, TV, TT, V, F);
            }
            break;
        case 'P':
            if (test_case > NUM_TEST_CASES) {
                test_case = 0;
                mt_test_case(viewer, test_case, TV, TT, V, F);
            } else {
                test_case = (test_case - 1) % NUM_TEST_CASES;
                mt_test_case(viewer, test_case, TV, TT, V, F);
            }
            break;
        case 'R':
            viewer.data().set_mesh(TV, TF);
            viewer.data().set_normals(TN);
            test_case = NUM_TEST_CASES + 1;
            break;
        }
        return false;
    };

    cout << "Press [r] to reset." << endl;
    cout << "Press [n] to go to the next test case." << endl;
    cout << "Press [p] to go to the previous test case." << endl;
    return viewer.launch();
}

