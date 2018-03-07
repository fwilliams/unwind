#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/colormap.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/harmonic.h>
#include <igl/readOFF.h>
#include <igl/marching_tets.h>

//#include <igl/readOBJ.h>
//#include <igl/readMEDIT.h>
//#include <igl/components.h>

#include <iostream>
#include <utility>
#include <array>
#include "tetrahedralize.h"


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


// Visualize the tet mesh as a wireframe
void visualize_tet_wireframe(igl::opengl::glfw::Viewer& viewer,
                             const Eigen::MatrixXd& TV,
                             const Eigen::MatrixXi& TT,
                             const Eigen::VectorXd& isovals)
{
  // Make a black line for each edge in the tet mesh which we'll draw
  std::vector<std::pair<int, int>> edges;
  for (int i = 0; i < TT.rows(); i++)
  {
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
  for (int i = 0; i < edges.size(); i++)
  {
    v1.row(i) = TV.row(edges[i].first);
    v2.row(i) = TV.row(edges[i].second);
  }

  // Normalize the isovalues between 0 and 1 for the colormap
  Eigen::MatrixXd C;
  const double isoval_min = isovals.minCoeff();
  const double isoval_max = isovals.maxCoeff();
  const double isoval_spread = isoval_max - isoval_min;
  const std::size_t n_isovals = isovals.size();
  Eigen::VectorXd isovals_normalized =
    (isovals - isoval_min * Eigen::VectorXd::Ones(n_isovals)) / isoval_spread;

  // Draw colored vertices of tet mesh based on their isovalue and black
  // lines connecting the vertices
  igl::colormap(igl::COLOR_MAP_TYPE_MAGMA, isovals_normalized, false, C);
  viewer.data().point_size = 5.0;
  viewer.data().add_points(TV, C);
  viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.1, 0.1, 0.1));
}

int main(int argc, char *argv[]) {
  using namespace Eigen;
  using namespace std;

  Eigen::MatrixXd TV, V, isoV;
  Eigen::MatrixXi TF, TT, F, isoF;
  Eigen::VectorXd W;

  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);

  // Load a mesh in OFF format
  // igl::readOFF("../p-tapinosoma-100k.off", V, F);
  // igl::readOBJ("./meshes/armadillo.obj", V, F);
  // igl::readOFF("./meshes/p-tapinosoma-100k-watertight.off", V, F);
  // igl::readOFF("./meshes/fertility.off", V, F);
  igl::readOFF("./meshes/ball.off", V, F);
  cout << "Input surface mesh has " << V.rows() << " vertices and " << F.rows() << " faces" << endl;
  cout << "Loaded mesh bounding box: " <<
          V.colwise().maxCoeff() - V.colwise().minCoeff() << endl;
  tetrahedralize_mesh(V, F,
                      5, // Facet angle
                      2, // Facet size
                      0.1, // Facet distance
                      3, // Cell Radius Edge Ratio
                      3, // Cell size
                      TV, TF, TT);
//  igl::readMEDIT("./out2.mesh", TV, TF, TT);

  viewer.data().clear();
  viewer.data().set_mesh(TV, TF);

  array<int, 2> selected_end_coords{{-1, -1}};
  int current_idx = 0;
  bool done_harmonic = false;
  double level_set = 0.0;
  double increment = 0.005;
  viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer,
                                 unsigned char key, int modifiers) -> bool
  {
    switch (key)
    {
    case 'R':
      done_harmonic = false;
      current_idx = 0;
      viewer.data().clear();
      viewer.data().set_mesh(TV, TF);
      break;
    case 'N':
      if (W.rows() != TV.rows())
      {
        cerr << "No isofunction defined yet!" << endl;
      }
      else
      {
        level_set += increment;
        if (level_set > 1.0) {
          level_set = 0.0;
        }
        cout << "Showing level set for isovalue " << level_set << endl;
        igl::marching_tets(TV, TT, W, level_set, isoV, isoF);
        viewer.data().clear();
        visualize_tet_wireframe(viewer, TV, TT, W);
        viewer.data().set_mesh(isoV, isoF);
      }
      break;
    case 'P':
      if (W.rows() != TV.rows())
      {
        cerr << "No isofunction defined yet!" << endl;
      }
      else
      {
        level_set -= increment;
        if (level_set < 0.0) {
          level_set = 0.0;
        }
        cout << "Showing level set for isovalue " << level_set << endl;
        igl::marching_tets(TV, TT, W, level_set, isoV, isoF);
        viewer.data().clear();
        visualize_tet_wireframe(viewer, TV, TT, W);
        viewer.data().set_mesh(isoV, isoF);
      }
      break;
    }
  };

  viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int, int)->bool {
    int fid;
    Eigen::Vector3f bc; // Barycentric coordinates of the click point on the face

    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view * viewer.core.model,
                                viewer.core.proj, viewer.core.viewport, TV, TF, fid, bc))
    {
      if (current_idx  <= 1)
      {
        int max;
        bc.maxCoeff(&max);
        int vid = TF(fid, max);
        viewer.data().point_size = 5.0;
        viewer.data().add_points(TV.row(vid), Eigen::RowVector3d(1.0, 0.0, 0.0));

        selected_end_coords[current_idx] = vid;

        viewer.data().point_size = 7.0;
        Eigen::RowVector3d color;
        if (current_idx == 0) { color = Eigen::RowVector3d(0.0, 1.0, 0.0); } else { cout << "GREEN" << endl; color = Eigen::RowVector3d(1.0, 0.0, 0.0); }
        viewer.data().add_points(TV.row(vid), color);

        current_idx += 1;
      }
      else if (!done_harmonic)
      {
        Eigen::MatrixXi constraint_indices;
        Eigen::MatrixXd constraint_values;
        constraint_indices.resize(2, 1);
        constraint_values.resize(2, 1);
        constraint_indices(0, 0) = selected_end_coords[0];
        constraint_indices(1, 0) = selected_end_coords[1];
        constraint_values(0, 0) = 1.0;
        constraint_values(1, 0) = 0.0;

        igl::harmonic(TV, TT, constraint_indices, constraint_values, 1, W);
        done_harmonic = true;

        viewer.data().clear();
        viewer.data().set_mesh(TV, TF);
        visualize_tet_wireframe(viewer, TV, TT, W);
      }
    }
    return false;
  };

  cout << "Press [r] to reset." << endl;;
  return viewer.launch();
}

