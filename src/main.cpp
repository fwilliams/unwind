#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/colormap.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/harmonic.h>
#include <igl/readOFF.h>
#include <igl/marching_tets.h>
#include <igl/directed_edge_parents.h>
#include <igl/boundary_conditions.h>
#include <igl/bbw.h>
#include <igl/normalize_row_sums.h>
#include <igl/project_isometrically_to_plane.h>

#include <iostream>
#include <utility>
#include <array>

#include "MshLoader.h"

typedef igl::opengl::glfw::Viewer Viewer;

void load_yixin_tetmesh(const std::string& filename, Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
  using namespace std;
  MshLoader vol_loader(filename);
  assert(vol_loader.m_nodes_per_element == 4);
  assert(vol_loader.m_data_size == 8);

  int tv_rows = vol_loader.m_nodes.rows() / 3;
  int tt_rows = vol_loader.m_elements.rows() / vol_loader.m_nodes_per_element;

  TT.resize(tt_rows, vol_loader.m_nodes_per_element);
  TV.resize(tv_rows, 3);

  std::vector<array<int, 3>> tris;

  int vcount = 0;
  for (int i = 0; i < vol_loader.m_nodes.rows(); i += 3) {
    TV.row(vcount++) = Eigen::RowVector3d(vol_loader.m_nodes[i], vol_loader.m_nodes[i+2], vol_loader.m_nodes[i+1]);
  }
  int tcount = 0;
  for (int i = 0; i < vol_loader.m_elements.rows(); i += vol_loader.m_nodes_per_element) {
    const int e1 = vol_loader.m_elements[i];
    const int e2 = vol_loader.m_elements[i+1];
    const int e3 = vol_loader.m_elements[i+2];
    const int e4 = vol_loader.m_elements[i+3];
    TT.row(tcount++) = Eigen::RowVector4i(e1, e2, e3, e4);
    array<int, 3> t1, t2, t3, t4;
    t1 = array<int, 3>{{ e1, e2, e3 }};
    t2 = array<int, 3>{{ e1, e2, e3 }};
    t3 = array<int, 3>{{ e2, e3, e4 }};
    t4 = array<int, 3>{{ e1, e3, e4 }};
    sort(t1.begin(), t1.end());
    sort(t2.begin(), t2.end());
    sort(t3.begin(), t3.end());
    sort(t4.begin(), t4.end());
    tris.push_back(t1);
    tris.push_back(t2);
    tris.push_back(t3);
    tris.push_back(t4);
  }

  int fcount;
  TF.resize(tris.size(), 3);
  sort(tris.begin(), tris.end());
  for (int i = 0; i < TF.rows();) {
    int v1 = tris[i][0], v2 = tris[i][1], v3 = tris[i][2];
    int count = 0;
    while (v1 == tris[i][0] && v2 == tris[i][1] && v3 == tris[i][2]) {
      i += 1;
      count += 1;
    }
    if (count == 1) {
      TF.row(fcount++) = Eigen::RowVector3i(v1, v2, v3);
    }
  }

  TF.conservativeResize(fcount, 3);
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


void extract_skeleton(int num_samples, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, const Eigen::VectorXd isovals, Eigen::MatrixXd& SV) {
  SV.resize(num_samples, 3);
  Eigen::MatrixXd LV;
  Eigen::MatrixXi LF;
  int vcount = 0;
  for(int i = 1; i < num_samples; i++) {
    double isovalue = i * (1.0/num_samples);
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
    if (LV.rows() == 0) {
      continue;
    }
    Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();
    SV.row(vcount++) = C;
  }

  SV.conservativeResize(vcount, 3);
}

void draw_skeleton(Viewer& viewer, const Eigen::MatrixXd& SV) {
  Eigen::MatrixXd v1(SV.rows()-1, 3), v2(SV.rows()-1, 3);
  for (int i = 0; i < SV.rows()-1; i++) {
    v1.row(i) = SV.row(i);
    v2.row(i) = SV.row(i+1);
  }
  viewer.data().add_edges(v1, v2, Eigen::RowVector3d(0.8, 0.1, 0.4));
  viewer.data().add_points(SV, Eigen::RowVector3d(1.0, 0.0, 0.0));
}

bool bbw(const Eigen::MatrixXd& SV, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, Eigen::MatrixXd& W) {
 // Need to construct
  Eigen::MatrixXi BE(SV.rows() - 1, 2);

  for (int i = 0; i < SV.rows()-1; i++) {
    BE.row(i) = Eigen::RowVector2i(i, i+1);
  }

  Eigen::VectorXi b;
  Eigen::MatrixXd bc;
  igl::boundary_conditions(TV, TT, SV, Eigen::VectorXi(), BE, Eigen::MatrixXi(), b, bc);
  igl::BBWData bbw_data;
  //  bbw_data.active_set_params.max_iter = 3;
  bbw_data.verbosity = 2;
  return igl::bbw(TV, TT, b, bc, bbw_data, W);
}

int main(int argc, char *argv[]) {
  using namespace Eigen;
  using namespace std;

  Eigen::MatrixXd TV, isoV, SV, W;
  Eigen::MatrixXi TF, TT, isoF;
  Eigen::VectorXd isovals;

  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);

  // Load a mesh in OFF format

  load_yixin_tetmesh("outReoriented_.msh", TV, TF, TT);
  viewer.data().clear();
  viewer.data().set_mesh(TV, TF);

  array<int, 2> selected_end_coords{{-1, -1}};
  int current_idx = 0;
  bool done_harmonic = false;
  double level_set = 0.0;
  double increment = 0.05;
  int current_weight = 0;

  viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer,
                                 unsigned char key, int modifiers) -> bool {
    switch (key) {
    case 'R':
      done_harmonic = false;
      current_idx = 0;
      viewer.data().clear();
      viewer.data().set_mesh(TV, TF);
      break;
    case 'N':
      if (isovals.rows() != TV.rows()) {
        cerr << "No isofunction defined yet!" << endl;
      } else {
        level_set += increment;
        if (level_set > 1.0) {
          level_set = 0.0;
        }
        cout << "Showing level set for isovalue " << level_set << endl;
        igl::marching_tets(TV, TT, isovals, level_set, isoV, isoF);
        viewer.data().clear();
        visualize_tet_wireframe(viewer, TV, TT, isovals);
        viewer.data().set_mesh(isoV, isoF);
      }
      break;
    case 'P':
      if (isovals.rows() != TV.rows()) {
        cerr << "No isofunction defined yet!" << endl;
      } else {
        level_set -= increment;
        if (level_set < 0.0) {
          level_set = 0.0;
        }
        cout << "Showing level set for isovalue " << level_set << endl;
        igl::marching_tets(TV, TT, isovals, level_set, isoV, isoF);
        viewer.data().clear();
        visualize_tet_wireframe(viewer, TV, TT, isovals);
        viewer.data().set_mesh(isoV, isoF);
      }
      break;
    case 'S':
      if (isovals.rows() != TV.rows()) {
        cerr << "No isofunction defined yet!" << endl;
      } else {
        extract_skeleton(25, TV, TT, isovals, SV);
        viewer.data().clear();
        viewer.data().set_mesh(TV, TF);
        viewer.data().show_faces = false;
        draw_skeleton(viewer, SV);
      }
      break;
    case 'B':
      bbw(SV, TV, TT, W);
      for (int i = 0; i < W.rows(); i++) {
        double sum = W.row(i).sum();
        if (fabs(sum) < 1e-15) {
          cout << "Row " << i << " sums to zero..." << endl;
        } else {
          W.row(i) /= sum;
        }
      }
      cout << W.rows() << " x " << W.cols() << endl;
      break;
    case '.':
      if (W.cols() != 0) {
        current_weight = (current_weight + 1 % W.cols());
        viewer.data().clear();
        viewer.data().set_mesh(TV, TF);
        viewer.data().show_faces = false;
        visualize_tet_wireframe(viewer, TV, TT, W.col(current_weight));
        draw_skeleton(viewer, SV);
      }
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

        igl::harmonic(TV, TT, constraint_indices, constraint_values, 1, isovals);
        done_harmonic = true;

        viewer.data().clear();
        viewer.data().set_mesh(TV, TF);
        visualize_tet_wireframe(viewer, TV, TT, isovals);
      }
    }
    return false;
  };

  cout << "Press [r] to reset." << endl;;
  return viewer.launch();
}

