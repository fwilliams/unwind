#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/colormap.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/harmonic.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <igl/marching_tets.h>
#include <igl/components.h>
#include <igl/directed_edge_parents.h>
#include <igl/boundary_conditions.h>
#include <igl/bbw.h>
#include <igl/slim.h>
#include <igl/copyleft/marching_cubes.h>

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
  std::vector<std::pair<int, int>> deduped_edges;
  std::sort(edges.begin(), edges.end());
  for (int i = 0; i < edges.size();) {
    int v1 = edges[i].first;
    int v2 = edges[i].second;
    deduped_edges.push_back(edges[i]);
    while (v1 == edges[i].first && v2 == edges[i].second) {
      i += 1;
    }
  }

  Eigen::MatrixXd v1(deduped_edges.size(), 3), v2(deduped_edges.size(), 3);
  for (int i = 0; i < deduped_edges.size(); i++)
  {
    v1.row(i) = TV.row(deduped_edges[i].first);
    v2.row(i) = TV.row(deduped_edges[i].second);
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


void extract_skeleton(int num_samples, const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT,
                      const Eigen::VectorXd isovals, Eigen::MatrixXd& SV, Eigen::VectorXi& sk_verts) {
  using namespace std;

  SV.resize(num_samples, 3);
  Eigen::MatrixXd LV;
  Eigen::MatrixXi LF;
  sk_verts.resize(num_samples);
  int vcount = 0;
  for(int i = 1; i < num_samples; i++) {
    double isovalue = i * (1.0/num_samples);
    igl::marching_tets(TV, TT, isovals, isovalue, LV, LF);
    if (LV.rows() == 0) {
      continue;
    }
    Eigen::RowVector3d C = LV.colwise().sum() / LV.rows();
    int idx = -1;
    double min_norm = std::numeric_limits<double>::infinity();
    for (int k = 0; k < TV.rows(); k++) {
      double norm = (TV.row(k) - C).norm();
      if (norm < min_norm) {
        idx = k;
        min_norm = norm;
      }
    }
    SV.row(vcount) = TV.row(idx);
    sk_verts[vcount] = idx;
    vcount += 1;
  }

  sk_verts.conservativeResize(vcount);
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
  using namespace std;
  Eigen::MatrixXi BE(SV.rows() - 1, 2);

  for (int i = 0; i < SV.rows()-1; i++) {
    BE.row(i) = Eigen::RowVector2i(i, i+1);
  }

  Eigen::VectorXi b;
  Eigen::MatrixXd bc;
  if (!igl::boundary_conditions(TV, TT, SV, Eigen::VectorXi(), BE, Eigen::MatrixXi(), b, bc)) {
    cerr << "suspicious boundary conditions!" << endl;
    cerr << bc.rows() << ", " << bc.cols() << endl;
//    return false;
  }
  igl::BBWData bbw_data;
  bbw_data.active_set_params.max_iter = 3;
  bbw_data.verbosity = 2;
  return igl::bbw(TV, TT, b, bc, bbw_data, W);
}

bool mesh_datfile(const std::string& dat_filename, Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
  using namespace std;

  ifstream datfile(dat_filename);

  string raw_filename;
  datfile >> raw_filename;
  datfile >> raw_filename;
  raw_filename = string("./meshes/") + raw_filename;
  cout << "rawfile is " << raw_filename << endl;
  int w, h, d;
  string resolution_str;
  datfile >> resolution_str;
  datfile >> w;
  datfile >> h;
  datfile >> d;

  cout << "Grid has dimensions " << w << " x " << h << " x " << d << endl;

  char* data = new char[w*h*d];
  ifstream rawfile(raw_filename, std::ifstream::binary);
  rawfile.read(data, w*h*d);
  if (rawfile) {
    cout << "Read rawfile successfully" << endl;
  } else {
    cout << "Only read " << rawfile.gcount() << " bytes" << endl;
    return EXIT_FAILURE;
  }
  rawfile.close();

  Eigen::MatrixXd GP((w+2)*(h+2)*(d+2), 3);
  Eigen::VectorXd SV(GP.rows());

  int readcount = 0;
  int appendcount = 0;
  for (int zi = 0; zi < d+2; zi++) {
    for (int yi = 0; yi < h+2; yi++) {
      for (int xi = 0; xi < w+2; xi++) {
        if (xi == 0 || yi == 0 || zi == 0 || xi == (w+1) || yi == (h+1) || zi == (d+1)) {
          SV[readcount] = 0.0;
        } else {
          SV[readcount] = double(data[appendcount]);
          appendcount += 1;
        }
        GP.row(readcount) = Eigen::RowVector3d(xi, yi, zi);
        readcount += 1;
      }
    }
  }
  delete data;

  igl::copyleft::marching_cubes(SV, GP, w+2, h+2, d+2, V, F);

  igl::writeOFF("out.off", V, F);
}

void remove_garbage_components(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Eigen::MatrixXi& newF) {
  using namespace std;

  cout << "Input model has " << V.rows() << " vertices and " << F.rows() << " faces" << endl;

  cout << "Computing connected components..." << endl;
  Eigen::VectorXi components;
  igl::components(F, components);
  vector<int> component_count;
  component_count.resize(components.maxCoeff());
  for (int i = 0; i < V.rows(); i++) {
    component_count[components[i]] += 1;
  }
  int max_component = -1;
  int max_component_count = 0;
  for (int i = 0; i < component_count.size(); i++) {
    if (max_component_count < component_count[i]) {
      max_component = i;
      max_component_count = component_count[i];
    }
  }

  cout << "The model has " << component_count.size() << " connected components." << endl;
  cout << "Component " << max_component << " has the most vertices with a count of " << max_component_count << endl;

  newF.resize(F.rows(), 3);

  int fcount = 0;
  for(int i = 0; i < F.rows(); i++) {
    bool keep = true;
    for (int j = 0; j < 3; j++) {
      if (components[F(i, j)] != max_component) {
        keep = false;
        break;
      }
    }
    if (keep) {
      newF.row(fcount++) = F.row(i);
    }
  }

  cout << "Output model has " << fcount << " faces and " << newF.maxCoeff() << " vertices." << endl;
  newF.conservativeResize(fcount, 3);
}

bool slim(const Eigen::MatrixXd& TV, const Eigen::MatrixXi& TT, Eigen::VectorXi& b, igl::SLIMData& sData, bool first) {
  if (first) {
    Eigen::MatrixXd bc(b.rows(), 3);
    bc.row(0) = TV.row(b[0]);
    for (int i = 0; i < b.rows()-1; i++) {
      Eigen::RowVector3d s = TV.row(b[i]);
      Eigen::RowVector3d n = TV.row(b[i+1]);
      double d = (n - s).norm();
      bc.row(i+1) = bc.row(i) + Eigen::RowVector3d(0.0, 0.0, d);
    }

    Eigen::MatrixXd TV_0 = TV;
    double soft_const_p = 1e5;
    sData.exp_factor = 5.0;
    slim_precompute(TV, TT, TV_0, sData, igl::SLIMData::EXP_CONFORMAL, b, bc, soft_const_p);
  } else {
    slim_solve(sData, 20);
  }
}

int main(int argc, char *argv[]) {
  using namespace Eigen;
  using namespace std;

  if (argc > 1 && string(argv[1]) == "--mesh") {
    if (argc != 2) { cerr << "--mesh expected .dat file argument" << endl; }
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    string in_filename = string(argv[2]);
    mesh_datfile(in_filename, V, F);

    // For some stupid reason marching cubes flips the triangle order
    Eigen::VectorXd Vswap = V.col(1);
    V.col(1) = V.col(2);
    V.col(2) = Vswap;

    Eigen::MatrixXi Fmain;
    remove_garbage_components(V, F, Fmain);
    string out_filename = in_filename + string(".off");
    igl::writeOFF(out_filename, V, Fmain);
  } else if (argc > 1 && string(argv[1]) == "--straighten") {

  } else if (argc > 1 && string(argv[1]) == "--pick-endpoints") {

  }

  Eigen::MatrixXd TV, isoV, SV, W;
  Eigen::MatrixXi TF, TT, isoF;
  Eigen::VectorXd isovals;
  Eigen::VectorXi sk_verts;
  igl::SLIMData sData;

  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);

  // Load a mesh in OFF format

  load_yixin_tetmesh("outReoriented_.msh", TV, TF, TT);
  viewer.data().clear();
  viewer.data().set_mesh(TV, TF);

  cout << "Input mesh has " << TV.rows() << " vertices and " << TT.rows() << " tets" << endl;

  array<int, 2> selected_end_coords{{-1, -1}};
  int current_idx = 0;
  bool done_harmonic = false;
  double level_set = 0.0;
  double increment = 0.05;
  int current_weight = 0;
  bool slim_first_iter = true;

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
        extract_skeleton(25, TV, TT, isovals, SV, sk_verts);
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
    case 'X':
      cout << "About to do slim" << endl;
      slim(TV, TT, sk_verts, sData, slim_first_iter);
      slim_first_iter = false;
      cout << "Done slim" << endl;
      viewer.data().clear();
      viewer.data().set_mesh(TV, TF);
      viewer.data().show_faces = false;
      visualize_tet_wireframe(viewer, sData.V_o, TT, isovals);
      draw_skeleton(viewer, sData.bc);
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

  viewer.callback_mouse_down = [&](Viewer& viewer, int, int)->bool {
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
        selected_end_coords[current_idx] = vid;

        viewer.data().point_size = 7.0;
        Eigen::RowVector3d color;
        if (current_idx == 0) { color = Eigen::RowVector3d(0.0, 1.0, 0.0); } else { color = Eigen::RowVector3d(1.0, 0.0, 0.0); }
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

