#include <igl/copyleft/marching_cubes.h>
#include <igl/components.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include <libgen.h>


struct OutputDatFile {
  int w, h, d;
  Eigen::RowVector3d bb_min, bb_max;
  std::string raw_filename;
  std::string mesh_filename;
  std::string format;

  void serialize(const std::string& filename) {
    using namespace std;
    ofstream of(filename);
    of << "RawFile: " << raw_filename << endl;
    of << "Resolution: " << w << " " << h << " " << d << endl;
    of << "Format: " << format << endl;
    of << "Mesh: " << mesh_filename << endl;
    of << "BBmin: " << bb_min[0] << " " << bb_min[1] << " " << bb_min[2] << endl;
    of << "BBmax: " << bb_max[0] << " " << bb_max[1] << " " << bb_max[2] << endl;
    of.close();
  }
};


bool mesh_datfile(const std::string& dat_filename,
                  Eigen::MatrixXd& V,
                  Eigen::MatrixXi& F,
                  OutputDatFile& output) {
  using namespace std;

  char* datfile_full_path = realpath(dat_filename.c_str(), NULL);

  cout << "Loading datfile " << datfile_full_path << endl;
  ifstream datfile(dat_filename);

  char* dat_directory = dirname(datfile_full_path);

  string raw_filename;
  datfile >> raw_filename;
  datfile >> raw_filename;
  raw_filename = string(dat_directory) + string("/") + raw_filename;
  cout << "Raw file is " << raw_filename << endl;
  int w, h, d;
  string resolution_str;
  datfile >> resolution_str;
  datfile >> w;
  datfile >> h;
  datfile >> d;

  datfile >> output.format;
  datfile >> output.format;
  assert(output.format == string("UINT8"));

  cout << "Grid has dimensions " << w << " x " << h << " x " << d << endl;

  char* data = new char[w*h*d];
  ifstream rawfile(raw_filename, std::ifstream::binary);
  rawfile.read(data, w*h*d);
  if (rawfile) {
    cout << "Read raw file successfully" << endl;
    output.raw_filename = raw_filename;
  } else {
    cout << "Only read " << rawfile.gcount() << " bytes" << endl;
    free(datfile_full_path);
    return false;
  }
  rawfile.close();

  cout << "Generating Marching Cubes Input..." << endl;
  Eigen::MatrixXd GP((w+2)*(h+2)*(d+2), 3);
  Eigen::VectorXd SV(GP.rows());

  int readcount = 0;
  int appendcount = 0;
  for (int zi = 0; zi < d+2; zi++) {
    for (int yi = 0; yi < h+2; yi++) {
      for (int xi = 0; xi < w+2; xi++) {
        if (xi == 0 || yi == 0 || zi == 0 || xi == (w+1) ||
            yi == (h+1) || zi == (d+1)) {
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

  output.bb_min = Eigen::RowVector3d(1.0, 1.0, 1.0);
  output.bb_min = Eigen::RowVector3d(w, h, d);
  output.w = w;
  output.h = h;
  output.d = d;

  cout << "Running Marching Cubes..." << endl;
  igl::copyleft::marching_cubes(SV, GP, w+2, h+2, d+2, V, F);

  cout << "Marching cubes odel has " << V.rows() << " vertices and " <<
          F.rows() << " faces." << endl;
  free(datfile_full_path);
  return true;
}


void remove_garbage_components(const Eigen::MatrixXd& V,
                               const Eigen::MatrixXi& F,
                               Eigen::MatrixXi& newF) {
  using namespace std;

  cout << "Computing connected components..." << endl;
  Eigen::VectorXi components;
  igl::components(F, components);

  cout << "Counting connected components..." << endl;
  vector<int> component_count;
  component_count.resize(components.maxCoeff());
  for (int i = 0; i < V.rows(); i++) {
    component_count[components[i]] += 1;
  }
  cout << "The model has " << component_count.size() <<
          " connected components." << endl;

  cout << "Finding component with most vertices..." << endl;
  int max_component = -1;
  int max_component_count = 0;
  for (int i = 0; i < component_count.size(); i++) {
    if (max_component_count < component_count[i]) {
      max_component = i;
      max_component_count = component_count[i];
    }
  }
  cout << "Component " << max_component <<
          " has the most vertices with a count of " <<
          max_component_count << endl;

  cout << "Deleting smaller components..." << endl;
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

  cout << "Largest component of model has " << fcount << " faces and " <<
          newF.maxCoeff() << " vertices." << endl;
  newF.conservativeResize(fcount, 3);
}


int main(int argc, char *argv[]) {
  using namespace Eigen;
  using namespace std;

  if (argc != 2) {
    cerr << "Invalid number of arguments should be:" << endl;
    cerr << "  mesh_datfile_bin /path/to/.dat/file" << endl;
    return EXIT_FAILURE;
  }

  MatrixXd V;
  MatrixXi F, F_max_component;
  OutputDatFile out_datfile;

  char* datfile_full_path = realpath(argv[1], NULL);

  mesh_datfile(datfile_full_path, V, F, out_datfile);
  remove_garbage_components(V, F, F_max_component);
  cout << "Flipping triangle orientation..." << endl;
  VectorXd V2 = V.col(2);
  V.col(2) = V.col(1);
  V.col(1) = V2;

  char* datfile_full_path_dup = strdup(datfile_full_path);
  char* dat_directory = dirname(datfile_full_path_dup);
  char* dat_basename = basename(datfile_full_path);

  string out_mesh_filename = string(dat_directory) + string("/") +
                             string(dat_basename) + string(".out.off");
  cout << "Saving mesh file " << out_mesh_filename << endl;
  igl::writeOFF(out_mesh_filename, V, F_max_component);
  out_datfile.mesh_filename = out_mesh_filename;

  string out_dat_filename = string(dat_directory) + string("/") +
                            string(dat_basename) + string(".out.dat");
  cout << "Saving output dat file" << endl;
  out_datfile.serialize(out_dat_filename);

  cout << "Done!" << endl;
  free(datfile_full_path_dup);
  free(datfile_full_path);
}

