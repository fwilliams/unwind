#include <igl/copyleft/marching_cubes.h>
#include <igl/components.h>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>

#include <iostream>
#include <fstream>

#include "datfile.h"


bool compute_surface_mesh(DatFile& datfile,
                  Eigen::MatrixXd& V,
                  Eigen::MatrixXi& F, bool thin=false) {
  using namespace std;

  string raw_filename;
  if (thin) {
    raw_filename = datfile.m_directory + string("/") + datfile.m_thin_raw_filename;
  } else  {
    raw_filename = datfile.m_directory + string("/") + datfile.m_raw_filename;
  }
  assert(datfile.m_format == string("UINT8"));

  const size_t num_datfile_bytes = datfile.w * datfile.h * datfile.d;
  char* data = new char[num_datfile_bytes];
  ifstream rawfile(raw_filename, std::ifstream::binary);
  if (!rawfile.good()) {
    cerr << "ERROR: RawFile '" << raw_filename << "' does not exist." << endl;
    return false;
  }
  rawfile.read(data, num_datfile_bytes);
  if (!rawfile) {
    cout << "ERROR: Only read " << rawfile.gcount() <<
            " bytes from Raw File '" << datfile.m_raw_filename <<
            "' but expected to read " << num_datfile_bytes <<
            " bytes." << endl;
    return false;
  }
  rawfile.close();

  cout << "Generating Marching Cubes Input..." << endl;
  Eigen::MatrixXd GP((datfile.w+2)*(datfile.h+2)*(datfile.d+2), 3);
  Eigen::VectorXd SV(GP.rows());

  int readcount = 0;
  int appendcount = 0;
  for (int zi = 0; zi < datfile.d+2; zi++) {
    for (int yi = 0; yi < datfile.h+2; yi++) {
      for (int xi = 0; xi < datfile.w+2; xi++) {
        if (xi == 0 || yi == 0 || zi == 0 || xi == (datfile.w+1) ||
            yi == (datfile.h+1) || zi == (datfile.d+1)) {
          SV[readcount] = -1.0;
        } else {
          SV[readcount] = double((unsigned char)(data[appendcount]));
          appendcount += 1;
        }
        GP.row(readcount) = Eigen::RowVector3d(xi, yi, zi);
        readcount += 1;
      }
    }
  }
  delete data;

  datfile.m_bb_min = Eigen::RowVector3d(1.0, 1.0, 1.0);
  datfile.m_bb_max = Eigen::RowVector3d(datfile.w, datfile.h, datfile.d);

  cout << "Running Marching Cubes..." << endl;
  igl::copyleft::marching_cubes(SV, GP, datfile.w+2, datfile.h+2, datfile.d+2, V, F);

  cout << "Marching cubes odel has " << V.rows() << " vertices and " <<
          F.rows() << " faces." << endl;
  return true;
}


void remove_garbage_components(const Eigen::MatrixXd& V,
                               const Eigen::MatrixXi& F,
                               Eigen::MatrixXi& newF, double keep_thresh=1.0) {
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
  int min_component = -1;
  int max_component_count = std::numeric_limits<int>::min();
  int min_component_count = std::numeric_limits<int>::max();
  for (int i = 0; i < component_count.size(); i++) {
    if (max_component_count < component_count[i]) {
      max_component = i;
      max_component_count = component_count[i];
    }
    if (min_component_count > component_count[i]) {
      min_component = i;
      min_component_count = component_count[i];
    }
  }
  cout << "Component " << max_component <<
          " has the most vertices with a count of " <<
          max_component_count << endl;
  cout << "Component " << min_component << " has the fewest vertices with a count of " << min_component_count << endl;

  cout << "Deleting smaller components..." << endl;
  newF.resize(F.rows(), 3);

  int keep_thresh_count = min_component_count + int(keep_thresh*(max_component_count - min_component_count));
  int fcount = 0;
  for(int i = 0; i < F.rows(); i++) {
    bool keep = true;
    for (int j = 0; j < 3; j++) {
      if (component_count[components[F(i, j)]] < keep_thresh_count) {
        keep = false;
        break;
      }
    }
    if (keep) {
      newF.row(fcount++) = F.row(i);
    }
  }

  cout << "Final model has " << fcount << " faces and " <<
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

  MatrixXd Vfat, Vthin;
  MatrixXi Ffat, Fthin, F_max_component_fat, F_max_component_thin;
  DatFile out_datfile;
  if (!out_datfile.deserialize(argv[1])) {
    return EXIT_FAILURE;
  }
  cout << endl;

  if (!compute_surface_mesh(out_datfile, Vfat, Ffat, false/*thin*/)) {
    return EXIT_FAILURE;
  }
  remove_garbage_components(Vfat, Ffat, F_max_component_fat, 1.0);

  string out_mesh_filename = out_datfile.m_directory + string("/") + out_datfile.m_basename + string(".off");
  cout << "Saving fat mesh file " << out_mesh_filename << endl;
  igl::writeOFF(out_mesh_filename, Vfat, F_max_component_fat);
  out_datfile.m_mesh_filename = out_datfile.m_basename + string(".off");

  if (!out_datfile.m_thin_raw_filename.empty()) {
    if (!compute_surface_mesh(out_datfile, Vthin, Fthin, true/*thin*/)) {
      return EXIT_FAILURE;
    }
    remove_garbage_components(Vthin, Fthin, F_max_component_thin, 0.1);

    out_mesh_filename = out_datfile.m_directory + string("/") + out_datfile.m_basename + string(".thin.off");
    cout << "Saving thin mesh file " << out_mesh_filename << endl;
    igl::writeOFF(out_mesh_filename, Vthin, F_max_component_thin);
    out_datfile.m_thin_surface_mesh = out_datfile.m_basename + string(".thin.off");
  }

  string out_dat_filename = out_datfile.m_directory + string("/") + out_datfile.m_basename;
  cout << "Saving output dat file" << endl;
  if (!out_datfile.serialize(out_dat_filename)) {
    return EXIT_FAILURE;
  }

  cout << "Done!" << endl;
  return EXIT_SUCCESS;
}

