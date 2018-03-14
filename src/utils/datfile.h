#ifndef DATFILE_H
#define DATFILE_H

#include <Eigen/Core>

#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <libgen.h>


struct DatFile {
private:
  bool set_filename(const std::string& filename) {
    std::ifstream is(filename);
    if (!is.good()) {
      std::cerr << "Error .dat file '" << filename << "' does not exist." << std::endl;
      return false;
    }

    char* datfile_full_path = realpath(filename.c_str(), NULL);
    char* datfile_full_path_dup = strdup(datfile_full_path);

    m_filename = std::string(filename);
    char* dat_directory = dirname(datfile_full_path);
    m_directory = std::string(dat_directory);

    char* dat_basename = basename(datfile_full_path_dup);
    m_basename = std::string(dat_basename);

    free(datfile_full_path);
    free(datfile_full_path_dup);
    return true;
  }

public:
  int w, h, d;
  Eigen::RowVector3d m_bb_min, m_bb_max;
  std::string m_raw_filename;
  std::string m_mesh_filename;
  std::string m_format;
  std::string m_filename;
  std::string m_directory;
  std::string m_basename;

  bool serialize(const std::string& filename) {
    using namespace std;

    ofstream of(filename);
    of << "RawFile: " << m_raw_filename << endl;
    of << "Resolution: " << w << " " << h << " " << d << endl;
    of << "Format: " << m_format << endl;
    of << "Mesh: " << m_mesh_filename << endl;
    of << "BBmin: " << m_bb_min[0] << " " << m_bb_min[1] << " " << m_bb_min[2] << endl;
    of << "BBmax: " << m_bb_max[0] << " " << m_bb_max[1] << " " << m_bb_max[2] << endl;
    of.close();

    return true;
  }

  bool deserialize(const std::string& filename) {
    using namespace std;
    string token;
    ifstream is(filename);
    if (!set_filename(filename)) {
      return false;
    }

    cout << "Deserializing " << filename << endl;
    while(is >> token) {
      if (token == "RawFile:") {
        is >> m_raw_filename;
        cout << "RawFile: " << m_raw_filename << endl;
      } else if (token == "Resolution:") {
        is >> w;
        is >> h;
        is >> d;
        cout << "Resolution: " << w << " " << h << " " << d << endl;
      } else if (token == "Format:") {
        is >> m_format;
        cout << "Format: " << m_format << endl;
      } else if(token == "Mesh:") {
        is >> m_mesh_filename;
        cout << "Mesh: " << m_mesh_filename << endl;
      } else if (token == "BBmin:") {
        is >> m_bb_min[0];
        is >> m_bb_min[1];
        is >> m_bb_min[2];
        cout << "BBmin: " << m_bb_min[0] << " " << m_bb_min[1] << " " << m_bb_min[2] << endl;
      } else if (token == "BBmax:") {
        is >> m_bb_max[0];
        is >> m_bb_max[1];
        is >> m_bb_max[2];
        cout << "BBmax: " << m_bb_max[0] << " " << m_bb_max[1] << " " << m_bb_max[2] << endl;
      } else {
        cerr << "ERROR: Unexpected token " << token << endl;
        return false;
      }
    }
    cout << "Done!" << endl;

    return true;
  }
};

#endif // DATFILE_H
