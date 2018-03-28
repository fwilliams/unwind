#ifndef DATFILE_H
#define DATFILE_H

#include <Eigen/Core>

#include <string>


struct DatFile {
private:
#ifdef _MSC_VER
  std::pair<std::string, std::string> dir_and_base_name(const char* name);
#endif
  bool set_filename(const std::string& filename);

public:
  int w, h, d;
  Eigen::RowVector3d m_bb_min, m_bb_max;
  std::string m_raw_filename;
  std::string m_mesh_filename;
  std::string m_format;
  std::string m_filename;
  std::string m_directory;
  std::string m_basename;
  std::string m_texture_filename;
  std::string m_thin_raw_filename;
  std::string m_thin_surface_mesh;

  DatFile(const std::string& filename);

  DatFile() = default;

  bool serialize(const std::string& filename);

  bool deserialize(const std::string& filename);
};

#endif // DATFILE_H
