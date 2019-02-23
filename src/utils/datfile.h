#ifndef DATFILE_H
#define DATFILE_H

#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <string>
#include <memory>
#include <igl/serialize.h>


struct DatFile {
private:
#ifdef _WIN32
  std::pair<std::string, std::string> dir_and_base_name(const char* name);
#endif
  bool set_filename(const std::string& filename, std::shared_ptr<spdlog::logger> logger);

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

  DatFile(const std::string& filename, std::shared_ptr<spdlog::logger> logger);

  DatFile() = default;

  bool serialize(const std::string& filename);

  bool deserialize(const std::string& filename, std::shared_ptr<spdlog::logger> logger);
};

namespace igl {
namespace serialization {
template <> inline void serialize(const DatFile& obj, std::vector<char>& buffer) {
    igl::serialize(obj.m_raw_filename, std::string("m_raw_filename"), buffer);
    igl::serialize(obj.m_mesh_filename, std::string("m_mesh_filename"), buffer);
    igl::serialize(obj.m_format, std::string("m_format"), buffer);
    igl::serialize(obj.m_filename, std::string("m_filename"), buffer);
    igl::serialize(obj.m_directory, std::string("m_directory"), buffer);
    igl::serialize(obj.m_basename, std::string("m_basename"), buffer);
    igl::serialize(obj.m_texture_filename, std::string("m_texture_filename"), buffer);
    igl::serialize(obj.m_thin_raw_filename, std::string("m_thin_raw_filename"), buffer);
    igl::serialize(obj.m_thin_surface_mesh, std::string("m_thin_surface_mesh"), buffer);
}

template <> inline void deserialize(DatFile& obj, const std::vector<char>& buffer){
    igl::deserialize(obj.m_raw_filename, std::string("m_raw_filename"), buffer);
    igl::deserialize(obj.m_mesh_filename, std::string("m_mesh_filename"), buffer);
    igl::deserialize(obj.m_format, std::string("m_format"), buffer);
    igl::deserialize(obj.m_filename, std::string("m_filename"), buffer);
    igl::deserialize(obj.m_directory, std::string("m_directory"), buffer);
    igl::deserialize(obj.m_basename, std::string("m_basename"), buffer);
    igl::deserialize(obj.m_texture_filename, std::string("m_texture_filename"), buffer);
    igl::deserialize(obj.m_thin_raw_filename, std::string("m_thin_raw_filename"), buffer);
    igl::deserialize(obj.m_thin_surface_mesh, std::string("m_thin_surface_mesh"), buffer);
}
}
}
#endif // DATFILE_H
