#include "datfile.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#ifndef _WIN32
#include <libgen.h>
#else
#include <Windows.h>
#endif

#ifdef _WIN32

std::pair<std::string, std::string> DatFile::dir_and_base_name(const char* name) {
    std::string filename = std::string(name);
    std::string::size_type separator = filename.rfind('\\');
    if (separator != std::string::npos) {
        return { filename.substr(0, separator), filename.substr(separator + 1) };
    }
    else {
        return { filename, filename };
    }
}

bool DatFile::set_filename(const std::string& filename, std::shared_ptr<spdlog::logger> logger) {
    std::ifstream is(filename);
    if (!is.good()) {
        logger->error("Error .dat file '{}' does not exist", filename);
        return false;
    }

    constexpr const int BufferSize = 1024;
    char datfile_full_path[1024];
    const DWORD success = GetFullPathNameA(filename.c_str(), BufferSize, datfile_full_path, 0);
    char* datfile_full_path_dup = strdup(datfile_full_path);
    m_filename = std::string(datfile_full_path);

    const std::pair<std::string, std::string>& v = dir_and_base_name(datfile_full_path);
    m_directory = v.first;
    m_basename = v.second;
    //m_filename = std::string(filename);
    //m_directory = dirname(datfile_full_path);

    //char* dat_basename = basename(datfile_full_path_dup);
    //m_basename = std::string(dat_basename);

    //free(datfile_full_path);
    //free(datfile_full_path_dup);
    return true;
}
#else
bool DatFile::set_filename(const std::string& filename, std::shared_ptr<spdlog::logger> logger) {
    std::ifstream is(filename);
    if (!is.good()) {
        logger->error("Error .dat file '{}' does not exist", filename);
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
#endif


DatFile::DatFile(const std::string& filename, std::shared_ptr<spdlog::logger> logger) {
    deserialize(filename, logger);
}

bool DatFile::serialize(const std::string& filename) {
    using namespace std;

    ofstream of(filename);
    of << "RawFile: " << m_raw_filename << endl;
    of << "ThinRawFile: " << m_thin_raw_filename << endl;
    of << "Resolution: " << w << " " << h << " " << d << endl;
    of << "Format: " << m_format << endl;
    of << "SurfaceMesh: " << m_mesh_filename << endl;
    of << "ThinSurfaceMesh: " << m_thin_surface_mesh << endl;
    of << "TextureFile: " << m_texture_filename << endl;
    of << "BBmin: " << m_bb_min[0] << " " << m_bb_min[1] << " " << m_bb_min[2] << endl;
    of << "BBmax: " << m_bb_max[0] << " " << m_bb_max[1] << " " << m_bb_max[2] << endl;
    of.close();

    return true;
}

bool DatFile::deserialize(const std::string& filename, std::shared_ptr<spdlog::logger> logger) {
    using namespace std;
    string token;
    ifstream is(filename);
    if (!set_filename(filename, logger)) {
        return false;
    }

    logger->debug("Deserializing {}", filename);
    while(is >> token) {
        if (token == "RawFile:") {
            is >> m_raw_filename;
            logger->debug("RawFile: {}", m_raw_filename);
        } else if (token == "Resolution:") {
            is >> w;
            is >> h;
            is >> d;
            logger->debug("Resolution: {} {} {}", w, h, d);
        } else if (token == "Format:") {
            is >> m_format;
            logger->debug("Format: {}", m_format);
        } else if(token == "SurfaceMesh:") {
            is >> m_mesh_filename;
            logger->debug("SurfaceMesh: {}", m_mesh_filename);
        } else if (token == "BBmin:") {
            is >> m_bb_min[0];
            is >> m_bb_min[1];
            is >> m_bb_min[2];
            logger->debug("BBmin: {} {} {}", m_bb_min[0], m_bb_min[1], m_bb_min[2]);
        } else if (token == "ThinSurfaceMesh:") {
            is >> m_thin_surface_mesh;
            logger->debug("ThinSurfaceMesh: {}", m_thin_surface_mesh);
        } else if (token == "BBmax:") {
            is >> m_bb_max[0];
            is >> m_bb_max[1];
            is >> m_bb_max[2];
            logger->debug("BBmax: {} {} {}", m_bb_max[0], m_bb_max[1], m_bb_max[2]);
        } else if (token == "TextureFile:") {
            is >> m_texture_filename;
            logger->debug("TextureFile: {}", m_texture_filename);
        } else if (token == "ThinRawFile:") {
            is >> m_thin_raw_filename;
            logger->debug("ThinRawFile: {}", m_thin_raw_filename);
        } else {
            logger->error("Unexpected token {}", token);
            return false;
        }
    }
    logger->debug("Done deserializing {}", filename);

    return true;
}

