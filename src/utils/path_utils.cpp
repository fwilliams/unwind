#include "path_utils.h"

#include <errno.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif /* HAVE_UNISTD_H */

#include <string>
#include <vector>
#include <cstdlib>

#ifdef WIN32
#include <Windows.h>

static BOOL directoryExists(LPCTSTR szPath) {
  DWORD dwAttrib = GetFileAttributes(szPath);

  return (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

int mkpath(const char* cpath, mode_t mode) {
  std::string path(cpath);
  const int PathBufferSize = 4096;
  std::vector<char> buffer(PathBufferSize);

  const DWORD success = GetFullPathNameA(path.c_str(), PathBufferSize,
                                         buffer.data(), nullptr);
  path.assign(buffer.data());

  std::string::size_type pos = 0;
  do {
    pos = path.find_first_of("\\/", pos + 1);
    std::string p = path.substr(0, pos);
    CreateDirectoryA(p.c_str(), nullptr);
  } while (pos != std::string::npos);

  return 0;
}

std::pair<std::string, std::string> dir_and_base_name(const char* name) {
    std::string filename = std::string(name);
    std::string::size_type separator = filename.rfind('\\');
    if (separator != std::string::npos) {
        return { filename.substr(0, separator), filename.substr(separator + 1) };
    }
    else {
        separator = filename.rfind('/');
        if (separator != std::string::npos) {
            return { filename.substr(0, separator), filename.substr(separator + 1) };
        } else {
            return { filename, filename };
        }
    }
}

FileType get_file_type(const char* path) {
    struct stat stat_buf;
    int retval = stat(path, &stat_buf);
    if (retval != 0 && errno == ENOENT) {
        return FT_DOES_NOT_EXIST;
    } else if (retval != 0) {
        return FT_ERROR;
    } else if (S_ISDIR(stat_buf.st_mode)) {
        return FT_DIRECTORY;
    } else if (S_ISREG(stat_buf.st_mode)) {
        return FT_REGULAR_FILE;
    } else {
        return FT_OTHER;
    }
}
#else

#include <string.h>
#include <libgen.h>
#include <limits.h>

static int do_mkdir(const char *path, mode_t mode) {
  typedef struct stat Stat;
  Stat st;
  int status = 0;

  if (stat(path, &st) != 0) {
    /* Directory does not exist. EEXIST for race condition */
    if (mkdir(path, mode) != 0 && errno != EEXIST) {
      status = -1;
    }
  }
  else if (!S_ISDIR(st.st_mode)) {
    errno = ENOTDIR;
    status = -1;
  }

  return(status);
}

int mkpath(const char *path, mode_t mode) {
  char *pp;
  char *sp;
  int  status;
  char *copypath = strdup(path);

  status = 0;
  pp = copypath;
  while (status == 0 && (sp = strchr(pp, '/')) != 0) {
    if (sp != pp) {
      /* Neither root nor double slash in path */
      *sp = '\0';
      status = do_mkdir(copypath, mode);
      *sp = '/';
    }
    pp = sp + 1;
  }
  if (status == 0) {
    status = do_mkdir(path, mode);
  }
  free(copypath);
  return (status);
}

FileType get_file_type(const char* path) {
    struct stat stat_buf;
    int retval = stat(path, &stat_buf);
    if (retval != 0 && errno == ENOENT) {
        return FT_DOES_NOT_EXIST;
    } else if (retval != 0) {
        return FT_ERROR;
    } else if (S_ISDIR(stat_buf.st_mode)) {
        return FT_DIRECTORY;
    } else if (S_ISREG(stat_buf.st_mode)) {
        return FT_REGULAR_FILE;
    } else {
        return FT_OTHER;
    }
}


std::pair<std::string, std::string> dir_and_base_name(const char* name) {
    char* full_path = realpath(name, NULL);
    char* full_path_dup = strdup(full_path);

    char* dir_name_1 = dirname(full_path);
    char* base_name_1 = basename(full_path_dup);

    char dir_name[PATH_MAX*4];
    strcpy(dir_name, dir_name_1);

    char base_name[PATH_MAX*4];
    strcpy(base_name, base_name_1);

    std::string ret1, ret2;
    ret1.assign(dir_name);
    ret2.assign(base_name);

    return std::make_pair(ret1, ret2);
}
#endif
