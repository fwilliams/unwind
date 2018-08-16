#include "mkpath.h"

#include <errno.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif /* HAVE_UNISTD_H */

#include <string.h>
#include <cstdlib>

#ifdef WIN32
#include <Windows.h>

static BOOL directoryExists(LPCTSTR szPath) {
  DWORD dwAttrib = GetFileAttributes(szPath);

  return (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

void mkpath(cont char* cpath, mode_t mode=0777) {
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
}
#else

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

#endif
