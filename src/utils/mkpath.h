#ifndef MKPATH_H
#define MKPATH_H

#include <sys/stat.h>
#ifdef _MSC_VER
using mode_t = int;
#endif // WIN32

int mkpath(const char *path, mode_t mode=0777);

#endif // MKPATH_H
