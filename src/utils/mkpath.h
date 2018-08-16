#ifndef MKPATH_H
#define MKPATH_H

#include <sys/stat.h>

int mkpath(const char *path, mode_t mode=0777);

#endif // MKPATH_H
