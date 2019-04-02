# Unwind: Interactive Fish Straightening
----------------------------------------
This repository contains the source code for [Unwind](http://google.com), a tool for unwarping bent volumetric scans of fishes.

![](https://github.com/fwilliams/unwind/blob/master/img/teaser.png)

## Installing Unwind
Executables for Unwind are available for [Windows](http://). 

Packages for Linux will be made available soon. For now, you can compile Unwind from source (see below)

## Compiling Unwind from source
Unwind requires minimal dependencies and can be easily compiled from source.

### On Linux
To build Unwind you need to have the following dependencies installed:
* A modern C++ compiler (C++ 14 or later) such as GCC or Clang
* CMake version 3.8 or later
* Qt5 

These should all be available for installation through the package manager on any distribution. Once you have installed, the dependencies simply clone this repository:
```
git clone ttps://github.com/fwilliams/unwind
cd unwind
```

The from the root directory of the repository run:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

The binary for Unwind will be in `build/src/unwind`

## On Windows with Visual Studio
TODO: Windows instructions
