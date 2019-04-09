# Unwind: Interactive Fish Straightening

This repository contains the source code for [Unwind](https://cs.nyu.edu/~francisw/files/unwind.pdf), a tool for unwarping bent volumetric scans of fishes.

![](https://github.com/fwilliams/unwind/blob/master/img/teaser.png)

## Installing Unwind
Executables for Unwind are available for [Windows](https://drive.google.com/open?id=1sMOADLW4UndNcVG1y5wLlFNg16LJLoGY). 

Packages for Linux will be made available soon. For now, you can compile Unwind from source (see below)

## Compiling Unwind from source
Unwind requires minimal dependencies and can be easily compiled from source.

### On Linux

#### Dependencies
To build Unwind you need to have the following dependencies installed:
* A modern C++ compiler (C++ 14 or later) such as GCC or Clang
* CMake version 3.8 or later
* Qt5 
* Xorg development libraries (Needed by GLFW)

##### Installing Dependencies on Ubuntu
You can install the necessary packages on ubuntu as follows:
```
sudo apt-get install build-essential cmake xorg-dev qt5-default
```

#### Building on Linux
Once you have installed the dependencies, simply clone this repository:
```
git clone https://github.com/fwilliams/unwind
cd unwind
```

Then, from the root directory of the repository run:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

The binary for Unwind will be in `build/src/unwind`

## On Windows with Visual Studio
TODO: Windows instructions
