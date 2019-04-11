# Unwind: Interactive Fish Straightening

This repository contains the source code for [Unwind](https://arxiv.org/abs/1904.04890), a tool for unwarping bent volumetric scans of fishes.

![](https://github.com/fwilliams/unwind/blob/master/img/teaser.png)

## Installing Unwind
Executables for Unwind are available for [Windows](https://drive.google.com/open?id=1sMOADLW4UndNcVG1y5wLlFNg16LJLoGY). 

Packages for Linux will be made available soon. For now, you can compile Unwind from source (see below)

## Compiling Unwind from source
Unwind requires minimal dependencies and can be easily compiled from source.


### Linux

#### 1. Install Dependencies
To build Unwind you need to have the following dependencies installed:
* A modern C++ compiler (C++ 14 or later) such as GCC or Clang
* CMake version 3.8 or later
* Qt5 
* Xorg development libraries (Needed by GLFW)
##### 1.1 Ubuntu
You can install the necessary packages on ubuntu as follows:
```
sudo apt-get install build-essential cmake xorg-dev qt5-default
```

#### 2. Compile
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

#### 3. Run
The binary for Unwind will be in `build/src/unwind` relative to the root of the repository.

### Windows with Visual Studio

#### 1. Clone the repository
From [Powershell](https://docs.microsoft.com/en-us/powershell/scripting/install/installing-powershell-core-on-windows?view=powershell-6), run:
```
git clone https://github.com/fwilliams/unwind
cd unwind
```

#### 2. Install Dependencies
To build Unwind on Windows, you need to install the following dependencies:
* vcpkg: Install using instructions at [https://github.com/Microsoft/vcpkg](https://github.com/Microsoft/vcpkg)
* Qt5: Download the installer [https://www.qt.io/offline-installers](https://www.qt.io/offline-installers)
* CMake: Dowload and run the installer: [https://cmake.org/download/](https://cmake.org/download/)
* Boost: From the vcpkg directory, run:
    ```
    .\vcpkg.exe --triplet x64-windows install mpir mpfr boost
    ```

#### 3. Generate a Visual Studio Project
Using [Powershell](https://docs.microsoft.com/en-us/powershell/scripting/install/installing-powershell-core-on-windows?view=powershell-6), from the root directory of the Unwind repository, first create a build directory:
```
mkdir build
cd build
```
then generate a visual studio project:
```
cmake -G 'Visual Studio 15 2017 Win64' -DCMAKE_PREFIX_PATH="C:/Qt/Qt5.10.1/5.10.1/msvc2017_64" -DMPFR_INCLUDE_DIR:PATH="C:/local/vcpkg/installed/x64-windows/include" -DMPFR_LIBRARIES:FILEPATH="C:/local/vcpkg/installed/x64-windows/lib/mpfr.lib" -DGMP_INCLUDE_DIR:PATH="C:/local/vcpkg/installed/x64-windows/include" -DGMP_LIBRARIES:FILEPATH="C:/local/vcpkg/installed/x64-windows/lib/mpir.lib" -DBOOST_ROOT="C:/local/vcpkg/installed/x64-windows" ../
```
*NOTE: The above commands assumes that Qt 5.10.1 is installed in C:\Qt, and vcpkg in C:\local\vcpkg. 
The paths need to be updated according to where these are installed (and version of Qt installed).*

#### 4. Compile From Visual Studio
The previous step generates a `fish_deformation.sln` file in the `build` directory creaated in step 3
Open this file using Visual Studio and build the project (making sure that the "Solution configuration" is release and "Solution platform" is x64
