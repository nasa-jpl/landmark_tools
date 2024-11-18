## Build
Instructions for building executables from source code.

Build has been tested on Mac OS 13.4.1 and Ubuntu 20.04

### Dependencies

You will need a C compiler and CMake

Install libraries with brew (Mac) or apt (Ubuntu)
- libpng
- libyaml
- gsl
- gdal (optional)
- opencv (optional)

For the renderer and visualization scripts, you will also need Python=3.11 and the following python packages
- bpy 
- matplotlib
- numpy
- pillow

Compatibility note: bpy is supported for M3 Mac OS but not M3 Ubuntu OS. If using an M3 Mac, use native OS.

### Setup XCode Project
```
mkdir build
cd build
cmake -G Xcode -DWITH_OPENCV=ON ..
```
Open project and build in XCode.

### CMake only
```
mkdir build
cd build
cmake -DWITH_OPENCV=ON ..
make
```
