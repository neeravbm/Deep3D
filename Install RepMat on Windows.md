# Install RepMat on Windows

1. Set up the variables.

```
"D:\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
```

## Install Python 3

1. QtQml requires Python 2. QtQml doesn't work with Python 3. Download `python-2.7.13.msi` from [Python's website](https://www.python.org/ftp/python/2.7.13/python-2.7.13.msi) and run it. Install Python at `D:\Python27`. Make sure that `python` command is in PATH.

## Install Qt

1. Download Qt from [its website]() to `E:\libs\qt\`.

```
cd /mnt/e
mkdir -p libs/qt
cd libs/qt
wget http://download.qt.io/official_releases/qt/5.9/5.9.2/single/qt-everywhere-opensource-src-5.9.2.zip
unzip -xvf qt-everywhere-opensource-src-5.9.0.zip
rm qt-everywhere-opensource-src-5.9.0.zip
```


### Debug build

1. Configure and build Qt.

```
configure -opengl dynamic -prefix E:\libs\qt\builds\win64-shared-debug -opensource -confirm-license -debug
D:\jom_1_1_2\jom.exe
D:\jom_1_1_2\jom.exe install
```

### Release build

1. Configure and build Qt.

```
configure -opengl dynamic -prefix E:\libs\qt\builds\win64-shared-release -opensource -confirm-license -release
D:\jom_1_1_2\jom.exe
D:\jom_1_1_2\jom.exe install
```

## Install LAPACK

1. Download LAPACK from [its website](http://www.netlib.org/lapack/index.html#_lapack_version_3_7_1_2) to `E:\libs\lapack\src`.


cmake -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=ON -DLAPACKE:BOOL=ON -DCBLAS:BOOL=ON -DCMAKE_Fortran_COMPILER=ftn95.exe ..\..\lapack-3.7.1\


## Install Eigen

1. Download Eigen from [bitbucket](https://bitbucket.org/eigen/eigen/) to `E:\libs\Eigen\src`.

```
cd /mnt/e
mkdir -p libs\Eigen
cd libs/Eigen
wget https://bitbucket.org/eigen/eigen/get/1ff051b7a3ae.zip
unzip 1ff051b7a3ae.zip
mv eigen-eigen-1ff051b7a3ae src
rm 1ff051b7a3ae.zip
cd ..
```

### Debug build

1. Create `builds/win-debug-cl` directory and change into it.

```
mkdir -p builds/win-debug-cl
cd builds/win-debug-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug ..\..\src\
nmake
nmake install
```

### Release build

1. Create `builds/win-debug-cl` directory and change into it.

```
mkdir -p builds/win-release-cl
cd builds/win-release-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release ..\..\src\
nmake
nmake install
```


## Install Boost

1. Download Boost 1.62 from [Sourceforge](https://sourceforge.net/projects/boost/files/boost/1.62.0/boost_1_62_0.zip/download) to `E:\libs\boost\boost_1_62_0`.

2. Go to `boost_1_62_0\tools\build` directory and install `b2` using the following commands.

```
bootstrap
```


### Debug build

1. Compile. Compiling as shared library using the command `.\b2 --prefix=E:\libs\boost\builds\win64-shared-debug-cl variant=debug threading=multi link=shared install` causes some libraries to be created static and some to be shared. Hence for now, we'll just create static libraries with shared runtime C/C++ libraries.

```
.\b2 --prefix=E:\libs\boost\builds\win64-static-debug-cl threading=multi link=static runtime-link=shared variant=debug address-model=64 install
```

This will install Boost's debug build in `E:\libs\boost\builds\win64-static-debug-cl`.

2. If the above doesn't work, then as per [StackOverflow](https://stackoverflow.com/questions/41464356/build-boost-with-msvc-14-1-vs2017-rc), open `E:\libs\boost\boost_1_62_0\project-config.jam` and replace the line `using msvc ;` with `using msvc : 15.0 : D:\Microsoft Visual Studio\2017\Community\VC\Tools\MSVC\14.10.25017\bin\HostX64\x64\cl.exe ;`.


### Release build

1. Compile. Compiling as shared library using the command `.\b2 --prefix=E:\libs\boost\builds\win64-shared-release-cl variant=release threading=multi link=shared install` causes some libraries to be created static and some to be shared. Hence for now, we'll just create static libraries with shared runtime C/C++ libraries.

```
.\b2 --prefix=E:\libs\boost\builds\win64-static-release-cl threading=multi link=static runtime-link=shared variant=release address-model=64 install
```

This will install Boost's release build in `E:\libs\boost\builds\win64-static-release-cl`.

2. If the above doesn't work, then as per [StackOverflow](https://stackoverflow.com/questions/41464356/build-boost-with-msvc-14-1-vs2017-rc), open `E:\libs\boost\boost_1_62_0\project-config.jam` and replace the line `using msvc ;` with `using msvc : 15.0 : D:\Microsoft Visual Studio\2017\Community\VC\Tools\MSVC\14.10.25017\bin\HostX64\x64\cl.exe ;`.


## Install VTK

1. Download VTK 7 from [its website](http://www.vtk.org/download/) to `E:\libs\vtk\VTK-7.1.1`. VTK 8 doesn't work with PCL yet.

### Debug build without Qt

1. Create `builds\win64-static-debug-cl` and change into it.

```
mkdir builds\win64-static-debug-cl
cd builds\win64-static-debug-cl
```

2. Compile. Make sure to disable compilation of Group_StandAlone otherwise it will compile the HDF5 library that's automatically downloaded inside VTK. Since we are disabling StandAlone group, we need to enable other modules inside it which are needed by PCL.

```
cmake -G "NMake Makefiles" -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=dist -DVTK_Group_StandAlone:BOOL=OFF -DModule_vtkIOImage:BOOL=ON -DModule_vtkIOParallelXML:BOOL=ON -DModule_vtkIOPLY:BOOL=ON -DModule_vtkIOImport:BOOL=ON -DModule_vtkIOGeometry:BOOL=ON -DModule_vtkDomainsChemistry:BOOL=ON ..\..\VTK-7.1.1
jom.exe
jom.exe install
```

### Release build without Qt

1. Create `builds\win64-static-release-cl` and change into it.

```
mkdir builds\win64-static-release-cl
cd builds\win64-static-release-cl
```

2. Compile. Make sure to disable compilation of Group_StandAlone otherwise it will compile the HDF5 library that's automatically downloaded inside VTK. Since we are disabling StandAlone group, we need to enable other modules inside it which are needed by PCL.

```
cmake -G "NMake Makefiles" -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=dist -DVTK_Group_StandAlone:BOOL=OFF -DModule_vtkIOImage:BOOL=ON -DModule_vtkIOParallelXML:BOOL=ON -DModule_vtkIOPLY:BOOL=ON -DModule_vtkIOImport:BOOL=ON -DModule_vtkIOGeometry:BOOL=ON -DModule_vtkDomainsChemistry:BOOL=ON ..\..\VTK-7.1.1
jom.exe
jom.exe install
```

### Debug build with Qt

1. Create `builds\win64-shared-debug-cl-qt` and change into it.

```
mkdir builds\win64-shared-debug-cl-qt
cd builds\win64-shared-debug-cl-qt
```

2. Compile. Make sure to disable compilation of Group_StandAlone otherwise it will compile the HDF5 library that's automatically downloaded inside VTK. Since we are disabling StandAlone group, we need to enable other modules inside it which are needed by PCL.

```
cmake -G "NMake Makefiles" -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=dist -DVTK_Group_Qt:BOOL=ON -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=E:\libs\qt\builds\win64-shared-debug\bin\qmake.exe -DQt5_DIR=E:\libs\qt\builds\win64-shared-debug\lib\cmake\Qt5 -DVTK_Group_StandAlone:BOOL=OFF -DModule_vtkIOImage:BOOL=ON -DModule_vtkIOParallelXML:BOOL=ON -DModule_vtkIOPLY:BOOL=ON -DModule_vtkIOImport:BOOL=ON -DModule_vtkIOGeometry:BOOL=ON -DModule_vtkDomainsChemistry:BOOL=ON ..\..\VTK-7.1.1
jom.exe
jom.exe install
```

### Release build with Qt

1. Create `builds\win64-shared-release-cl-qt` and change into it.

```
mkdir builds\win64-shared-release-cl-qt
cd builds\win64-shared-release-cl-qt
```

2. Compile. Make sure to disable compilation of Group_StandAlone otherwise it will compile the HDF5 library that's automatically downloaded inside VTK. Since we are disabling StandAlone group, we need to enable other modules inside it which are needed by PCL.

```
cmake -G "Ninja" -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=dist -DVTK_Group_Qt:BOOL=ON -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=E:\libs\qt\builds\win64-shared-release\bin\qmake.exe -DQt5_DIR=E:\libs\qt\builds\win64-shared-release\lib\cmake\Qt5 -DVTK_Group_StandAlone:BOOL=OFF -DModule_vtkIOImage:BOOL=ON -DModule_vtkIOParallelXML:BOOL=ON -DModule_vtkIOPLY:BOOL=ON -DModule_vtkIOImport:BOOL=ON -DModule_vtkIOGeometry:BOOL=ON -DModule_vtkDomainsChemistry:BOOL=ON ..\..\VTK-7.1.1
ninja.exe install
```


## Install FLANN

1. Download FLANN from [its website](http://www.cs.ubc.ca/research/flann/#download) to `E:\libs\flann`.

```
E:
cd libs
mkdir flann
cd flann
wget64.exe http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
"D:\Program Files\7-Zip\7z.exe" x flann-1.8.4-src.zip
del flann-1.8.4-src.zip
```

2. According to [github issue](https://github.com/chambbj/osgeo-superbuild/issues/3), fix the bug with `src\cpp\flann\util\serialization.h`. Open `src\cpp\flann\util\serialization.h` and around line 94, after all the other `BASIC_TYPE_SERIALIZER()` commands, add the following:

```
#ifdef _MSC_VER
BASIC_TYPE_SERIALIZER(unsigned __int64);
#endif
```


### Debug build

1. Create `builds/win64-debug-cl` and change into it.

```
mkdir -p builds\win64-debug-cl
cd builds\win64-debug-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug ..\..\flann-1.8.4-src\
nmake
nmake install
```

### Release build

1. Create `builds/win64-release-cl` and change into it.

```
mkdir -p builds\win64-release-cl
cd builds\win64-release-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release ..\..\flann-1.8.4-src\
nmake
nmake install
```


## Install Qhull

1. Clone Qhull from [github](https://github.com/qhull/qhull) to `E:\libs\qhull\src`.

```
E:
cd E:\libs
mkdir qhull
cd qhull
git clone https://github.com/qhull/qhull.git src
```

8. Since Qhull's static libraries are going to be used to build PCL's shared libraries, Qhull's static libraries need to be compiled using `-fPIC`. Open `E:\libs\qhull\src\CMakeLists.txt` and set `POSITION_INDEPENDENT_CODE` to `ON` for all the static libraries. Specifically on line 355, code should look as:

```bash
add_library(${qhull_STATIC} STATIC ${libqhull_SOURCES})
set_target_properties(${qhull_STATIC} PROPERTIES
    VERSION ${qhull_VERSION}
	POSITION_INDEPENDENT_CODE ON)

add_library(${qhull_STATICR} STATIC ${libqhullr_SOURCES})
set_target_properties(${qhull_STATICR} PROPERTIES
    VERSION ${qhull_VERSION}
	POSITION_INDEPENDENT_CODE ON)
```

On line 375, code should look like:

```bash
add_library(${qhull_CPP} STATIC ${libqhullcpp_SOURCES})
set_target_properties(${qhull_CPP} PROPERTIES
    VERSION ${qhull_VERSION}
	POSITION_INDEPENDENT_CODE ON)
```

### Debug build

1. Create `builds/win64-debug-cl` and change into it.

```
mkdir builds\win64-debug-cl
cd builds\win64-debug-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=OFF ../../src
nmake
nmake install
```

### Release build

1. Create `builds/win64-release-cl` and change into it.

```
mkdir builds\win64-release-cl
cd builds\win64-release-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=OFF ../../src
nmake
nmake install
```

## Install PCL

1. Clone PCL for [github](https://github.com/PointCloudLibrary/pcl) to `/Users/neeravbm/Documents/libs/pcl/src`.

```bash
mkdir /Users/neeravbm/Documents/libs/pcl
cd /Users/neeravbm/Documents/libs/pcl
git clone https://github.com/PointCloudLibrary/pcl.git src
```

2. PCL expects Qhull's debug and release libraries to be located in the same directory, but on Ubuntu, since we are building Qhull multiple times with different build types, this will not be the case. As a result, PCL is not able to find Qhull. Modify lines above `include(FindPackageHandleStandardArgs)` in `E:/libs/pcl/src/cmake/Modules/FindQhull.cmake`.

```bash
set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
if(QHULL_LIBRARY AND QHULL_LIBRARY_DEBUG)
  set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})
elseif(QHULL_LIBRARY)
  set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY})
else(QHULL_LIBRARY AND QHULL_LIBRARY_DEBUG)
  set(QHULL_LIBRARIES debug ${QHULL_LIBRARY_DEBUG})
  set(QHULL_LIBRARY ${QHULL_LIBRARY_DEBUG} CACHE FILEPATH "Qhull library path" FORCE)
  set(QHULL_LIBRARY_NAME ${QHULL_LIBRARY_DEBUG_NAME} CACHE STRING "Qhull library name" FORCE)
endif(QHULL_LIBRARY AND QHULL_LIBRARY_DEBUG)
```

3. D3D requires the function `pcl::ExtractIndices::applyFilter` to work with `pcl::PointNormal`. But `pcl::ExtractIndices` is not being instantiated in the library with `pcl::PointNormal` type. One option is to compile PCL using `-DPCL_NO_PRECOMPILE` flag but then compilation of `Analyzer.cpp` in D3D fails with the error that the number of objects has exceeded the maximum limit. This error can be fixed by adding `/bigobj` to compile flags as well as definitions using `add_definitions()`. But we haven't tried this approach fully yet. The easier option is: 

a. Open `E:\libs\pcl\src\filters\src\extract_indices.cpp`
b. Go to the bottom and replace the line `PCL_INSTANTIATE(ExtractIndices, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::Normal)(pcl::PointXYZRGBNormal))` with `PCL_INSTANTIATE(ExtractIndices, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::Normal)(pcl::PointXYZRGBNormal)(pcl::PointNormal)(pcl::ReferenceFrame)(pcl::SHOT352)(pcl::FPFHSignature33))`. Note that we are adding `(pcl::PointNormal)(pcl::ReferenceFrame)(pcl::SHOT352)(pcl::FPFHSignature33)` at the end.
c. While executing cmake command, make sure to add the options `-DPCL_NO_PRECOMPILE:BOOL=OFF -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON`.

4. RepMat requires GeometricConsistencyGrouping to work with point type `PointXYZRGB`. 

a. Open `E:\libs\pcl\src\recognition\src\cg\geometric_consistency.cpp`.
b. Go to the bottom and replace the line `PCL_INSTANTIATE_PRODUCT(GeometricConsistencyGrouping, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA))((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)))` with `PCL_INSTANTIATE_PRODUCT(GeometricConsistencyGrouping, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB))((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)))`. Note that we are adding `(pcl::PointXYZRGB)` twice, once in the middle and once at the end.
c. While executing cmake command, make sure to add the options `-DPCL_NO_PRECOMPILE:BOOL=OFF -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON`.


5. IndoorReconstruction requires NormalEstimation and NormalEstimationOMP to work with point type `PointXYZRGBNormal,Normal1`.

a. Open `E:\libs\pcl\pcl_forked\features\src\normal_3d.cpp`.
b. Go to the bottom and below the line `#ifdef PCL_ONLY_CORE_POINT_TYPES`, remove the existing two lines and add the following:
```
  PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal)(pcl::PointXYZRGBNormal))((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)))
  PCL_INSTANTIATE_PRODUCT(NormalEstimationOMP, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal)(pcl::PointXYZRGBNormal))((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)))
```
Note that we are adding `(pcl::PointXYZRGBNormal)` inside the first bracket in both the lines.

6. ScanToBIMLib requires PassThrough filter to work with point type `pcl::Normal`.

a. Open `E:\libs\pcl\src\filters\src\passthrough.cpp`.
b. Near the bottom, below the line `PCL_INSTANTIATE(PassThrough, PCL_XYZ_POINT_TYPES)`, add the line `PCL_INSTANTIATE(PassThrough, (pcl::Normal))`.
c. Open `E:\libs\pcl\src\filters\include\pcl\filters\impl\passthrough.hpp`.
d. Below `#include <pcl/common/io.h>`, add the following code:

```bash
namespace pcl {

  template <typename PointT, class Enable = void>
  struct PassthroughHelper {
    static void resetRemovedIndicesXYZValues(typename pcl::PointCloud<PointT> &output, pcl::IndicesPtr removed_indices_, float user_filter_value_) {}

    static bool passNonFiniteEntriesToRemovedIndices(typename pcl::PointCloud<PointT>::ConstPtr input_, pcl::IndicesPtr indices_, pcl::IndicesPtr removed_indices_, bool extract_removed_indices_, int iii, int* rii) {
      return true;
    }
  };

  template <typename PointT>
  struct PassthroughHelper<PointT, typename std::enable_if<pcl::traits::has_xyz<PointT>::value>::type> {
    static void resetRemovedIndicesXYZValues(typename pcl::PointCloud<PointT> &output, pcl::IndicesPtr removed_indices_, float user_filter_value_) {
      for (int rii = 0; rii < static_cast<int> (removed_indices_->size ()); ++rii)  // rii = removed indices iterator
        output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
    }

    static bool passNonFiniteEntriesToRemovedIndices(typename pcl::PointCloud<PointT>::ConstPtr input_, pcl::IndicesPtr indices_, pcl::IndicesPtr removed_indices_, bool extract_removed_indices_, int iii, int* rii) {
      if (!pcl_isfinite (input_->points[(*indices_)[iii]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[*rii++] = (*indices_)[iii];
        return false;
      }

      return true;
    }
  };
}
```

e. In the function `applyFilter()`, comment out the following two lines:

```
    for (int rii = 0; rii < static_cast<int> (removed_indices_->size ()); ++rii)  // rii = removed indices iterator
      output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
```

and instead, add the following line in its place:

```
pcl::PassthroughHelper<PointT>::resetRemovedIndicesXYZValues(output, removed_indices_, user_filter_value_);
```

f. In the function `applyFilterIndices()`, comment out the following lines:

```
      if (!pcl_isfinite (input_->points[(*indices_)[iii]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[iii];
        continue;
      }
```

and instead add the following code:

```
      if (!pcl::PassthroughHelper<PointT>::passNonFiniteEntriesToRemovedIndices(input_, indices_, removed_indices_, extract_removed_indices_, iii, &rii)) continue;
```

Note that the above code appears twice in the same function and both those instances need to be replaced.


7. We need to make region growing multi-threaded.

a. Open `E:\libs\pcl\src\segmentation\include\pcl\segmentation\imp\region_growing.hpp`.
b. At the top below `#include <time.h>`, add the following code:

```
#ifdef _OPENMP
#include <omp.h>
#endif
```

c. In function `findPointNeighbors()`, add OpenMP. Replace the following code:

```
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->points.size (), neighbours);
  if (input_->is_dense)
  {
    #pragma omp parallel for
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      int point_index = (*indices_)[i_point];
      neighbours.clear ();
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
  else
  {
    #pragma omp parallel for
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      neighbours.clear ();
      int point_index = (*indices_)[i_point];
      if (!pcl::isFinite (input_->points[point_index]))
        continue;
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
}
```

with the following code:

```
template <typename PointT, typename NormalT> void
pcl::RegionGrowing<PointT, NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->points.size (), neighbours);
  if (input_->is_dense)
  {
    #pragma omp parallel for private(neighbours)
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      int point_index = (*indices_)[i_point];
      neighbours.clear ();
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
  else
  {
    #pragma omp parallel for private(neighbours)
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      neighbours.clear ();
      int point_index = (*indices_)[i_point];
      if (!pcl::isFinite (input_->points[point_index]))
        continue;
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
}

```

d. In the function `applySmoothRegionGrowingAlgorithm()`, add OpenMP. Replace the following code:

```
  if (normal_flag_ == true)
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = normals_->points[point_index].curvature;
      point_residual[i_point].second = point_index;
    }
    std::sort (point_residual.begin (), point_residual.end (), comparePair);
  }
  else
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = 0;
      point_residual[i_point].second = point_index;
    }
  }
```

with

```
  if (normal_flag_ == true)
  {
    #pragma omp parallel for schedule(static, 1024)
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = normals_->points[point_index].curvature;
      point_residual[i_point].second = point_index;
    }
    std::sort (point_residual.begin (), point_residual.end (), comparePair);
  }
  else
  {
    #pragma omp parallel for schedule(static, 1024)
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = 0;
      point_residual[i_point].second = point_index;
    }
  }
```


7. Open `E:\libs\pcl\src\features\include\pcl\features\impl\board.hpp` and add OpenMP for loop in `computeFeature()` function. It should look like:

```
  #pragma omp parallel for
  for (int point_idx = 0; point_idx < indices_->size (); ++point_idx)
  {
    Eigen::Matrix3f currentLrf;
    PointOutT &rf = output[point_idx];

    //rf.confidence = computePointLRF (*indices_[point_idx], currentLrf);
    //if (rf.confidence == std::numeric_limits<float>::max ())
    if (computePointLRF ((*indices_)[point_idx], currentLrf) == std::numeric_limits<float>::max ())
    {
	  // Visual Studio C++ doesn't support OpenMP3, which means critical block is not supported.
      //#pragma omp critical outputDenseUpdate
      //{
        output.is_dense = false;
      //}
    }

    for (int d = 0; d < 3; ++d)
    {
      rf.x_axis[d] = currentLrf (0, d);
      rf.y_axis[d] = currentLrf (1, d);
      rf.z_axis[d] = currentLrf (2, d);
    }
  }
```

At the top of the file, make sure to add 

```
#ifdef _OPENMP
#include <omp.h>
#endif
```

5. On line 477 of `E:\libs\pcl\src\features\include\pcl\features\impl\board.hpp`, after `float max_hole_prob = -std::numeric_limits<float>::max ();`, inside the function `computePointLRF()`, add:

```
  if (first_no_border < 0) first_no_border = 0;
```

6. DO NOT DO THIS. IT SLOWS DOWN THE ALGORITHM. Open `E:\libs\pcl\src\recognition\include\pcl\recognition\impl\cg\geometric_consistency.hpp`. In function `clusterCorrespondences (std::vector<Correspondences> &model_instances)`, there is a `for` loop on line 104. It looks like:

```
        for (size_t k = 0; k < consensus_set.size (); ++k)
        {
          int scene_index_k = model_scene_corrs_->at (consensus_set[k]).index_match;
          int model_index_k = model_scene_corrs_->at (consensus_set[k]).index_query;
          int scene_index_j = model_scene_corrs_->at (j).index_match;
          int model_index_j = model_scene_corrs_->at (j).index_query;
          
          const Eigen::Vector3f& scene_point_k = scene_->at (scene_index_k).getVector3fMap ();
          const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();
          const Eigen::Vector3f& scene_point_j = scene_->at (scene_index_j).getVector3fMap ();
          const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

          dist_ref = scene_point_k - scene_point_j;
          dist_trg = model_point_k - model_point_j;

          double distance = fabs (dist_ref.norm () - dist_trg.norm ());

          if (distance > gc_size_)
          {
            is_a_good_candidate = false;
            break;
          }
        }

```

Change the above `for` loop to:

```
#pragma omp parallel for
        for (int k = 0; k < consensus_set.size (); ++k)
        {
#pragma omp flush (is_a_good_candidate)
          if (is_a_good_candidate) {
            int scene_index_k = model_scene_corrs_->at (consensus_set[k]).index_match;
            int model_index_k = model_scene_corrs_->at (consensus_set[k]).index_query;
            int scene_index_j = model_scene_corrs_->at (j).index_match;
            int model_index_j = model_scene_corrs_->at (j).index_query;

            const Eigen::Vector3f& scene_point_k = scene_->at (scene_index_k).getVector3fMap ();
            const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();
            const Eigen::Vector3f& scene_point_j = scene_->at (scene_index_j).getVector3fMap ();
            const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

            dist_ref = scene_point_k - scene_point_j;
            dist_trg = model_point_k - model_point_j;

            double distance = fabs (dist_ref.norm () - dist_trg.norm ());

            if (distance > gc_size_)
            {
              is_a_good_candidate = false;
#pragma omp flush (is_a_good_candidate)
            }
          }
        }
```

7. DO NOT DO THIS. IT SLOWS DOWN THE ALGORITHM. Open `E:\libs\pcl\src\sample_consensus\include\pcl\sample_consensus\impl\ransac.hpp`. In function `computeModel(int)`, there is a `while` loop. It looks like:

```
  while (iterations_ < k && skipped_count < max_skip)
  {
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);

    if (selection.empty ()) 
    {
      PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
      break;
    }

    // Search for inliers in the point cloud for the current plane model M
    if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
    {
      //++iterations_;
      ++skipped_count;
      continue;
    }

    // Select the inliers that are within threshold_ from the model
    //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
    //if (inliers.empty () && k > 1.0)
    //  continue;

    n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);

    // Better match ?
    if (n_inliers_count > n_best_inliers_count)
    {
      n_best_inliers_count = n_inliers_count;

      // Save the current model/inlier/coefficients selection as being the best so far
      model_              = selection;
      model_coefficients_ = model_coefficients;

      // Compute the k parameter (k=log(z)/log(1-w^n))
      double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
      double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
      p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = log_probability / log (p_no_outliers);
    }

    ++iterations_;
    PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
    if (iterations_ > max_iterations_)
    {
      PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
      break;
    }
  }
```

Change the above to:

```
  if (sac_model_->getIndices()->size() < sac_model_->getSampleSize()) {
    PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] Can not select %lu unique points out of %lu!\n",
               sac_model_->getSampleSize(), sac_model_->getIndices()->size());
  }
  else {
    bool continue_iterations = true;
#pragma omp parallel for reduction(+:skipped_count)
    for (int i = 0; i < max_iterations_; i++) {
#pragma omp flush (continue_iterations)
      if (continue_iterations) {
	  	std::vector<int> selection;
			
        // Get X samples which satisfy the model criteria
        sac_model_->getSamples (iterations_, selection);

        if (selection.empty ())
        {
          PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
          continue_iterations = false;
#pragma omp flush (continue_iterations)
          continue;
        }

        // Search for inliers in the point cloud for the current plane model M
        if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
        {
          //++iterations_;
          ++skipped_count;
          continue;
        }

        // Select the inliers that are within threshold_ from the model
        //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
        //if (inliers.empty () && k > 1.0)
        //  continue;

        n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);

        // Better match ?
#pragma omp critical (update_best_ransac_model)
        {
          if (n_inliers_count > n_best_inliers_count)
          {
            n_best_inliers_count = n_inliers_count;

            // Save the current model/inlier/coefficients selection as being the best so far
            model_              = selection;
            model_coefficients_ = model_coefficients;

            // Compute the k parameter (k=log(z)/log(1-w^n))
            double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
            double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
            p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
            p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
            k = log_probability / log (p_no_outliers);
          }

          ++iterations_;
        }

        PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
        if (iterations_ > max_iterations_)
        {
          PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
          continue_iterations = false;
#pragma omp flush (continue_iterations)
        }
        else if (iterations_ >= k || skipped_count >= max_skip) {
          continue_iterations = false;
#pragma omp flush (continue_iterations)
        }
      }
    }
  }
```

Also delete the definition of `std::vector<int> selection` from line 61 above.

8. Open `E:\libs\pcl\src\features\include\pcl\features\normal_3d.h`. Function `computePointNormal()` on line 60 looks like:

```
  template <typename PointT> inline bool
  computePointNormal (const pcl::PointCloud<PointT> &cloud,
                      Eigen::Vector4f &plane_parameters, float &curvature)
  {
    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    if (cloud.size () < 3 ||
        computeMeanAndCovarianceMatrix (cloud, covariance_matrix, xyz_centroid) == 0)
    {
      plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
      curvature = std::numeric_limits<float>::quiet_NaN ();
      return false;
    }

    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);
    return true;
  }
```

Change the above to

```
  template <typename PointT> inline bool
  computePointNormal (const pcl::PointCloud<PointT> &cloud,
                      Eigen::Vector4f &plane_parameters, float &curvature)
  {
    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;
	compute3DCentroid(cloud, xyz_centroid);
    if (cloud.size () < 3 ||
        computeCovarianceMatrixNormalized (cloud, xyz_centroid, covariance_matrix) == 0)
    {
      plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
      curvature = std::numeric_limits<float>::quiet_NaN ();
      return false;
    }

    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix.cast<float>(), xyz_centroid.cast<float>(), plane_parameters, curvature);
    return true;
  }
```

9. Open `E:\libs\pcl\src\features\include\pcl\features\normal_3d.h`. Function `computePointNormal()` on line 92 looks like:

```
  template <typename PointT> inline bool
  computePointNormal (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                      Eigen::Vector4f &plane_parameters, float &curvature)
  {
    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    if (indices.size () < 3 ||
        computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix, xyz_centroid) == 0)
    {
      plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
      curvature = std::numeric_limits<float>::quiet_NaN ();
      return false;
    }
    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix.cast<float>(), xyz_centroid.cast<float>(), plane_parameters, curvature);
    return true;
  }
```

Change the above to

```
  template <typename PointT> inline bool
  computePointNormal (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                      Eigen::Vector4f &plane_parameters, float &curvature)
  {
    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;
	compute3DCentroid(cloud, indices, xyz_centroid);
    if (indices.size () < 3 ||
        computeCovarianceMatrixNormalized(cloud, indices, xyz_centroid, covariance_matrix) == 0)
    {
      plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
      curvature = std::numeric_limits<float>::quiet_NaN ();
      return false;
    }
    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix.cast<float>(), xyz_centroid.cast<float>(), plane_parameters, curvature);
    return true;
  }
```

10. Open `E:\libs\pcl\src\sample_consensus\include\pcl\sample_consensus\sac_model_registration.h`. Function computeSampleDistanceThreshold() on line 265 looks like:

```
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        computeMeanAndCovarianceMatrix (*cloud, indices, covariance_matrix, xyz_centroid);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }
```

Change the above to

```
      inline void
      computeSampleDistanceThreshold (const PointCloudConstPtr &cloud,
                                      const std::vector<int> &indices)
      {
        // Compute the principal directions via PCA
        Eigen::Vector4d xyz_centroid;
        Eigen::Matrix3d covariance_matrix;
		compute3DCentroid(*cloud, indices, xyz_centroid);
		computeCovarianceMatrixNormalized(*cloud, indices, xyz_centroid, covariance_matrix);

        // Check if the covariance matrix is finite or not.
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix.coeffRef (i, j)))
              PCL_ERROR ("[pcl::SampleConsensusModelRegistration::computeSampleDistanceThreshold] Covariance matrix has NaN values! Is the input cloud finite?\n");

        Eigen::Vector3f eigen_values;
        pcl::eigen33 (covariance_matrix, eigen_values);

        // Compute the distance threshold for sample selection
        sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
        sample_dist_thresh_ *= sample_dist_thresh_;
        PCL_DEBUG ("[pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample selection distance threshold of: %f\n", sample_dist_thresh_);
      }
```


10. Open `E:\libs\pcl\src\kdtree\include\pcl\kdtree\impl\kdtree_flann.hpp`. In function `radiusSearch()`, at line 193, just below `params.max_neighbors = max_nn;`, add the following:

```
#ifdef _OPENMP
  params.cores = omp_get_max_threads();
#endif // _OPENMP
```

At the top of the file, add:

```
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
```

This ensures that radius search uses all the cores possible instead of just one.


11. Later versions of VC++ compiler has a bug which results in an error `'pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type::callbacks_element<U1>': cannot access private struct declared in class 'pcl::io::ply::ply_parser::scalar_property_definition_callbacks_type'`. As per [Spurious compiler error webpage](https://developercommunity.visualstudio.com/content/problem/70100/spurious-compiler-error-complaining-about-nested-t.html), make the `callbacks_element` member of the `list_property_definition_callbacks_type` and `scalar_property_definition_callbacks_type` classes as public instead of private in the file `E:/libs/pcl/src/io/include/pcl/io/ply/ply_parser.h`.

On line 117, following should be the definition of the class `scalar_property_definition_callbacks_type`.

```          
          class scalar_property_definition_callbacks_type
          {
            public:
              template <typename T>
              struct callbacks_element
              {
//                callbacks_element () : callback ();
                typedef T scalar_type;
                typename scalar_property_definition_callback_type<scalar_type>::type callback;
              };

            private:
              typedef boost::mpl::inherit_linearly<
                scalar_types,
                boost::mpl::inherit<
                boost::mpl::_1,
                callbacks_element<boost::mpl::_2>
                >
                >::type callbacks;
              callbacks callbacks_;

            public:
              template <typename ScalarType>
              const typename scalar_property_definition_callback_type<ScalarType>::type&
              get () const
              {
                return (static_cast<const callbacks_element<ScalarType>&> (callbacks_).callback);
              }

              template <typename ScalarType>
              typename scalar_property_definition_callback_type<ScalarType>::type&
              get ()
              {
                return (static_cast<callbacks_element<ScalarType>&> (callbacks_).callback);
              }

              template <typename ScalarType>
              friend typename scalar_property_definition_callback_type<ScalarType>::type&
              at (scalar_property_definition_callbacks_type& scalar_property_definition_callbacks);

              template <typename ScalarType>
              friend const typename scalar_property_definition_callback_type<ScalarType>::type&
              at (const scalar_property_definition_callbacks_type& scalar_property_definition_callbacks);
          };
```

On line 210, following should be the definition of the class `list_property_definition_callbacks_type`:

```
          class list_property_definition_callbacks_type
          {
            public:
              template <typename T>
              struct callbacks_element
              {
                typedef typename T::first size_type;
                typedef typename T::second scalar_type;
                typename list_property_definition_callback_type<size_type, scalar_type>::type callback;
              };

            private:
              template <typename T> struct pair_with : boost::mpl::pair<T,boost::mpl::_> {};
              template<typename Sequence1, typename Sequence2>

                struct sequence_product :
                  boost::mpl::fold<Sequence1, boost::mpl::vector0<>,
                    boost::mpl::joint_view<
                      boost::mpl::_1,boost::mpl::transform<Sequence2, pair_with<boost::mpl::_2> > > >
                {};

              typedef boost::mpl::inherit_linearly<sequence_product<size_types, scalar_types>::type, boost::mpl::inherit<boost::mpl::_1, callbacks_element<boost::mpl::_2> > >::type callbacks;
              callbacks callbacks_;

            public:
              template <typename SizeType, typename ScalarType>
              typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              get ()
              {
                return (static_cast<callbacks_element<boost::mpl::pair<SizeType, ScalarType> >&> (callbacks_).callback);
              }

              template <typename SizeType, typename ScalarType>
              const typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              get () const
              {
                return (static_cast<const callbacks_element<boost::mpl::pair<SizeType, ScalarType> >&> (callbacks_).callback);
              }

              template <typename SizeType, typename ScalarType>
              friend typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              at (list_property_definition_callbacks_type& list_property_definition_callbacks);

              template <typename SizeType, typename ScalarType>
              friend const typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              at (const list_property_definition_callbacks_type& list_property_definition_callbacks);
          };
```


### Debug build without Qt

1. Create `builds\win64-static-debug-cl` and change into it.

```
mkdir builds\win64-static-debug-cl
cd builds\win64-static-debug-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DEIGEN_ROOT=E:\libs\Eigen\builds\win64-debug-cl\dist\include -DFLANN_ROOT=E:\libs\flann\builds\win64-debug-cl\dist -DQHULL_USE_STATIC:BOOL=ON -DQHULL_ROOT=E:\libs\qhull\builds\win64-debug-cl\dist\ -DQHULL_INCLUDE_DIR=E:\libs\qhull\builds\win64-debug-cl\dist\include\ -DVTK_DIR=E:\libs\vtk\builds\win64-static-debug-cl -DBOOST_ROOT=E:\libs\boost\builds\win64-static-cl\ -DBOOST_LIBRARYDIR=E:\libs\boost\builds\win64-static-cl\lib\ -DBoost_COMPILER=-vc141 -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_examples:BOOL=ON ..\..\src\
jom.exe
jom.exe install
```


### Release build without Qt

1. Create `builds\win64-static-release-cl` and change into it.

```
mkdir builds\win64-static-release-cl
cd builds\win64-static-release-cl
```

2. Compile.

```
cmake -G "NMake Makefiles" -DEIGEN_ROOT=E:\libs\Eigen\builds\win64-release-cl \dist\include -DFLANN_ROOT=E:\libs\flann\builds\win64-release-cl\dist -DQHULL_USE_STATIC:BOOL=ON -DQHULL_ROOT=E:\libs\qhull\builds\win64-release-cl\dist\ -DQHULL_INCLUDE_DIR=E:\libs\qhull\builds\win64-release-cl\dist\include\ -DVTK_DIR=E:\libs\vtk\builds\win64-static-release-cl -DBOOST_ROOT=E:\libs\boost\builds\win64-cl\ -DBOOST_LIBRARYDIR=E:\libs\boost\builds\win64-cl\lib\ -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_examples:BOOL=ON ..\..\src\
jom.exe
jom.exe install
```


### Debug build with Qt

1. Create `builds\win64-shared-debug-cl-qt` and change into it.

```
mkdir builds\win64-shared-debug-cl-qt
cd builds\win64-shared-debug-cl-qt
```

2. Compile.

```
cmake -G "Ninja" -DEIGEN_ROOT=E:/libs/Eigen/builds/win64-debug-cl/dist/include -DFLANN_ROOT=E:/libs/flann/builds/win64-debug-cl/dist -DQHULL_USE_STATIC:BOOL=ON -DQHULL_ROOT=E:/libs/qhull/builds/win64-debug-cl/dist -DQHULL_INCLUDE_DIR=E:/libs/qhull/builds/win64-debug-cl/dist/include -DVTK_DIR=E:/libs/vtk/builds/win64-shared-debug-cl-qt -DBOOST_ROOT=E:/libs/boost/builds/win64-static-debug-cl -DBOOST_LIBRARYDIR=E:/libs/boost/builds/win64-static-debug-cl/lib -DBoost_COMPILER=-vc141 -DQt5_DIR=E:/libs/qt/builds/win64-shared-debug/lib/cmake/Qt5 -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=ON -DBUILD_examples:BOOL=ON -DPCL_NO_PRECOMPILE:BOOL=OFF -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON ../../src
ninja.exe install
```


### Release build with Qt

1. Create `builds\win64-static-release-cl-qt` and change into it.

```
mkdir builds\win64-static-release-cl-qt
cd builds\win64-static-release-cl-qt
```

2. Compile.

```
cmake -G "Ninja" -DEIGEN_ROOT=E:/libs/Eigen/builds/win64-release-cl/dist/include -DFLANN_ROOT=E:/libs/flann/builds/win64-release-cl/dist -DQHULL_USE_STATIC:BOOL=ON -DQHULL_ROOT=E:/libs/qhull/builds/win64-release-cl/dist -DQHULL_INCLUDE_DIR=E:/libs/qhull/builds/win64-release-cl/dist/include -DVTK_DIR=E:/libs/vtk/builds/win64-shared-release-cl-qt -DBOOST_ROOT=E:/libs/boost/builds/win64-static-release-cl -DBOOST_LIBRARYDIR=E:/libs/boost/builds/win64-static-release-cl/lib -DBoost_COMPILER=-vc141 -DQt5_DIR=E:/libs/qt/builds/win64-shared-release/lib/cmake/Qt5 -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=ON -DBUILD_examples:BOOL=ON -DPCL_NO_PRECOMPILE:BOOL=OFF -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON ../../src
ninja.exe install
```

### Static Debug build with Qt

1. Create `builds\win64-static-debug-cl-qt` and change into it.

```
mkdir builds\win64-static-debug-cl-qt
cd builds\win64-static-debug-cl-qt
```

2. Compile.

```
cmake -G "NMake Makefiles" -DEIGEN_ROOT=E:\libs\Eigen\builds\win64-debug-cl\dist\include -DFLANN_ROOT=E:\libs\flann\builds\win64-debug-cl\dist -DQHULL_USE_STATIC:BOOL=ON -DQHULL_ROOT=E:\libs\qhull\builds\win64-debug-cl\dist\ -DQHULL_INCLUDE_DIR=E:\libs\qhull\builds\win64-debug-cl\dist\include\ -DVTK_DIR=E:\libs\vtk\builds\win64-shared-debug-cl-qt -DBOOST_ROOT=E:\libs\boost\builds\win64-static-cl\ -DBOOST_LIBRARYDIR=E:\libs\boost\builds\win64-static-cl\lib\ -DBoost_COMPILER=-vc141 -DQt5_DIR=E:/libs/qt/builds/win64-shared-debug/lib/cmake/Qt5 -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=OFF -DPCL_SHARED_LIBS:BOOL=OFF -DBUILD_examples:BOOL=ON ..\..\src\
jom.exe
jom.exe install
```

## Install 

1. Clone OpenCV [git repo](https://github.com/opencv/opencv) in `E:\libs\OpenCV\src`. 

```bash
mkdir E:\libs\OpenCV
E:
cd E:\libs\OpenCV
git clone https://github.com/opencv/opencv.git src
```

2. There is an error in the file `cmake/OpenCVFindLibsPerf.cmake`. Remove `/include` after `$ENV{EIGEN_ROOT}`. The command should be:

```bash
  find_path(EIGEN_INCLUDE_PATH "Eigen/Core"
            PATHS /usr/local /opt /usr $ENV{EIGEN_ROOT} ENV ProgramFiles ENV ProgramW6432
            PATH_SUFFIXES include/eigen3 include/eigen2 Eigen/include/eigen3 Eigen/include/eigen2
            DOC "The path to Eigen3/Eigen2 headers"
            CMAKE_FIND_ROOT_PATH_BOTH)
```


### Debug build

1. Create the environment variable `EIGEN_ROOT`:

```
set EIGEN_ROOT=E:\libs\Eigen\builds\win64-debug-cl\dist
```

2. Create a directory `builds\win64-shared-debug-cl` in `E:\libs\OpenCV` and change into it.

```
mkdir builds/win64-shared-debug-cl
cd builds\win64-shared-debug-cl
```

3. Compile.

D3D needs OpenCV's `core`, `photo` and `highhui` modules only so that `Histogram2D`, `Histogram3D` and `DBSCAN3D` can be compiled. Hence we'll disable all modules except these.

```
cmake -G "Ninja" -DCMAKE_INSTALL_PREFIX=dist -DWITH_EIGEN:BOOL=ON -DWITH_OPENGL:BOOL=OFF -DWITH_QT:BOOL=OFF -DWITH_OPENMP:BOOL=ON -DWITH_VTK:BOOL=OFF -DWITH_CUDA:BOOL=OFF -DWITH_OPENCL:BOOL=OFF -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=ON -DBUILD_opencv_objdetect:BOOL=OFF -DBUILD_opencv_video:BOOL=OFF -DBUILD_opencv_dnn:BOOL=OFF -DBUILD_opencv_shape:BOOL=OFF -DBUILD_opencv_videoio:BOOL=OFF -DBUILD_opencv_superres:BOOL=OFF -DBUILD_opencv_ts:BOOL=OFF -DBUILD_opencv_flann:BOOL=OFF -DBUILD_opencv_ml:BOOL=OFF ..\..\src\
ninja.exe
ninja.exe install
```

4. unset `EIGEN_ROOT` environment variable.

```
set EIGEN_ROOT=
```

### Release build

1. Create the environment variable `EIGEN_ROOT`:

```
set EIGEN_ROOT=E:\libs\Eigen\builds\win64-release-cl\dist
```

2. Create a directory `builds\win64-shared-release-cl` in `E:\libs\OpenCV` and change into it.

```
mkdir builds/win64-shared-release-cl
cd builds\win64-shared-release-cl
```

3. Compile.

D3D needs OpenCV's `core`, `photo` and `highhui` modules only so that `Histogram2D`, `Histogram3D` and `DBSCAN3D` can be compiled. Hence we'll disable all modules except these.

```
cmake -G "Ninja" -DCMAKE_INSTALL_PREFIX=dist -DWITH_EIGEN:BOOL=ON -DWITH_OPENGL:BOOL=OFF -DWITH_QT:BOOL=OFF -DWITH_OPENMP:BOOL=ON -DWITH_VTK:BOOL=OFF -DWITH_CUDA:BOOL=OFF -DWITH_OPENCL:BOOL=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=ON -DBUILD_opencv_objdetect:BOOL=OFF -DBUILD_opencv_video:BOOL=OFF -DBUILD_opencv_dnn:BOOL=OFF -DBUILD_opencv_shape:BOOL=OFF -DBUILD_opencv_videoio:BOOL=OFF -DBUILD_opencv_superres:BOOL=OFF -DBUILD_opencv_ts:BOOL=OFF -DBUILD_opencv_flann:BOOL=OFF -DBUILD_opencv_ml:BOOL=OFF ..\..\src\
ninja.exe
ninja.exe install
```

4. unset `EIGEN_ROOT` environment variable.

```
set EIGEN_ROOT=
```



## Install NLOpt

1. Create a directory `E:\libs\nlopt`.

2. Download NLOpt from http://ab-initio.mit.edu/nlopt/nlopt-2.4.2.tar.gz.

3. Unzip and untar above to `E:\libs\nlopt\nlopt-2.4.2`.

4. According to https://nlopt.readthedocs.io/en/latest/NLopt_on_Windows/, there exists CMakeLists.txt and config.cmake.h.in file for Windows. Download them from http://ab-initio.mit.edu/nlopt/CMakeLists.txt and http://ab-initio.mit.edu/nlopt/config.cmake.h.in respectively.

### Debug build

1. Create directory `builds\win64-shared-debug-cl` in `E:\libs\nlopt` and change into it.

```
mkdir builds/win64-shared-debug-cl
cd builds\win64-shared-debug-cl
```

2. Compile and install.

```
cmake -G "Ninja" -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_INSTALL_PREFIX=dist ..\..\nlopt-2.4.2\
ninja.exe install
```

### Release build

1. Create directory `builds\win64-shared-release-cl` in `E:\libs\nlopt` and change into it.

```
mkdir builds/win64-shared-release-cl
cd builds\win64-shared-release-cl
```

2. Compile and install.

```
cmake -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_INSTALL_PREFIX=dist ..\..\nlopt-2.4.2\
ninja.exe install
```



## Install ApproxMVBB