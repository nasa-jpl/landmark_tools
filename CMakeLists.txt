cmake_minimum_required(VERSION 3.16)
project(landmark_tools)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Configuring on/for Linux")
    add_definitions(-DLINUX_OS)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "Configuring on/for Windows")
    add_definitions(-DWINDOWS_OS)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    # Darwin is the system name for macOS
    message(STATUS "Configuring on/for macOS")
    add_definitions(-DMAC_OS)
endif()

# Add local CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(GSL REQUIRED)
message(STATUS "GSL_FOUND: ${GSL_FOUND}")
message(STATUS "GSL_INCLUDE_DIRS: ${GSL_INCLUDE_DIRS}")
message(STATUS "GSL_LIBRARIES: ${GSL_LIBRARIES}")
message(STATUS "GSL_VERSION: ${GSL_VERSION}")

find_package(GDAL)
if(GDAL_FOUND)
    add_definitions(-DUSE_GEOTIFF)
    include_directories(${GDAL_INCLUDE_DIRS})
    set(gdal_sources
    src/landmark_tools/image_io/geotiff_interface.c
    )
endif()

find_package(yaml)
if(NOT yaml_FOUND)
find_library(yaml_LIBRARIES
    yaml
    HINTS /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/homebrew/Cellar/libyaml/0.2.5
    REQUIRED
)
message(STATUS "Using yaml library: ${yaml_LIBRARIES}")
else()
message(STATUS "Used config file: ${yaml_CONFIG}")
endif()

include(CMakeLinkHeaders.cmake)
add_custom_target(link_public_headers)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/img)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/math)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/landmark_tools)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include
submodules/
submodules/librply/src/lib/
${GSL_INCLUDE_DIRS}
${PNG_INCLUDE_DIRS})


# OpenCV (optional, only used for 2D-to-2D matching for now)
option(WITH_OPENCV "Enable OpenCV-dependent components" OFF)
if (WITH_OPENCV)
    add_definitions(-DWITH_OPENCV=1)
    message(STATUS "Using OpenCV")
    find_package(OpenCV 4 QUIET)
    if(NOT OpenCV_FOUND)
      message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
      set(_opencv_version 3)
    endif()
    find_package(OpenCV ${_opencv_version} REQUIRED)
else()
    message(STATUS "Not using OpenCV; run CMake with -DWITH_OPENCV=ON to enable")
endif()

set(common_sources
src/landmark_tools/data_interpolation/interpolate_data.c
src/landmark_tools/image_io/image_utils.c
src/landmark_tools/landmark_util/landmark.c
src/landmark_tools/map_projection/datum_conversion.c
src/landmark_tools/math/double_matrix.c
src/landmark_tools/math/math_utils.c
src/landmark_tools/math/point_line_plane_util.c
src/landmark_tools/utils/parse_args.c
src/landmark_tools/utils/endian_read_write.c
src/math/mat3/mat3.c
)

add_executable( create_landmark
src/main/create_landmark_main.c
${common_sources}
${gdal_sources}
src/landmark_tools/landmark_util/create_landmark.c
src/landmark_tools/map_projection/equidistant_cylindrical_projection.c
src/landmark_tools/map_projection/lambert.c
src/landmark_tools/map_projection/stereographic_projection.c
src/landmark_tools/map_projection/utm.c
src/landmark_tools/utils/two_level_yaml_parser.c
submodules/librply/src/lib/rply.c
)
add_dependencies(create_landmark link_public_headers)

add_executable( create_landmark_from_img
src/main/create_landmark_from_img_main.c
${common_sources}
${gdal_sources}
src/landmark_tools/landmark_util/create_landmark.c
src/landmark_tools/map_projection/equidistant_cylindrical_projection.c
src/landmark_tools/map_projection/lambert.c
src/landmark_tools/map_projection/stereographic_projection.c
src/landmark_tools/map_projection/utm.c
src/landmark_tools/utils/two_level_yaml_parser.c
submodules/librply/src/lib/rply.c
)
add_dependencies(create_landmark_from_img link_public_headers)

add_executable( landmark_comparison
src/main/landmark_comparison_main.c
${common_sources}
src/landmark_tools/landmark_util/estimate_homography.c
src/landmark_tools/feature_tracking/feature_match.c
src/landmark_tools/feature_tracking/corr_image_long.c
src/landmark_tools/feature_tracking/parameters.c
src/landmark_tools/math/homography_util.c
src/landmark_tools/utils/two_level_yaml_parser.c
)
add_dependencies(landmark_comparison link_public_headers)
 
add_executable( landmark_registration
src/main/landmark_registration_main.c
${common_sources}
src/landmark_tools/landmark_util/estimate_homography.c
src/landmark_tools/feature_selection/int_forstner_extended.c
src/landmark_tools/feature_tracking/corr_image_long.c
src/landmark_tools/feature_tracking/parameters.c
src/landmark_tools/math/homography_util.c
src/landmark_tools/image_io/imagedraw.c
src/landmark_tools/utils/two_level_yaml_parser.c
src/img/utils/int_forstner.c
 )
add_dependencies(landmark_registration link_public_headers)
 
add_executable(point_2_landmark
src/main/point_2_landmark_main.c
${common_sources}
src/landmark_tools/landmark_util/point_cloud2grid.c
submodules/librply/src/lib/rply.c
)
add_dependencies(point_2_landmark link_public_headers)

add_executable(landmark_2_point
src/main/landmark_2_point_main.c
${common_sources}
src/landmark_tools/landmark_util/point_cloud2grid.c
submodules/librply/src/lib/rply.c
)
add_dependencies(landmark_2_point link_public_headers)

add_executable(edit_landmark
    src/main/edit_landmark_main.c
    ${common_sources}
)
add_dependencies(edit_landmark link_public_headers)

add_executable(distort_landmark
    src/main/distort_landmark_main.c
    ${common_sources}
)
add_dependencies(distort_landmark link_public_headers)

add_executable(add_srm
    src/main/add_srm_main.c
    ${common_sources}
)
add_dependencies(add_srm link_public_headers)

if(GDAL_FOUND)
    message("create_landmark building with GeoTiff support")
    add_definitions(-DUSE_GEOTIFF)
    
    include_directories(${GDAL_INCLUDE_DIRS})

    target_link_libraries( create_landmark GDAL::GDAL ${yaml_LIBRARIES} ${GSL_LIBRARIES} m -lz)
    target_link_libraries( create_landmark_from_img GDAL::GDAL ${yaml_LIBRARIES} ${GSL_LIBRARIES} m -lz)
else()
    message("create_landmark building without GeoTiff support")

    target_link_libraries( create_landmark ${yaml_LIBRARIES} ${GSL_LIBRARIES} m -lz)
    target_link_libraries( create_landmark_from_img ${GSL_LIBRARIES} ${yaml_LIBRARIES} m -lz)
endif()

target_link_libraries( landmark_comparison ${yaml_LIBRARIES} ${PNG_LIBRARIES} ${GSL_LIBRARIES} m -lz)
target_link_libraries( landmark_registration ${yaml_LIBRARIES} ${PNG_LIBRARIES} ${GSL_LIBRARIES} m -lz)
target_link_libraries( point_2_landmark ${GSL_LIBRARIES} m)
target_link_libraries( landmark_2_point ${GSL_LIBRARIES} m)
target_link_libraries( distort_landmark ${GSL_LIBRARIES} m)
target_link_libraries( edit_landmark  ${PNG_LIBRARIES} ${GSL_LIBRARIES} m -lz)
target_link_libraries( add_srm ${PNG_LIBRARIES} ${GSL_LIBRARIES} m -lz)


if (WITH_OPENCV)
    add_library(landmark_tools_opencv_tools
       src/landmark_tools/opencv_tools/opencv_feature_matching.cpp
       src/landmark_tools/opencv_tools/opencv_image_io.cpp
    )
    target_compile_features(landmark_tools_opencv_tools PUBLIC cxx_std_11)
    target_include_directories(landmark_tools_opencv_tools PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(landmark_tools_opencv_tools ${OpenCV_LIBRARIES})


    add_executable(image_comparison
        src/main/image_comparison_main.c
        ${common_sources}
        src/landmark_tools/feature_tracking/feature_match.c
        src/landmark_tools/feature_tracking/corr_image_long.c
        src/landmark_tools/feature_tracking/parameters.c
        src/landmark_tools/math/homography_util.c
        src/landmark_tools/utils/two_level_yaml_parser.c
    )
    add_dependencies(image_comparison link_public_headers)
    target_link_libraries(image_comparison landmark_tools_opencv_tools  ${yaml_LIBRARIES} ${GSL_LIBRARIES})
else()
    message(STATUS "Not building image_comparison as OpenCV is OFF")
endif()

## Build landmark_tools Library

file(GLOB_RECURSE library_sources
 "src/landmark_tools/*.c"
 "src/math/*.c"
 "src/img/*.c")

## Shared library

add_library(lib_landmark_tools
SHARED
${library_sources}
submodules/librply/src/lib/rply.c
)

target_include_directories(lib_landmark_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    submodules/
    ${PNG_INCLUDE_DIRS})

target_link_libraries(lib_landmark_tools PUBLIC GDAL::GDAL ${yaml_LIBRARIES} ${PNG_LIBRARIES} ${GSL_LIBRARIES} m -lz)

install(TARGETS lib_landmark_tools
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include)
