cmake_minimum_required(VERSION 3.16)

# Download and configure Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add test executable
add_executable(landmark_tests
  unit_tests/landmark_test.cpp
)

# Include directories
target_include_directories(landmark_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src
  ${GSL_INCLUDE_DIRS}
  ${PNG_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
)

# Link test executable against gtest and landmark_tools
target_link_libraries(landmark_tests
  PRIVATE
    gtest_main
    landmark_tools
    ${GSL_LIBRARIES}
    ${PNG_LIBRARIES}
    ${yaml_LIBRARIES}
    m
)

# Add test to CTest
include(GoogleTest)
gtest_discover_tests(landmark_tests) 