cmake_minimum_required(VERSION 3.16)
project(omni_drive_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ODC_BUILD_TESTS "Build the C++ unit tests" ON)
option(ODC_BUILD_PYTHON "Build the pybind11 module" OFF)
option(ODC_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)

add_library(odc
  cpp/src/mecanum_kinematics.cpp
  cpp/src/wheel_controller.cpp
  cpp/src/pose_ekf.cpp
  cpp/src/slip_detector.cpp
  cpp/src/trajectory_tracker.cpp
  cpp/src/safety_monitor.cpp
  cpp/src/drive_core.cpp
)
target_include_directories(odc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_options(odc PRIVATE /W4)
  if(ODC_WARNINGS_AS_ERRORS)
    target_compile_options(odc PRIVATE /WX)
  endif()
else()
  target_compile_options(odc PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wold-style-cast -Wnon-virtual-dtor -Wdouble-promotion)
  if(ODC_WARNINGS_AS_ERRORS)
    target_compile_options(odc PRIVATE -Werror)
  endif()
endif()

# The control path must not throw and must not allocate. Exceptions are left
# enabled for the host build so gtest works, but the target build turns them
# off; keeping the code exception-free is enforced by review, not by flags.

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(ODC_BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(odc_tests cpp/tests/test_core.cpp)
  target_link_libraries(odc_tests PRIVATE odc GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(odc_tests)
endif()

# -----------------------------------------------------------------------------
# Python bindings
# -----------------------------------------------------------------------------
if(ODC_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(omni_drive_core bindings/bindings.cpp)
  target_link_libraries(omni_drive_core PRIVATE odc)
endif()

install(TARGETS odc EXPORT odcTargets ARCHIVE DESTINATION lib)
install(DIRECTORY cpp/include/odc DESTINATION include)
