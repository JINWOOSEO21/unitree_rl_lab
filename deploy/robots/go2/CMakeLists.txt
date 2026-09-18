cmake_minimum_required(VERSION 3.13)
project(go2_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The SDK checkout already contains the architecture-specific libraries.
# Use its exported targets directly; no separate SDK installation is needed.
get_filename_component(DEFAULT_SDK_ROOT "${PROJECT_SOURCE_DIR}/../../../../unitree_sdk2" ABSOLUTE)
set(UNITREE_SDK_ROOT "${DEFAULT_SDK_ROOT}" CACHE PATH "Unitree SDK2 source checkout")
if(NOT EXISTS "${UNITREE_SDK_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "Unitree SDK2 checkout not found: ${UNITREE_SDK_ROOT}. Set UNITREE_SDK_ROOT.")
endif()
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Unitree SDK examples" FORCE)
add_subdirectory("${UNITREE_SDK_ROOT}" "unitree_sdk2" EXCLUDE_FROM_ALL)

# DT_RPATH keeps the SDK's DDS libraries ahead of ROS LD_LIBRARY_PATH entries.
set(CMAKE_BUILD_RPATH "${UNITREE_SDK_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
add_link_options("LINKER:--disable-new-dtags")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(X11 REQUIRED)

include_directories(
  /usr/include/eigen3
  include
  ${X11_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../parkour/include   # parkour/*.h (관측/액션/조향/scandots)
)

link_libraries(
  ${X11_LIBRARIES}
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)

file(GLOB_RECURSE ADD_SRC_LIST
  src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(go2_ctrl main.cpp)

# Optional recording utility for live sensor and controller telemetry.
add_executable(go2_walk_record tools/walk_record.cpp)
