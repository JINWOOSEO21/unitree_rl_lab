cmake_minimum_required(VERSION 3.12)
project(go2_controller)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(X11 REQUIRED)

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
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
add_executable(go2_scandots_probe tools/scandots_probe.cpp)

enable_testing()
add_executable(go2_keyboard_tests tests/test_keyboard_control.cpp)
target_include_directories(go2_keyboard_tests PRIVATE include)
target_compile_options(go2_keyboard_tests PRIVATE -UNDEBUG)
add_test(NAME go2_keyboard_tests COMMAND go2_keyboard_tests)
add_executable(go2_startup_handoff_tests tests/test_startup_handoff.cpp)
target_include_directories(go2_startup_handoff_tests PRIVATE include)
target_compile_options(go2_startup_handoff_tests PRIVATE -UNDEBUG)
add_test(NAME go2_startup_handoff_tests COMMAND go2_startup_handoff_tests)
add_test(
  NAME go2_keyboard_rejects_non_tty_before_dds
  COMMAND bash -c "printf x | '$<TARGET_FILE:go2_ctrl>' --keyboard-check >/dev/null 2>&1; test $? -eq 2"
)
find_program(GO2_PYTHON3 python3 REQUIRED)
add_test(NAME go2_keyboard_pty
  COMMAND ${GO2_PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_keyboard_pty.py $<TARGET_FILE:go2_ctrl>)

# Fake Xlib backend tests release/focus handling without desktop key injection.
add_executable(go2_held_heading_tests tests/test_held_heading_input.cpp src/HeldHeadingInput.cpp)
target_compile_options(go2_held_heading_tests PRIVATE -UNDEBUG)
add_test(NAME go2_held_heading_tests COMMAND go2_held_heading_tests)
