cmake_minimum_required(VERSION 3.12)
project(parkour_tests)

# 배포 바이너리와 같은 헤더를 쓰되, 별도 프로젝트로 둔다 — 검사용 실행파일이
# go2_ctrl 빌드에 섞이지 않게 하기 위해서다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

set(DEPLOY_ROOT ${PROJECT_SOURCE_DIR}/../..)

include_directories(
  /usr/include/eigen3
  ${PROJECT_SOURCE_DIR}/../include          # parkour/*.h
  ${DEPLOY_ROOT}/include                    # isaaclab/algorithms/algorithms.h
  ${DEPLOY_ROOT}/thirdparty/onnxruntime-linux-x64-1.22.0/include
)

add_executable(test_obs_golden test_obs_golden.cpp)
target_link_libraries(test_obs_golden
  libyaml-cpp.a
  ${DEPLOY_ROOT}/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so
)
