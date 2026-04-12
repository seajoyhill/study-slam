cmake_minimum_required(VERSION 3.22)
project(study_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# gflags、glog、Pangolin 均为 git submodule；勿 add_subdirectory。构建目录放在 .deps/build/*，安装前缀 .deps（与主工程 build/ 分离）。
#
#   git submodule update --init thirdparty/gflags thirdparty/glog thirdparty/Pangolin
#   (cd thirdparty/gflags && git checkout v2.2.2)
#   (cd thirdparty/glog && git checkout v0.7.1)
#   (cd thirdparty/Pangolin && git checkout v0.9.5)   # 可选；与当前子模块记录提交一致时可省略
#
#   cmake -S thirdparty/gflags -B .deps/build/gflags -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
#     -DCMAKE_INSTALL_PREFIX="$PWD/.deps" -DBUILD_SHARED_LIBS=OFF
#   cmake --build .deps/build/gflags && cmake --install .deps/build/gflags
#
#   cmake -S thirdparty/glog -B .deps/build/glog \
#     -DCMAKE_PREFIX_PATH="$PWD/.deps" -DCMAKE_INSTALL_PREFIX="$PWD/.deps" \
#     -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DWITH_GTEST=OFF
#   cmake --build .deps/build/glog && cmake --install .deps/build/glog
#
# Pangolin 置于 thirdparty/Pangolin（版本自选）。当前上游常用「构建树」导出 PangolinConfig.cmake，不必 install。
# 依赖 Eigen3、OpenGL 与各平台窗口库（macOS 上 Xcode/Command Line Tools 即可）。
#
#   cmake -S thirdparty/Pangolin -B .deps/build/pangolin -DCMAKE_BUILD_TYPE=Release \
#     -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF
#   cmake --build .deps/build/pangolin
#
# 若主工程用 Debug 而 Pangolin 仅配置了 Release，可在配置主工程时加：
#   -DCMAKE_MAP_IMPORTED_CONFIG_DEBUG=Release
#
# 仅当 gflags、glog 都已安装到 .deps 时才优先 .deps；否则易出现「conda 的 glog + .deps 的 gflags」混链、重复注册 gflags。
set(_study_deps "${CMAKE_SOURCE_DIR}/.deps")
if(EXISTS "${_study_deps}/lib/cmake/gflags/gflags-config.cmake" AND EXISTS "${_study_deps}/lib/cmake/glog/glog-config.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${_study_deps}")
endif()
unset(_study_deps)

# 只 find glog；其 glog-config 会 find_dependency(gflags)。勿再单独 find_package(gflags)，否则易重复链接两份 gflags。
find_package(glog 0.7.1 CONFIG REQUIRED)

set(_study_pangolin_build "${CMAKE_SOURCE_DIR}/.deps/build/pangolin")
if(EXISTS "${_study_pangolin_build}/PangolinConfig.cmake")
  set(Pangolin_DIR "${_study_pangolin_build}")
endif()
unset(_study_pangolin_build)
find_package(Pangolin 0.7 CONFIG REQUIRED)

add_subdirectory(test)
