#!/bin/bash

set -e

script_dir=$(cd $(dirname $0); pwd)

package_path_prefix="$script_dir/src"
build_path_prefix="$script_dir/build"
install_dir="$script_dir/install"

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$install_dir

# 检测 ninja 是否安装
if ! command -v ninja &> /dev/null; then
    echo "Error: ninja is needed to build the packages. Use the following command to install ninja:"
    echo "    sudo apt update && sudo apt install -y ninja-build"
    exit 1
fi

build_and_install_package() {
    package_path=$1
    shift
    cmake_additional_command=$@

    package_abs_path=$package_path_prefix/$package_path
    build_dir=$build_path_prefix/$package_path


    echo "Building package: $package_path"
    echo "    package_abs_path: $package_abs_path"
    echo "    install_dir: $install_dir"
    echo "    build_dir: $build_dir"
    echo "    cmake_additional_command: $cmake_additional_command"

    # 创建目录
    mkdir -p $install_dir
    mkdir -p $build_dir

    # 运行 cmake
    cmake \
        -S $package_abs_path \
        -B $build_dir \
        -DCMAKE_INSTALL_PREFIX=$install_dir \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -GNinja \
        $cmake_additional_command

    # 编译并安装
    cmake --build $build_dir --target install
}

build_and_install_package grpc
build_and_install_package grasp_model_grpc_msg

$script_dir/install_python_pkg.sh