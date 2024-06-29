#!/bin/bash

# 清理目录
clean_dir=false
# 源ROS环境
source_ros=false
# 构建 camera_calibrator 包
build_package=false
# 运行相机标定程序
run_calibrator=false

# 解析命令行参数
while getopts "csbr" opt; do
    case $opt in
        c)
            clean_dir=true
            ;;
        s)
            source_ros=true
            ;;
        b)
            build_package=true
            ;;
        r)
            run_calibrator=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

# 切换到ROS工作空间目录
cd /root/kylin/camera_calibration

# 清理之前的构建
if [ "$clean_dir" = true ]; then
    rm -rf build/ devel/
    echo "Cleaned build and devel directories."
fi

# 构建 camera_calibrator 包
if [ "$build_package" = true ]; then
    catkin_make --source camera_calibrator/ --build build/camera_calibrator/
    if [ $? -eq 0 ]; then
        echo "Camera calibrator package built successfully."
    else
        echo "Failed to build camera calibrator package."
        exit 1
    fi
fi

# 源ROS环境
if [ "$source_ros" = true ]; then
    source devel/setup.bash
    echo "ROS environment sourced."
fi

# 运行相机标定launch文件
if [ "$run_calibrator" = true ]; then
    roslaunch camera_calibrator camera_calibrator.launch
fi
