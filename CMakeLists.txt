cmake_minimum_required(VERSION 3.10)

project(camera_calibrator)

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
  ${OpenCV_LIB_DIRS}
  ${argp_LIB_DIRS}
)

add_executable(camera_calibrator
  camera_calibrator.cpp
)

target_link_libraries(camera_calibrator
  ${OpenCV_LIBS}
  -lspcdev
)