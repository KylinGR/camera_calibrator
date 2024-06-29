cmake_minimum_required(VERSION 3.2)
project(camera_calibration)

message(STATUS "Searching required modules")
find_package(PkgConfig REQUIRED)

set(CXX_STD "14" CACHE STRING "C++ standard")

# OPENCV
find_package(OpenCV REQUIRED)

# Catkin
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    camera_info_manager
)
catkin_package(
    INCLUDE_DIRS include
)

# Build scenario
option(SUPPRESS_BIND_GLOBAL_PLACEHOLDERS
    "Suppresses Boost >= 1.68 deprecation warning message for global placeholders in boost::bind instantiation"
    ON
)
# Workaround for deprecation message from instantiation of Boost::bind
# Edit for CMake < 3.12 and ROS_DISTRO < melodic by knorth55
if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
    if(SUPPRESS_BIND_GLOBAL_PLACEHOLDERS)
        add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
    endif()
endif()

include_directories(include)

set(camera_calibration_SHARED_HEADERS
    include/image_subscriber.h
    include/camera_calibrator.h
)
add_library(camera_calibration_lib
    ${camera_calibration_SHARED_HEADERS}
    src/image_subscriber.cpp
    src/camera_calibration.cpp
)
target_include_directories(camera_calibration_lib PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(camera_calibration_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

## Declare a cpp executable
# add_executable(image_subscriber_node
#     src/image_subscriber_node.cpp
# )

# target_include_directories(image_subscriber_node PRIVATE
#     ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(image_subscriber_node
#     image_subscriber
#     ${catkin_LIBRARIES}
#     spcdev
# )

add_executable(camera_calibration_node
    src/camera_calibration_node.cpp
)

target_include_directories(camera_calibration_node PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(camera_calibration_node
    camera_calibration_lib
    ${catkin_LIBRARIES}
)


# Installation scenario
# Executables and libraries
install(TARGETS ${PROJECT_NAME}_node camera_calibration_lib
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# Launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
# Include files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
