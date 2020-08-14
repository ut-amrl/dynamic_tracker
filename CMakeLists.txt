PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Dynamic Tracker")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

FIND_PACKAGE(octomap REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# FIND_PACKAGE(PCL REQUIRED QUIET COMPONENTS common filters)
# INCLUDE_DIRECTORIES(${pcl_INCLUDE_DIRS})



INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags lua5.1 # libpcl # amrl-shared-lib
    pthread)

ADD_SUBDIRECTORY(k4a_image/share/vision_geometry)
INCLUDE_DIRECTORIES(k4a_image/share/vision_geometry)
INCLUDE_DIRECTORIES(k4a_image)
INCLUDE_DIRECTORIES(k4a_image/include)
INCLUDE_DIRECTORIES(/home/knapsack/software/octomap/octomap/include/)
INCLUDE_DIRECTORIES(/home/knapsack/workspaces/unitree_ws/src/grid_map/grid_map_core/include)
INCLUDE_DIRECTORIES(/home/knapsack/workspaces/unitree_ws/src/grid_map/grid_map_octomap/include)
INCLUDE_DIRECTORIES(/home/knapsack/workspaces/unitree_ws/src/grid_map/grid_map_msgs/include)
INCLUDE_DIRECTORIES(/home/knapsack/workspaces/unitree_ws/src/grid_map/grid_map_ros/include)
# INCLUDE_DIRECTORIES(/usr/include/pcl-1.8/pcl)
# INCLUDE_DIRECTORIES(/usr/include/pcl-1.8/pcl/filters/)
# INCLUDE_DIRECTORIES(/usr/include/pcl-1.8/pcl/common/)


MESSAGE(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

LINK_LIBRARIES(${OCTOMAP_LIBRARIES})

FIND_LIBRARY(VISION_GEOMETRY_LIBRARY vision_geometry HINTS ${CMAKE_SOURCE_DIR}/share/vision_geometry/build/)

ADD_LIBRARY(k4a_utils SHARED k4a_image/src/utils/k4a_utils.cpp)
install(TARGETS k4a_utils
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


ROSBUILD_ADD_EXECUTABLE(test_octomap
                        k4a_image/test/test_octomap.cpp
                        k4a_image/src/KinectWrapper.cpp
						k4a_image/src/KFRDisplay.cpp)
TARGET_LINK_LIBRARIES(test_octomap 
	${libs} 
	${OpenCV_LIBS} 
    k4a
    k4arecord
    ${OCTOMAP_LIBRARIES}
)

ROSBUILD_ADD_EXECUTABLE(kinect_to_gridmap
                        k4a_image/src/kinect_to_gridmap.cpp
                        k4a_image/src/KinectWrapper.cpp
						k4a_image/src/KFRDisplay.cpp
						k4a_image/src/utils/k4a_utils.cpp
						)
TARGET_LINK_LIBRARIES(kinect_to_gridmap 
	${libs} 
	${OpenCV_LIBS} 
    k4a
    k4arecord
    k4a_utils
    ${OCTOMAP_LIBRARIES}
)


# ROSBUILD_ADD_EXECUTABLE(ekf_tracker
#                         src/ekf_tracker/ekf_tracker_main.cc
#                         src/ekf_tracker/ekf_tracker.cc)
# TARGET_LINK_LIBRARIES(ekf_tracker ${libs})

# ADD_EXECUTABLE(tracker_unit_tests
#                src/tests/ekf_tracker_tests.cc)
# TARGET_LINK_LIBRARIES(tracker_unit_tests gtest gtest_main ${libs})
