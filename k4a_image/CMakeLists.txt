cmake_minimum_required(VERSION 2.8)
project(k4a_image)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(VISION_GEOMETRY_LIBRARY vision_geometry HINTS ${CMAKE_SOURCE_DIR}/share/vision_geometry/build/)

message(STATUS ${CMAKE_SOURCE_DIR})

SET(KINECT_WRAPPER_SOURCES
    src/KinectWrapper.cpp
    src/KFRDisplay.cpp
    src/KFRBodyTracker.cpp
    src/conversions.cpp
)

add_executable(k4a_image
    src/main.cpp
    ${KINECT_WRAPPER_SOURCES}
)

add_executable(viewer
    src/viewer.cpp
    ${KINECT_WRAPPER_SOURCES}
)

add_executable(server
    src/server.cpp
)

add_executable(client
    src/client.cpp
    src/KinectWrapper.cpp
    src/KFRDisplay.cpp
    src/conversions.cpp
)

add_executable(calib
    src/calib.cpp
    src/conversions.cpp
    src/KinectCalibrator.cpp
    src/KinectWrapper.cpp
    src/KFRCalibration.cpp
)

add_library(sockets SHARED src/sockets.cpp)

include_directories(
	include
	share/vision_geometry/include/
    ${Ceres_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
	/usr/include/eigen3
)

target_link_libraries(sockets
    ${OpenCV_LIBS} 
	#${VISION_GEOMETRY_LIBRARY}
    k4a
    k4arecord
)

# Dependencies of this library
target_link_libraries(k4a_image
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ceres
    k4a
    k4abt
    k4arecord
)

target_link_libraries(viewer
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ceres
    k4a
    k4abt
    k4arecord
    GL
    GLU
    glfw
    m
)

# Dependencies of this library
target_link_libraries(client
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ceres
    k4a
    k4arecord
    sockets
)

# Dependencies of this library
target_link_libraries(server
    sockets
)

target_link_libraries(calib
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ${Eigen3_LIBS}
    ceres
    k4a
    k4arecord
)
