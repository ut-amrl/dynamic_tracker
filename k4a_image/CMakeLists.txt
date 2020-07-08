cmake_minimum_required(VERSION 2.8)
project(k4a_image)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(VISION_GEOMETRY_LIBRARY vision_geometry HINTS ${CMAKE_SOURCE_DIR}/share/vision_geometry/build/)

message(STATUS ${CMAKE_SOURCE_DIR})

add_executable(k4a_image
    src/KinectWrapper.cpp
    src/main.cpp
)

add_executable(server
    src/server.cpp
)

add_executable(client
    src/client.cpp
)

include_directories(
	include
	share/vision_geometry/include/
    ${Ceres_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
	/usr/include/eigen3
)

# Dependencies of this library
target_link_libraries(k4a_image
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ceres
    k4a
    k4arecord
)
