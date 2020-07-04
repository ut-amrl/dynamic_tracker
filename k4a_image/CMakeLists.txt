cmake_minimum_required(VERSION 2.8)
project(k4a_image)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(k4a_image
    src/KinectWrapper.cpp
    src/main.cpp
)

include_directories( include 
    ${Ceres_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
	/usr/include/eigen3
    )

# Dependencies of this library
target_link_libraries(k4a_image PRIVATE
    ceres
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
    vision_geometry
    k4a
    k4arecord
    )
