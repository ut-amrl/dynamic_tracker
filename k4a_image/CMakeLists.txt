cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)
add_executable(k4a_image
    src/KinectWrapper.cpp
    src/main.cpp
)
include_directories( include )
# Dependencies of this library
target_link_libraries(k4a_image PRIVATE
    ${OpenCV_LIBS} 
    k4a
    k4arecord
    )
