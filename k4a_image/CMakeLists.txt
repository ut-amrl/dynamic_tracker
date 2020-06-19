cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)
set( NAME_SRC src/main.cpp )
add_executable(k4a_image ${NAME_SRC})
# Dependencies of this library
target_link_libraries(k4a_image PRIVATE
    ${OpenCV_LIBS} 
    k4a
    k4arecord
    )
