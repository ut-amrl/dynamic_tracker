add_executable(k4a_image main.cpp)

# Dependencies of this library
target_link_libraries(k4a_image PRIVATE 
    k4a
    k4arecord
    )