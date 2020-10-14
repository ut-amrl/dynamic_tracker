cmake_minimum_required(VERSION 2.8)
project(glfw_example)

SET(main_sources
    src/main.cpp
)

add_executable(glfw_example
    ${main_sources}
)

include_directories(
	include
)

target_link_libraries(glfw_example
	glfw
	GL
	m
)
