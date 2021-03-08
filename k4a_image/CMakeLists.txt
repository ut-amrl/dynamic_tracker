cmake_minimum_required(VERSION 2.8)
project(k4a_image)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(VISION_GEOMETRY_LIBRARY vision_geometry HINTS ${CMAKE_SOURCE_DIR}/share/vision_geometry/build/)
find_library(APRIL_TAG_LIBRARY apriltag HINTS ${CMAKE_SOURCE_DIR}/share/apriltag/build/)
#find_library(JSON_LIBRARY nlohmann_json_cmake_fetchcontent HINTS ${CMAKE_SOURCE_DIR}/share/nlohmann_json_cmake_fetchcontent/build)
#find_library(JSON_LIBRARY nlohman_json HINTS ${CMAKE_SOURCE_DIR}/share/json/build)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(share/nlohmann_json)

message(STATUS ${CMAKE_SOURCE_DIR})

SET(KINECT_WRAPPER_SOURCES
    src/KinectWrapper.cpp
    src/KFRDisplay.cpp
    src/KFRBodyTracker.cpp
    src/conversions.cpp
)

SET(CALIBRATOR_SOURCES
    src/calibration.cpp
    src/KFRCalibration.cpp
    src/CalibrationManager.cpp
)

add_executable(k4a_image
    src/main.cpp
    src/KFRRecord.cpp
    src/MultiRecorder.cpp
    src/CalibrationManager.cpp
    ${KINECT_WRAPPER_SOURCES}
)

add_executable(viewer
    src/viewer.cpp
    ${CALIBRATOR_SOURCES}
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

add_executable(capture-corners
    src/capture-corners.cpp
    ${CALIBRATOR_SOURCES}
    ${KINECT_WRAPPER_SOURCES}
)

add_executable(record
    src/record.cpp
    src/KFRRecord.cpp
    src/MultiRecorder.cpp
    ${KINECT_WRAPPER_SOURCES}
)

add_executable(apriltag_demo
    src/apriltag_demo.cpp
)

add_executable(body-tracker
    src/bodyTracking.cpp
    src/KFRRecord.cpp
    src/PlaybackWrapper.cpp
    src/KFRBodyTracker.cpp
    src/MultiRecorder.cpp
    ${KINECT_WRAPPER_SOURCES}
)

add_library(sockets SHARED src/sockets.cpp)

include_directories(
	include
	share/vision_geometry/include/
    share/apriltag/
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

target_link_libraries(capture-corners
    ${Ceres_LIBS}
    ${OpenCV_LIBS} 
	${VISION_GEOMETRY_LIBRARY}
    ${Eigen3_LIBS}
    ceres
    k4a
    k4arecord
    k4abt
)

target_link_libraries(record
    ${OpenCV_LIBS} 
    k4a
    k4abt
    k4arecord
    -pthread
)

target_link_libraries(body-tracker
    ${OpenCV_LIBS}
    k4a
    k4abt
    k4arecord
    nlohmann_json::nlohmann_json
)

target_link_libraries(apriltag_demo
    ${APRIL_TAG_LIBRARY} 
    ${OpenCV_LIBS} 
)
