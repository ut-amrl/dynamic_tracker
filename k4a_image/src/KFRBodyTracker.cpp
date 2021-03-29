#include "KFRBodyTracker.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <vector>



KFRBodyTracker::KFRBodyTracker(const char* path, bool realTime, bool writeToFile)
    : _realTime(realTime), _writeToFile(writeToFile), _path(path)
{
}

KFRBodyTracker::~KFRBodyTracker()
{
    k4abt_tracker_shutdown(_tracker);
    k4abt_tracker_destroy(_tracker);
}

void KFRBodyTracker::getDevice(k4a_device_t device, k4a_device_configuration_t config){
    k4a_calibration_t sensor_calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
    {
        printf("Get depth camera calibration failed!\n");
        return;
    }
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&sensor_calibration, tracker_config, &_tracker);
}

void KFRBodyTracker::getCalibration(k4a_calibration_t calib){
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&calib, tracker_config, &_tracker);
}

void KFRBodyTracker::receiveFrame(k4a_capture_t capture)
{
    std::fstream outFile;
    if(this->_writeToFile){
        outFile.open(this->_path, std::fstream::binary | std::fstream::app);
    }

    k4a_capture_reference(capture);
    // Process real time or synchronously
    int wait = _realTime ? 0 : K4A_WAIT_INFINITE;
    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(_tracker, capture, wait);
    k4a_capture_release(capture);
    if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED) {
        // It should never hit timeout or error when K4A_WAIT_INFINITE is set.
        printf("Error! Adding capture to tracker process queue failed!\n");
        return;
    }
    k4abt_frame_t body_frame = NULL;
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(_tracker, &body_frame, wait);
    if (pop_frame_result != K4A_WAIT_RESULT_SUCCEEDED) {
        // It should never hit timeout or error when K4A_WAIT_INFINITE is set.
        printf("Error! Popping body tracking result failed!\n");
        return;
    }

    // do stuff
    if(this->_writeToFile){
        uint64_t timestamp = k4abt_frame_get_device_timestamp_usec(body_frame);
        outFile << timestamp;
        std::cout << "timestamp: " + std::to_string(timestamp) << std::endl;
        uint32_t numBodies = k4abt_frame_get_num_bodies(body_frame);
        outFile << numBodies;
        std::cout << "num bodies: " + std::to_string(numBodies) << std::endl;
        //loop through each body
        for(int i = 0; i < numBodies; i++){
            //array of vectors for storing positions and orientations of each joint
            std::vector<float> positions[(int)K4ABT_JOINT_COUNT];
            std::vector<float> orientations[(int)K4ABT_JOINT_COUNT];
            //get skeleton of 1 body if any
            k4abt_skeleton_t skeleton;
            k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
            uint32_t body_id = k4abt_frame_get_body_id(body_frame, i);
            outFile << body_id;
            for(int j = 0; j < (int)K4ABT_JOINT_COUNT; j++){
                positions[j].push_back(skeleton.joints[j].position.xyz.x);
                positions[j].push_back(skeleton.joints[j].position.xyz.y);
                positions[j].push_back(skeleton.joints[j].position.xyz.z);

                orientations[j].push_back(skeleton.joints[j].orientation.wxyz.w);
                orientations[j].push_back(skeleton.joints[j].orientation.wxyz.x);
                orientations[j].push_back(skeleton.joints[j].orientation.wxyz.y);
                orientations[j].push_back(skeleton.joints[j].orientation.wxyz.z);
            }
        }


    }
    printf("Num bodies in frame: %d\n", k4abt_frame_get_num_bodies(body_frame));
    /*
    for (int i = 0; i < k4abt_frame_get_num_bodies(body_frame); i++) {
        unsigned int id = k4abt_frame_get_body_id(body_frame, i);
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
        
        
    }*/

    k4abt_frame_release(body_frame);
}