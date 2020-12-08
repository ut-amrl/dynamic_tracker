#include "KFRBodyTracker.h"

KFRBodyTracker::KFRBodyTracker(k4a_calibration_t sensor_calibration, bool realTime)
    : _realTime(realTime)
{
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&sensor_calibration, tracker_config, &_tracker);
}

KFRBodyTracker::~KFRBodyTracker()
{
    k4abt_tracker_shutdown(_tracker);
    k4abt_tracker_destroy(_tracker);
}

void KFRBodyTracker::receiveFrame(k4a_capture_t capture)
{
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
    for (int i = 0; i < k4abt_frame_get_num_bodies(body_frame); i++) {
        unsigned int id = k4abt_frame_get_body_id(body_frame, i);
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
        
        
    }

    k4abt_frame_release(body_frame);
}