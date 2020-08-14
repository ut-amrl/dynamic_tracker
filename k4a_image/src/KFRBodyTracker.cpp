#include "KFRBodyTracker.h"

KFRBodyTracker::KFRBodyTracker(k4a_calibration_t sensor_calibration)
{
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&sensor_calibration, tracker_config, &_tracker);
}

KFRBodyTracker::~KFRBodyTracker() {}

void KFRBodyTracker::receiveFrame(k4a_capture_t capture)
{
    k4a_capture_reference(capture);
    k4a_capture_release(capture);
}