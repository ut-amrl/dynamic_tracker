#ifndef KFR_BODY_TRACKER_H
#define KFR_BODY_TRACKER_H

#include "K4ACaptureRecipient.h"

#include <k4abt.h>
#include <opencv2/opencv.hpp>

class KFRBodyTracker : public K4ACaptureRecipient {
public:
    KFRBodyTracker(k4a_calibration_t sensor_calibration);
    ~KFRBodyTracker();
    void receiveFrame(k4a_capture_t capture);
protected:
    k4abt_tracker_t _tracker;
};

#endif