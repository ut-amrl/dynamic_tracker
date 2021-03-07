#ifndef KFR_BODY_TRACKER_H
#define KFR_BODY_TRACKER_H

#include "K4ACaptureRecipient.h"

#include <k4abt.h>
#include <opencv2/opencv.hpp>

class KFRBodyTracker : public K4ACaptureRecipient {
public:
    KFRBodyTracker(bool realTime = true);
    ~KFRBodyTracker();
    void receiveFrame(k4a_capture_t capture);
    void getDevice(k4a_device_t device, k4a_device_configuration_t config) override;

protected:
    k4abt_tracker_t _tracker;
    bool _realTime;
};

#endif