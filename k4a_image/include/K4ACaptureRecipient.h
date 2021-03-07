#ifndef K4A_CAPTURE_RECIPIENT_H
#define K4A_CAPTURE_RECIPIENT_H

#include <k4a/k4a.h>

class K4ACaptureRecipient {
public:
    virtual void receiveFrame(k4a_capture_t capture) = 0;
    virtual void getDevice(k4a_device_t device, k4a_device_configuration_t config) {}
    virtual void getCalibration(k4a_calibration_t calib) {}
};

#endif