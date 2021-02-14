#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "K4ACaptureRecipient.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>

class KinectWrapper
{
public:
    KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient &kfr);
    KinectWrapper(uint8_t deviceIndex, k4a_wired_sync_mode_t syncMode, K4ACaptureRecipient &kfr);
    ~KinectWrapper();

    bool capture();
    k4a_calibration_intrinsics_t getColorCameraIntrinsics();
    k4a_calibration_t getCalibration();
    //void display();

    static size_t getNumCameras();

protected:
    k4a_device_t _device;
    k4a_calibration_t _calibration;
    k4a_device_configuration_t _config;
    K4ACaptureRecipient &_kfr;
};

#endif
