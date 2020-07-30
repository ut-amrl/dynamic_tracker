#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "KinectFrameRecipient.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>

class KinectWrapper
{
public:
    KinectWrapper(uint8_t deviceIndex, KinectFrameRecipient &kfr);
    ~KinectWrapper();

    cv::Mat capture();
    void display();

protected:
    k4a_device_t _device;
    KinectFrameRecipient &_kfr;
};

#endif
