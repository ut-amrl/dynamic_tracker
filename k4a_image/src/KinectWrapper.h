#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include <k4a/k4a.h>

class KinectWrapper {
public:
    KinectWrapper(uint8_t deviceIndex);
    ~KinectWrapper();

    void capture();

protected:
    k4a_device_t _device;
};

#endif