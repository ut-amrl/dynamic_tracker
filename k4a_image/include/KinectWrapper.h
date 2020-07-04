#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>


class KinectWrapper {
public:
    KinectWrapper(uint8_t deviceIndex);
    ~KinectWrapper();

    void capture();
    void display();

protected:
    k4a_device_t _device;
};

#endif
