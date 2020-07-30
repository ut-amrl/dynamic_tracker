#ifndef KINECT_FRAME_RECIPIENT_H
#define KINECT_FRAME_RECIPIENT_H

#include <opencv2/opencv.hpp>

class KinectFrameRecipient {
public:
    virtual void receiveFrame(cv::Mat &colorMat) = 0;
};

#endif