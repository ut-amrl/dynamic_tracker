#ifndef KFR_DISPLAY_H
#define KFR_DISPLAY_H

#include "KinectFrameRecipient.h"

class KFRDisplay : public KinectFrameRecipient {
public:
    void receiveFrame(cv::Mat &colorMat);
};

#endif