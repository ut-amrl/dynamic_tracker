#ifndef KFR_DISPLAY_H
#define KFR_DISPLAY_H

#include "K4ACaptureRecipient.h"

#include <opencv2/opencv.hpp>

class KFRDisplay : public K4ACaptureRecipient {
protected:
    int _rows, _cols;
    //cv::Mat _colorMat;

public:
    KFRDisplay(int rows, int cols);
    ~KFRDisplay();
    void receiveFrame(k4a_capture_t capture);
};

#endif