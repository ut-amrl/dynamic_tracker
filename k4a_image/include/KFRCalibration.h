#ifndef KFR_DISPLAY_H
#define KFR_DISPLAY_H

#include "K4ACaptureRecipient.h"
#include <vector>
#include <opencv2/opencv.hpp>

class KFRCalibration : public K4ACaptureRecipient {
protected:
    int chessboardRows, chessboardColumns;

public:
    std::vector<cv::Point2d> corners;

    KFRCalibration(int chessboardRows, int chessboardColumns);
    ~KFRCalibration();
    void receiveFrame(k4a_capture_t capture);
};

#endif