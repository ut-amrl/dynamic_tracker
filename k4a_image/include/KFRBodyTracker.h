#ifndef KFR_BODY_TRACKER_H
#define KFR_BODY_TRACKER_H

#include "K4ACaptureRecipient.h"

#include <k4abt.h>
#include <opencv2/opencv.hpp>

class KFRBodyTracker : public K4ACaptureRecipient {
public:
    KFRBodyTracker(const char* path, bool realTime = true, bool writeToFile = false);
    ~KFRBodyTracker();
    void receiveFrame(k4a_capture_t capture);
    void getCalibration(k4a_calibration_t calib) override;

protected:
    k4abt_tracker_t _tracker;
    bool _realTime;
    bool _writeToFile;
    const char* _path;

private:
    void writeArrayToFile(std::vector<float> positions[], std::fstream &outFile);
};

#endif