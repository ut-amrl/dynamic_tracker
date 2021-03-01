#ifndef MULTI_RECORDER_H
#define MULTI_RECORDER_H

#include "KinectWrapper.h"
#include <vector>

struct Capture {
    int deviceId;
    uint64_t deviceFrame;   // Frame number on this device
    uint64_t deviceTime;    // Time of capture according to camera
    uint64_t sysTime;       // Time received over bus
};

class KFRRecord;

class MultiRecorder {
public:
    MultiRecorder();
    ~MultiRecorder();
    int registerRecorder(KFRRecord*);
    void receiveFrame(int device, k4a_capture_t capture);
    void printCaptureHistory();

private:
    int _devices;
    // devices x N array representing capture history
    std::vector<std::vector<Capture>> _captureHistory;

};

#endif