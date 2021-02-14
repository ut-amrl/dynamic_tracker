#ifndef MULTI_RECORDER_H
#define MULTI_RECORDER_H

#include "KinectWrapper.h"
#include <vector>

struct Capture {
    uint64_t deviceTime;
    uint64_t sysTime;
};

class KFRRecord;

class MultiRecorder {
public:
    MultiRecorder();
    ~MultiRecorder();
    int registerRecorder(KFRRecord*);
    void receiveFrame(int device, k4a_capture_t capture);

private:
    int _devices;
    // devices x N array representing capture history
    std::vector<std::vector<Capture>> _captureHistory;

};

#endif