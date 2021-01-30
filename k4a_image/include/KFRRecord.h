#ifndef KFR_RECORD_H
#define KFR_RECORD_H

#include "K4ACaptureRecipient.h"
#include <k4arecord/record.h>

class KFRRecord : public K4ACaptureRecipient {

public:
    KFRRecord(const char* path);
    ~KFRRecord();
    void receiveFrame(k4a_capture_t capture);

protected:
    k4a_record_t _recordingHandle;
};

#endif