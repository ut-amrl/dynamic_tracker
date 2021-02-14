#ifndef KFR_RECORD_H
#define KFR_RECORD_H

#include "K4ACaptureRecipient.h"
#include <k4arecord/record.h>
#include "MultiRecorder.h"

class KFRRecord : public K4ACaptureRecipient {

public:
    KFRRecord(const char* path);
    KFRRecord(const char* path, MultiRecorder* multiRecorder);
    ~KFRRecord();
    void receiveFrame(k4a_capture_t capture);
    void getDevice(k4a_device_t device, k4a_device_configuration_t config) override;
    void addTag(const char* name, const char* value);
    void close();
protected:
    // Path for record destination
    const char* _path;
    k4a_record_t _recordingHandle;
    bool _headerWritten;
    bool _closed;
    // Send frame information to global
    MultiRecorder* _multiRecorder;
    int _multiRecorderDeviceId;
};

#endif