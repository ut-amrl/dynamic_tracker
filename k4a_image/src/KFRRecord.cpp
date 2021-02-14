#include "KFRRecord.h"
#include <iostream>

using namespace std;

KFRRecord::KFRRecord(const char* path) : _path(path), _headerWritten(false), _multiRecorder(NULL) {
    // Most of init requires device information
}

KFRRecord::KFRRecord(const char* path, MultiRecorder* multiRecorder) : _path(path), _headerWritten(false), _multiRecorder(multiRecorder) {
    // Most of init requires device information
    _multiRecorderDeviceId = multiRecorder->registerRecorder(this);
    cout << "registered receiver : " << _multiRecorderDeviceId << endl;
}

KFRRecord::~KFRRecord() {
    close();
}

void KFRRecord::receiveFrame(k4a_capture_t capture) {
    k4a_capture_reference(capture);
    if(!_headerWritten){
        k4a_record_write_header(_recordingHandle);
        _headerWritten = true;
    }
    if(_multiRecorder){
        _multiRecorder->receiveFrame(_multiRecorderDeviceId, capture);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_record_write_capture(_recordingHandle, capture))
    {
        cout << "Failed to initialize recording handle" << endl;
    }
    k4a_capture_release(capture);
}

void KFRRecord::getDevice(k4a_device_t device, k4a_device_configuration_t config){
    // Create new recording handle with path
    // Device handle set to NULL
    if (K4A_RESULT_SUCCEEDED != k4a_record_create(_path, device, config, &_recordingHandle))
    {
        cout << "Failed to initialize recording handle" << endl;
    }
}

void KFRRecord::addTag(const char* name, const char* value){
    k4a_record_add_tag(_recordingHandle, name, value);
}

void KFRRecord::close(){
    if(!_closed){
        // Note: will flush data to disk before closing
        k4a_record_close(_recordingHandle);
        _closed = true;
    }
}