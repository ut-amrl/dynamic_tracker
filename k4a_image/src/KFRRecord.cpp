#include "KFRRecord.h"
#include <iostream>

using namespace std;

KFRRecord::KFRRecord(const char* path) {

    // TODO probably want this as argument but currently KinectWrapper does not expose/give access
    // Config must match that of device
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // TODO don't think this format works for recordings?
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    // Create new recording handle with path
    // Device handle set to NULL
    if (K4A_RESULT_SUCCEEDED != k4a_record_create(path, NULL, config, &_recordingHandle))
    {
        cout << "Failed to initialize recording handle" << endl;
    }
}

KFRRecord::~KFRRecord() {
    // Note: will flush data to disk before closing
    k4a_record_close(_recordingHandle);
}

void KFRRecord::receiveFrame(k4a_capture_t capture) {
    if (K4A_RESULT_SUCCEEDED != k4a_record_write_capture(_recordingHandle, capture))
    {
        cout << "Failed to initialize recording handle" << endl;
    }
}