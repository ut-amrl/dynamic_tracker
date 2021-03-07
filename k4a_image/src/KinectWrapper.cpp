#include "KinectWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

KinectWrapper::KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient &kfr) :
    KinectWrapper(deviceIndex, K4A_WIRED_SYNC_MODE_STANDALONE, kfr)
{
}

KinectWrapper::KinectWrapper(uint8_t deviceIndex, k4a_wired_sync_mode_t syncMode, K4ACaptureRecipient &kfr) :
    _device(NULL), _config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL), _kfr(kfr)
{
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &_device))
    {
        cout << "aborting" << endl;
        abort();
    }
    
    _config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    _config.wired_sync_mode = syncMode;
    _config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    _config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _config.synchronized_images_only = true;

    // Recipient specific config
    _kfr.getDevice(_device, _config);

    // try to start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &_config))
    {
        cout << "Device " << (int) deviceIndex << " failed to start cameras" << endl;
        k4a_device_close(_device);
        abort();
    }
}

KinectWrapper::~KinectWrapper()
{
    k4a_device_stop_cameras(_device);
    k4a_device_close(_device);
}

k4a_calibration_t KinectWrapper::getCalibration() {
    return _calibration;
}

k4a_calibration_intrinsics_t KinectWrapper::getColorCameraIntrinsics() {
    return _calibration.color_camera_calibration.intrinsics;
}

bool KinectWrapper::capture()
{
    Mat res;
    k4a_capture_t capture = NULL;
    switch (k4a_device_get_capture(_device, &capture, K4A_WAIT_INFINITE))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
    {
        _kfr.receiveFrame(capture);
        k4a_capture_release	(capture);
        return true;
    }
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        break;
    }
    return false;
}

size_t KinectWrapper::getNumCameras() {
    return k4a_device_get_installed_count();
}

void KinectWrapper::seeConnectedCameras(){
    // Directly from Azure Kinect SDK Tutorial
    
    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d connected devices:\n", device_count);

    k4a_device_t device = NULL;

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
        {
            printf("%d: Failed to open device\n", deviceIndex);
            continue;
        }

        char *serial_number = NULL;
        size_t serial_number_length = 0;

        if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
        {
            printf("%d: Failed to get serial number length\n", deviceIndex);
            k4a_device_close(device);
            device = NULL;
            continue;
        }

        serial_number = (char *)malloc(serial_number_length);
        if (serial_number == NULL)
        {
            printf("%d: Failed to allocate memory for serial number (%zu bytes)\n", deviceIndex, serial_number_length);
            k4a_device_close(device);
            device = NULL;
            continue;
        }

        if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
        {
            printf("%d: Failed to get serial number\n", deviceIndex);
            free(serial_number);
            serial_number = NULL;
            k4a_device_close(device);
            device = NULL;
            continue;
        }

        printf("%d: Device \"%s\"\n", deviceIndex, serial_number);

        k4a_device_close(device);
    }
}