#include "KinectWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

KinectWrapper::KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient &kfr) :
    _device(NULL), _config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL), _kfr(kfr)
{
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &_device))
    {
        cout << "aborting" << endl;
        abort();
    }
    k4a_device_get_calibration(_device, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_2160P, &_calibration);
    // cout << "found " << calibration.color_camera_calibration.intrinsics.parameter_count << " intrinsic params" << endl;
    // cout << "cx: " << calibration.color_camera_calibration.intrinsics.parameters.param.cx << endl;
    // cout << "cy: " << calibration.color_camera_calibration.intrinsics.parameters.param.cy << endl;
    // cout << "fx: " << calibration.color_camera_calibration.intrinsics.parameters.param.fx << endl;
    // cout << "fy: " << calibration.color_camera_calibration.intrinsics.parameters.param.fy << endl;

    _config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    //_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    _config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    _config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _config.synchronized_images_only = true;

    // Recipient specific config
    _kfr.getDevice(_device, _config);

    // try to start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &_config))
    {
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
