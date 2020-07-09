#include "KinectWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

KinectWrapper::KinectWrapper(uint8_t deviceIndex) : _device(NULL)
{
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &_device))
    {
        cout << "aborting" << endl;
        abort();
    }
    k4a_calibration_t calibration;
    k4a_device_get_calibration(_device, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_2160P, &calibration);
    // cout << "found " << calibration.color_camera_calibration.intrinsics.parameter_count << " intrinsic params" << endl;
    // cout << "cx: " << calibration.color_camera_calibration.intrinsics.parameters.param.cx << endl;
    // cout << "cy: " << calibration.color_camera_calibration.intrinsics.parameters.param.cy << endl;
    // cout << "fx: " << calibration.color_camera_calibration.intrinsics.parameters.param.fx << endl;
    // cout << "fy: " << calibration.color_camera_calibration.intrinsics.parameters.param.fy << endl;

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    // try to start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &config))
    {
        k4a_device_close(_device);
        abort();
    }
}

KinectWrapper::~KinectWrapper()
{
    cout << "Destructing... do not use device handle again" << endl;
    k4a_device_close(_device);
}

Mat KinectWrapper::capture(bool display)
{
    Mat res;
    k4a_capture_t capture = NULL;
    switch (k4a_device_get_capture(_device, &capture, K4A_WAIT_INFINITE))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
    {
        k4a_image_t image = k4a_capture_get_color_image(capture);
        int rows = k4a_image_get_height_pixels(image);
        int cols = k4a_image_get_width_pixels(image);
        // image buffer
        uint8_t *buffer = k4a_image_get_buffer(image);
        // opencv matrix
        cv::Mat colorMat = cv::Mat(rows, cols, CV_8UC4, (void *)buffer, cv::Mat::AUTO_STEP).clone();
        if (display)
        {
            cv::imshow("Image", colorMat);
            int wait = cv::waitKey(0);
            printf("res:%4dx%4d\n", rows, cols);
        }
        k4a_image_release(image);
        k4a_capture_release(capture);
        res = colorMat;
        break;
    }
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        k4a_device_close(_device);
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        k4a_device_close(_device);
        break;
    }
    return res;
}

void KinectWrapper::display()
{
    k4a_capture_t capture = NULL;
    k4a_wait_result_t result = K4A_WAIT_RESULT_SUCCEEDED;
    bool exit = false;
    while (result == K4A_WAIT_RESULT_SUCCEEDED && !exit)
    {
        result = k4a_device_get_capture(_device, &capture, K4A_WAIT_INFINITE);
        switch (result)
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
        {
            k4a_image_t image = k4a_capture_get_color_image(capture);
            int rows = k4a_image_get_height_pixels(image);
            int cols = k4a_image_get_width_pixels(image);
            // opencv matrix
            cv::Mat colorMat(rows, cols, CV_8UC4, (void *)k4a_image_get_buffer(image), cv::Mat::AUTO_STEP);
            cv::imshow("Image", colorMat);
            char c = (char)cv::waitKey(25);

            // Press ESC to cancel, space to capture
            if (c == 27)
            {
                exit = true;
            }
            else if (c == ' ')
            {
                std::stringstream name;
                name << std::time(nullptr) << ".jpg";
                cout << "capturing " << name.str() << endl;
                imwrite("../captures/" + name.str(), colorMat);
            }
            // printf("res:%4dx%4d\n", rows, cols);
            k4a_image_release(image);
            k4a_capture_release(capture);

            break;
        }
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            k4a_device_close(_device);
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            k4a_device_close(_device);
            break;
        }
    }
}