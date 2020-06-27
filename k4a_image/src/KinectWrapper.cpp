#include "KinectWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>

using namespace std;

KinectWrapper::KinectWrapper(uint8_t deviceIndex) : _device(NULL) {
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &_device)) {
        abort();
    }

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    // try to start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &config)) {
        k4a_device_close(_device);
        abort();
    }
}

KinectWrapper::~KinectWrapper() {
    k4a_device_close(_device);
}

void KinectWrapper::capture() {
    // k4a_capture_t capture = NULL; 
    // switch (k4a_device_get_capture(_device, &capture, K4A_WAIT_INFINITE)) {
    //     case K4A_WAIT_RESULT_SUCCEEDED: {
    //         k4a_image_t image = k4a_capture_get_color_image(capture);
    //         printf(" | Depth16 res:%4dx%4d stride:%5d\n",
    //             k4a_image_get_height_pixels(image),
    //             k4a_image_get_width_pixels(image),
    //             k4a_image_get_stride_bytes(image));
    //         k4a_image_release(image);
    //         k4a_capture_release(capture);
    //         break;
    //     }
    // }
    k4a_capture_t capture = NULL; 
    switch (k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE)) {
    case K4A_WAIT_RESULT_SUCCEEDED: {
        k4a_image_t image = k4a_capture_get_color_image(capture);
        int rows = k4a_image_get_height_pixels(image);
        int cols = k4a_image_get_width_pixels(image);
        // image buffer
        uint8_t* buffer = k4a_image_get_buffer(image);
        // opencv matrix
        cv::Mat colorMat(rows, cols, CV_8UC4, (void*) buffer, cv::Mat::AUTO_STEP);
        cv::imshow("Image", colorMat);
        printf("res:%4dx%4d\n", rows, cols); 
        k4a_image_release(image);
        k4a_capture_release(capture);
        
        break;
    }
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        k4a_device_close(device);
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        k4a_device_close(device);
        break;
    }
}


    // // open the recording
    // if (k4a_playback_open("test_output.mkv", &playback_handle) != K4A_RESULT_SUCCEEDED)
    // {
    //     printf("Failed to open recording!\n ");
    //     return 1;
    // }
    // uint64_t recording_length = k4a_playback_get_last_timestamp_usec(playback_handle);

    // read captures from mkv file
    // k4a_capture_t capture = NULL;
    // k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;

    // // count frames
    // int count = 0;
    // while (result == K4A_STREAM_RESULT_SUCCEEDED) {
    //     result = k4a_playback_get_next_capture(playback_handle, &capture);
    //     if (result == K4A_STREAM_RESULT_SUCCEEDED){
    //         count++;
    //         // get image from capture and print image info
    //         k4a_image_t image = k4a_capture_get_color_image(capture);
    //         printf("%d", K4A_IMAGE_FORMAT_COLOR_MJPG == k4a_image_get_format(image));
    //         printf(" | Depth16 res:%4dx%4d stride:%5d\n",
    //             k4a_image_get_height_pixels(image),
    //             k4a_image_get_width_pixels(image),
    //             k4a_image_get_stride_bytes(image));
    //         k4a_image_release(image);
    //         k4a_capture_release(capture);
    //     }

    //     // reached eof
    //     else if (result == K4A_STREAM_RESULT_EOF) {
    //         printf("Read entire recording!\n");
    //         break;
    //     }
    // }

    // // failed to stream
    // if (result == K4A_STREAM_RESULT_FAILED) {
   	// k4a_playback_close(playback_handle);
    //     printf("Failed to read entire recording\n");
    //     return 1;
    // }

    // // print some recording info
    // printf("Recording is %d seconds long\n", recording_length / 1000000);
    // printf("Total Frames Read: %d\n", count);
    // printf("Average frames per second: %d\n",  count / (recording_length / 1000000));

    // // exit
    // k4a_playback_close(playback_handle);