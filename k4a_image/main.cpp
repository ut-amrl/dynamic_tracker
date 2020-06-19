#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <stdio.h>
#include <stdlib.h>

k4a_playback_t playback_handle = NULL;
int main() {
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
        else {
            printf("Successfully opened device!");
        }

        k4a_device_close(device);
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
    return 0;
}
