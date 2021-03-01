#include "PlaybackWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

PlaybackWrapper::PlaybackWrapper(const char* path, K4ACaptureRecipient &kfr): _kfr(kfr)
{
    if (k4a_playback_open(path, &_playback) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording\n");
    }
}


PlaybackWrapper::~PlaybackWrapper()
{
    k4a_playback_close(_playback);
}


bool PlaybackWrapper::capture()
{
    k4a_capture_t capture = NULL;
    switch (k4a_playback_get_next_capture(_playback, &capture))
    {
    case K4A_STREAM_RESULT_SUCCEEDED:
    {
        _kfr.receiveFrame(capture);
        k4a_capture_release	(capture);
        return true;
    }
    case K4A_STREAM_RESULT_EOF:
        printf("Reached end of file\n");
        break;
    case K4A_STREAM_RESULT_FAILED:
        printf("Failed to read entire recording\n");
        break;
    }
    return false;
}
