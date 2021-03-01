#ifndef PLAYBACK_WRAPPER_H
#define PLAYBACK_WRAPPER_H

#include "K4ACaptureRecipient.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>

class PlaybackWrapper
{
public:
    PlaybackWrapper(const char* path, K4ACaptureRecipient &kfr);
    ~PlaybackWrapper();

    bool capture();

protected:
    k4a_playback_t _playback;
    K4ACaptureRecipient &_kfr;
};

#endif
