#include "KinectWrapper.h"
#include <k4arecord/playback.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>


k4a_playback_t playback_handle = NULL;
int main() {
    KinectWrapper kinectWrapper(0);
    kinectWrapper.capture();
    return 0;
}
