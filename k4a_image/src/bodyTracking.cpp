#include "PlaybackWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "KFRBodyTracker.h"
#include "KFRDisplay.h"
#include "KinectWrapper.h"

int main (){
    KFRBodyTracker kfrBodyTracker(false, false);
    PlaybackWrapper playback("/home/fri/Documents/vijay/dynamic_tracker/k4a_image/main_pranav2.mkv", kfrBodyTracker);
    int n = 10;
    while(n--){
        playback.capture();
    }

    return 0;
}