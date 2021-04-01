#include "PlaybackWrapper.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <opencv2/opencv.hpp>
#include "KFRBodyTracker.h"
#include "KFRDisplay.h"
#include "KinectWrapper.h"

int main (int argc, char *argv[]){
    std::string saveFile = "";
    std::string mkvFile = "";
    if(argc == 2){
        saveFile = argv[1];
    }
    if(argc == 3){
        mkvFile = argv[2];
    }else{
        saveFile = "../pranav2.bt";
        mkvFile = "../main_pranav2.mkv";
    }

    KFRBodyTracker kfrBodyTracker(&saveFile[0],false, true);
    PlaybackWrapper playback(&mkvFile[0], kfrBodyTracker);
    int n = 200;
    while(n--){
        playback.capture();
    }

    return 0;
}