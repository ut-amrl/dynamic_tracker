#include "KinectWrapper.h"


k4a_playback_t playback_handle = NULL;
int main() {
    KinectWrapper kinectWrapper(0);
    kinectWrapper.capture();
    return 0;
}
