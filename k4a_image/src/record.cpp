#include "KFRRecord.h"
#include "KinectWrapper.h"
#include <iostream>
#include <pthread.h>

using namespace std;

int main()
{
    MultiRecorder mr;
    KFRRecord kfr("/home/fri/Documents/henry/dynamic_tracker/k4a_image/subordinate.mkv", &mr);
    KinectWrapper sub(1, K4A_WIRED_SYNC_MODE_SUBORDINATE, kfr);

    KFRRecord kfrRecord("/home/fri/Documents/henry/dynamic_tracker/k4a_image/main.mkv", &mr);
    KinectWrapper kinectWrapper(3, K4A_WIRED_SYNC_MODE_MASTER, kfrRecord);

    // Capture some frames
    int n = 150;
    while (n--)
    {
        kinectWrapper.capture();
        sub.capture();
        cout << n << endl;
    }

    kfrRecord.close();
    kfr.close();
      // TODO add multiple camera test - psuedocode
    // MultiRecorder mr;
    // for all devices:
    //    KFRRecord kfr(path)
    //    mr.register(kfr)
    //    KinectWrapper kw(device, kfr)
    //    in parallel kw.capture() N times

    // int numDevices = 2;
    // pthread_t tid[numDevices];
    // void *status;

    // int err;
    // long i;
    // for (i = 0; i < numDevices; i++)
    // {
    //     err = pthread_create(&tid[i], NULL, <do work> , (void *)<args>);
    // }

    // for (i = 0; i < numDevices; i++)
    //     err = pthread_join(tid[i], &status);
    return 0;
}

int playback()
{
    k4a_playback_t playback_handle = NULL;
    if (k4a_playback_open("/home/fri/Documents/henry/dynamic_tracker/k4a_image/test.mkv", &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording\n");
        return 1;
    }
    return 0;
}
