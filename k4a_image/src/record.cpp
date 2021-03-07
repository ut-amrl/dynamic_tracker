#include "KFRRecord.h"
#include "KinectWrapper.h"
#include <iostream>
#include <pthread.h>

using namespace std;

struct CaptureArgs{
    int threadId;
};

const int framesToCapture = 200;
vector<KFRRecord*> recorders;
vector<KinectWrapper*> cameras;

void* threadWork(void* args){
    // CaptureArgs* ca = (CaptureArgs*) args;
    int threadId = (long) args;
    int n = framesToCapture;
    while (n--)
    {
        cameras[threadId]->capture();
    }
}

void parallelCapture(){
    const int numDevices = 2;
    // Specify the usb device numbers' initialization order, last device will be master
    int usbIndex[numDevices] = {3, 2};
    pthread_t tid[numDevices];
    void *status;

    int err;
    for (int i = 0; i < numDevices; i++){
        recorders.push_back(new KFRRecord("some path"));
        // Only last device can be the master
        k4a_wired_sync_mode_t syncMode = i == numDevices - 1 ? K4A_WIRED_SYNC_MODE_SUBORDINATE : K4A_WIRED_SYNC_MODE_MASTER;
        cameras.push_back(new KinectWrapper(usbIndex[i], syncMode, *recorders[i]));
        err = pthread_create(&tid[i], NULL, threadWork, (void *)(long)i);
    }

    for (int i = 0; i < numDevices; i++){
        err = pthread_join(tid[i], &status);
    }
}

int main()
{
    // KinectWrapper::seeConnectedCameras();

    // 3 2 - desk
    MultiRecorder mr;
    KFRRecord kfr("/home/fri/Documents/henry/dynamic_tracker/k4a_image/subordinate.mkv", &mr);
    KinectWrapper sub(3, K4A_WIRED_SYNC_MODE_SUBORDINATE, kfr);
    cout << "---" << endl;
    KFRRecord kfrRecord("/home/fri/Documents/henry/dynamic_tracker/k4a_image/main.mkv", &mr);
    KinectWrapper kinectWrapper(2, K4A_WIRED_SYNC_MODE_MASTER, kfrRecord);

    // Capture some frames
    int n = 150;
    while (n--)
    {
        kinectWrapper.capture();
        sub.capture();
        cout << n << endl;
    }
    // parallelCapture();

    kfrRecord.close();
    kfr.close();

    mr.printCaptureHistory();
    // TODO add multiple camera test - psuedocode
    // MultiRecorder mr;
    // for all devices:
    //    KFRRecord kfr(path)
    //    mr.register(kfr)
    //    KinectWrapper kw(device, kfr)
    //    in parallel kw.capture() N times
        
    return 0;
}