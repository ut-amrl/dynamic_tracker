#include "MultiRecorder.h"
#include <ostream>

using namespace std;

MultiRecorder::MultiRecorder() : _devices(0) {
}

MultiRecorder::~MultiRecorder(){
    cout << "devices: " << _devices << endl;
    // for(int i = 0; i < _captureHistory[0].size(); i++){
    //     cout << _captureHistory[0][i].deviceTime << " " << _captureHistory[0][i].sysTime << endl;
    // }
    cout << "receiver 0 1st frame: " << _captureHistory[0][0].deviceTime << " " << _captureHistory[0][0].sysTime << endl;
    cout << "receiver 1 1st frame: " << _captureHistory[1][0].deviceTime << " " << _captureHistory[1][0].sysTime << endl;
}

// Register a new KFRRecord source
int MultiRecorder::registerRecorder(KFRRecord*){
    // TODO manage KFRRecord/KinectWrapper/Device ID/Device serial coordination better
    vector<Capture> deviceCaptures;
    _captureHistory.push_back(deviceCaptures);
    return _devices++;
}

void MultiRecorder::receiveFrame(int device, k4a_capture_t capture){
    k4a_capture_reference(capture);
    // Must get image to find timestamps
    k4a_image_t image = k4a_capture_get_color_image(capture);
    uint64_t deviceTime = k4a_image_get_device_timestamp_usec(image);
    uint64_t sysTime = k4a_image_get_system_timestamp_nsec(image);

    // Add to timing data for this frame to device history (should be thread-safe)
    _captureHistory[device].push_back({device, _captureHistory[device].size(), deviceTime, sysTime});

    k4a_image_release(image);
    k4a_capture_release(capture);
}

void MultiRecorder::printCaptureHistory(){
    ofstream outfile;  
    outfile.open("capture_history");

    vector<Capture> fullHistory;
    for(vector<Capture>& v : _captureHistory){
        // Append together
        fullHistory.insert(fullHistory.end(), v.begin(), v.end());
    }
    // Sort the overall history by the time on the local device - offsets should be 0
    sort(fullHistory.begin(), fullHistory.end(), 
        [](const Capture& a, const Capture& b) -> bool
        {
            return a.deviceTime < b.deviceTime;
        });
    
    for(int i = 0; i < fullHistory.size(); i++){
        Capture c = fullHistory[i];
        // Format (space separated integers)
        // overall frame number, device id, device frame number, device timestamp, overall timestamp
        outfile << i << " " << c.deviceId << " " << c.deviceFrame << " " << c.deviceTime << " " << c.sysTime << endl;
    }

    outfile.close();
}