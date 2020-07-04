#include "KinectWrapper.h"
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <vision_geometry/CVUtil.h>
#include <vision_geometry/Util.h>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/HomographyShortcuts.h>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

k4a_playback_t playback_handle = NULL;
int main() {
    // KinectWrapper kinectWrapper(0);
    // kinectWrapper.display();
    k4a_device_get_calibration();

    Size s(4, 6);

    cv::VideoCapture cap("/home/henry/Downloads/chessboard.mkv"); 
    while(true){
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()){
            break;
        }
        Mat c;
        bool found = findChessboardCorners(frame, s, c, CALIB_CB_FAST_CHECK);

        if(found){
            cout << "found corners" << endl;
            drawChessboardCorners(frame, s, c, found);
        } else{
        }
        cv::imshow("Frame", frame);

        char in = (char) cv::waitKey(16);
        if(in == 27){
            break;
        } else if(in == ' '){
            std::stringstream name;
            name << std::time(nullptr) << ".jpg";
            cout << "capturing " << name.str() << endl;
            imwrite("../captures/" + name.str(), frame);
        }
    }
    return 0;
}
