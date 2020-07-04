#include "KinectWrapper.h"
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <vision_geometry/CVUtil.h>
#include <vision_geometry/Util.h>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/HomographyShortcuts.h>
#include <Eigen/Dense>
#include "ceres/ceres.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const CameraIntrinsics intrinsics(1834.17,
                                          1833.1, 0.,
                                          1910.01,
                                          1113.17);
k4a_playback_t playback_handle = NULL;
int main() {
    // KinectWrapper kinectWrapper(0);
    // kinectWrapper.display();

    Size s(4, 6);
    ModelChessboard chessboard(s.height, s.width, 23.0);

    vector<string> files = listFiles(
      "/home/henry/dynamic_tracker/k4a_image/captures/");
    vector<Mat> images = loadImages(files);
    vector<Mat> detections;
    cout << "loadImages:  " << images.size() << endl;
    vector<MatrixXd> camPoints;
    detectCorners(s, images, detections, camPoints);
    cout << camPoints[0] << endl;

    vector<MatrixXd> homographies = computeHomographies(chessboard.getModelCBH2D(), camPoints);

    // cv::Mat frame = imread("/home/henry/dynamic_tracker/k4a_image/captures/1593831230.jpg");
    // Mat c;
    // bool found = findChessboardCorners(frame, s, c, CALIB_CB_FAST_CHECK);
    // if(found){
    //         cout << "found corners" << endl;
    //         drawChessboardCorners(frame, s, c, found);
    // } else{
    // }

    // cv::imshow("Frame", frame);
    // waitKey(0);


    // cv::VideoCapture cap("/home/henry/Downloads/chessboard.mkv"); 
    // while(true){
    //     cv::Mat frame;
    //     cap >> frame;
    //     if(frame.empty()){
    //         break;
    //     }
    //     Mat c;
    //     // bool found = findChessboardCorners(frame, s, c, CALIB_CB_FAST_CHECK);

    //     // if(found){
    //     //     cout << "found corners" << endl;
    //     //     drawChessboardCorners(frame, s, c, found);
    //     // } else{
    //     // }
    //     cv::imshow("Frame", frame);

    //     char in = (char) cv::waitKey(16);
    //     if(in == 27){
    //         break;
    //     } else if(in == ' '){
    //         std::stringstream name;
    //         name << std::time(nullptr) << ".jpg";
    //         cout << "capturing " << name.str() << endl;
    //         imwrite("../captures/" + name.str(), frame);
    //     }
    // }
    return 0;
}
