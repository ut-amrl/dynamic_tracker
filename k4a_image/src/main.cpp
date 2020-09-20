#include "KFRDisplay.h"
#include "KinectWrapper.h"
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <vision_geometry/CVUtil.h>
#include <vision_geometry/Util.h>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/HomographyShortcuts.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>
#include <vision_geometry/HomCartShortcuts.h>

using namespace std;
using namespace cv;
using namespace Eigen;

const double SCALE_FACTOR = 4;

const CameraIntrinsics intrinsics(1834.17,
                                  1833.1, 0.,
                                  1910.01,
                                  1113.17);
k4a_playback_t playback_handle = NULL;

Mat scale(Mat in, double scaleFactor)
{
  Mat scaled;
  Size size(in.cols / scaleFactor, in.rows / scaleFactor);
  resize(in, scaled, size);
  return scaled;
}

int main()
{
  KFRDisplay kfrDisplay(2160, 3840);
  KinectWrapper kinectWrapper(0, kfrDisplay);
  kinectWrapper.capture();
  //while(kinectWrapper.capture()) {}

  /*
  Size s(6, 4);
  ModelChessboard chessboard(s.height, s.width, 23.0);

  MatrixXd chessboardPoints = chessboard.getModelCBH2D();
  cout << "Chessboard points" << endl;
  cout << chessboardPoints << endl;

  vector<string> files = listFiles(
      "../captures/");
  vector<Mat> images = loadImages(files);
  vector<Mat> detections;
  cout << "loadImages:  " << images.size() << endl;
  vector<MatrixXd> camPoints;
  detectCorners(s, images, detections, camPoints);
  cout << "Detected corners:" << endl;
  cout << camPoints[0] << endl;

  vector<MatrixXd> homographies = computeHomographies(chessboardPoints, camPoints);

  // Try going from object points to camera points
  MatrixXd homPts = homographies[0] * chessboardPoints;
  homPts = divideByLastRowRemoveLastRow(homPts);
  cout << "Estimated points from homography (chessboard points -> camera points):" << endl;
  cout << homPts << endl;

  MatrixXd optHom = optimizeHomography(chessboardPoints, camPoints[0], homographies[0]);
  MatrixXd optHomPts = optHom * chessboardPoints;
  optHomPts = divideByLastRowRemoveLastRow(optHomPts);
  cout << "Optimized points from homography (chessboard points -> camera points):" << endl;
  cout << optHomPts << endl;

  Scalar colorA(0, 255, 255, 127);
  Mat scaled = scale(images[0], SCALE_FACTOR);
  // Draw calculated pointed
  showPoints(scaled, optHomPts / SCALE_FACTOR, colorA);

  // Image rectification
  MatrixXd homInv = homographies[0].inverse();
  // Apply a translation after applying homography
  MatrixXd t(3, 3);
  t << 1, 0, 1000,
      0, 1, 1000,
      0, 0, 1;
  homInv = t * homInv;
  Mat rectified;
  Mat H = convertToCVMat(homInv);
  warpPerspective(images[0], rectified, H, images[0].size());
  rectified = scale(rectified, SCALE_FACTOR);
  imshow("Rectified", rectified);

  waitKey(0);
  */

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
