#include "KFRCalibration.h"
#include "conversions.h"
#include <vector>

using namespace cv;

KFRCalibration::KFRCalibration(int chessboardRows, int chessboardColumns):
    chessboardRows(chessboardRows), chessboardColumns(chessboardColumns) {}
KFRCalibration::~KFRCalibration() {}

void KFRCalibration::receiveFrame(k4a_capture_t capture) {
    k4a_capture_reference(capture);

    k4a_image_t image = k4a_capture_get_color_image(capture);
    Mat cvImage = kinect_to_cv(image);
    k4a_image_release(image);

    Mat grayscaleImage;
    cvtColor(cvImage, grayscaleImage, CV_BGRA2GRAY);

    Size patternSize(chessboardRows, chessboardColumns);

    corners.clear();

    bool found = findChessboardCorners(grayscaleImage, patternSize, corners);
    // TODO maybe look at cornerSubPix to get more accuracy

    //drawChessboardCorners(cvImage, patternSize, corners, found);
    //Mat scaledImage;
    //resize(cvImage, scaledImage, Size(), 0.4, 0.4);
    //imshow("Chessboard", scaledImage);
    //waitKey();

    k4a_capture_release(capture);
}

