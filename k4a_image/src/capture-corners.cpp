#include <iostream>
#include <fstream>
#include "KinectCalibrator.h"
#include "KinectWrapper.h"
#include "calibration.h"

#include "KFRAprilTags.h"
#include "CalibrationManager.h"

using namespace Eigen;

int chessboardRows = 4;
int chessboardCols = 6;
// should be 30 mm
double chessboardSpacing = 30;

// NOTE: file format is different here
/*int main()
{
    MatrixXd chessboardPoints = ModelChessboard(chessboardRows, chessboardCols, chessboardSpacing).getModelCBH2D();

    std::vector<MatrixXd> camPoints;
    std::vector<MatrixXd> cameraIntrinsicMatrices;

    int numCams = KinectWrapper::getNumCameras();
    for (int device = 0; device < numCams; device++) {
        MatrixXd points, intrinsics;

        bool success = captureChessboardCorners(device, chessboardRows, chessboardCols, &intrinsics, &points);
        if (!success) {
            std::cout << "Failed to capture points. Run this program again" << std::endl;
            return 1;
        }

        camPoints.push_back(points);
        cameraIntrinsicMatrices.push_back(intrinsics);
    }

    std::vector<MatrixXd> chessboardToCameraRTs;
    for(size_t i = 0; i < camPoints.size(); i++) {
        chessboardToCameraRTs.push_back(computeRTModelToCamera(chessboardPoints, camPoints[i], cameraIntrinsicMatrices[i]));
    }

    std::ofstream output;
    output.open("calibration.txt"); // TODO file name

    output << chessboardRows << " " << chessboardCols << " " << chessboardSpacing << " " << numCams << "\n";

    for (int cam = 0; cam < numCams; cam++) {
        writeMatrix(output, cameraIntrinsicMatrices[cam]);
        writeMatrix(output, camPoints[cam]);
    }

    output.flush();
    output.close();

    std::cout << "Finished capturing points" << std::endl;
}*/

int main() {
    int numCams = KinectWrapper::getNumCameras();
    std::vector<Measurement> measurements;
    int baseCamera = -1;
    for (int i = 0; i < numCams; i++) {
        KFRAprilTags recipient(i);
        KinectWrapper wrapper(i, recipient);
        wrapper.capture();
        
        if (baseCamera == -1 && recipient.measurements.size() > 0) {
            baseCamera = i;
        }
        measurements.insert(measurements.end(), recipient.measurements.begin(), recipient.measurements.end());
    }

    if (baseCamera == -1) {
        std::cerr << "ERROR: No tags found" << std::endl;
        return 1;
    }

    Anchor base(CAMERA, baseCamera);

    Calibration calib(measurements);
    std::ofstream out("calibration.txt");
    calib.write(out);
    return 0;
}