#include <iostream>
#include <fstream>
#include "KinectCalibrator.h"
#include "KinectWrapper.h"
#include "calibration.h"

using namespace Eigen;

int chessboardRows = 4;
int chessboardCols = 6;
// should be 30 mm
double chessboardSpacing = 30;

void writeMatrix(std::ostream &out, MatrixXd matrix) {
    for (int row = 0; row < matrix.rows(); row++) {
        for (int col = 0; col < matrix.cols(); col++) {
            if (col > 0) {
                out << " ";
            }
            out << matrix(row, col);
        }
        out << "\n";
    }
}

int main()
{
    MatrixXd chessboardPoints = ModelChessboard(chessboardRows, chessboardCols, chessboardSpacing).getModelCBH2D();

    std::vector<MatrixXd> camPoints;
    std::vector<MatrixXd> cameraIntrinsicMatrices;

    int numCams = KinectWrapper::getNumCameras();
    for (int device = 0; device < 2; device++) {
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
}