#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "vision_geometry/CameraIntrinsics.h"

#include "KinectWrapper.h"
#include "KFRCalibration.h"
#include "vision_geometry/CVUtil.h"
#include "vision_geometry/HomographyShortcuts.h"
#include "vision_geometry/HomCartShortcuts.h"

CameraIntrinsics k(1834.17,
    1833.1, 0.,
    1910.01,
    1113.17);

MatrixXd createEmptyRT() {
    // Format: quaternion x, y, z, w; translation x, y, z
    VectorXd v(7);
    v(0) = 0;
    v(1) = 0;
    v(2) = 0;
    v(3) = 1;
    v(4) = 0;
    v(5) = 0;
    v(6) = 0;
    return v;
}

int main()
{
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    //kc.test();

    // Internal sizes - that is, the number of points where the squares touch
    int chessboardRows = 6;
    int chessboardCols = 4;
    double chessboardSpacing = 1; // TODO double-check
    kc.chessboard = ModelChessboard(chessboardRows, chessboardCols, chessboardSpacing);

    MatrixXd chessboard3dPoints = kc.chessboard.getModelCBC3D();
    std::vector<cv::Point3d> objectPoints;
    for (int i = 0; i < chessboard3dPoints.cols(); i++) {
        objectPoints.push_back(cv::Point3d(chessboard3dPoints(i, 0), chessboard3dPoints(i, 1), chessboard3dPoints(i, 2)));
    }

    KFRCalibration handler(chessboardRows, chessboardCols);
    std::vector<Camera> cameras;
    std::vector<MatrixXd> camPoints;
    std::vector<cv::Mat> images;
    for (int device = 0; device < 2; device++) {
        KinectWrapper wrapper(device, handler);

        bool success = false;
        for (int i = 0; i < 3; i++) {
            if (wrapper.capture()) {
                success = true;
                break;
            }
            std::cerr << "Failed to read a capture, will retry..." << std::endl;
        }
        if (!success) {
            std::cerr << "Too many failures, aborting" << std::endl;
            return 1;
        }

        Camera current;
        current.projections[0] = MatrixXd(2, handler.corners.size());
        for(int i = 0; i < handler.corners.size(); i++) {
            current.projections[0](0, i) = handler.corners[i].x;
            current.projections[0](1, i) = handler.corners[i].y;
        }
        camPoints.push_back(current.projections[0]);
        current.index = device;
        cameras.push_back(current);

        images.push_back(handler.image);

        //std::cout << handler.corners << std::endl;
    }

    MatrixXd chessboardPoints = kc.chessboard.getModelCBH2D();

    std::vector<MatrixXd> homographies = computeHomographies(chessboardPoints, camPoints);

    // Try going from object points to camera points
    //MatrixXd homPts = homographies[0] * kc.chessboard.getModelCBH2D();
    //homPts = divideByLastRowRemoveLastRow(homPts);
    //cout << "Estimated points from homography (chessboard points -> camera points):" << endl;
    //cout << homPts << endl;

    std::vector<MatrixXd> optHom = optimizeHomographies(chessboardPoints, camPoints, homographies);
    // Camera relative to chessboard
    std::vector<RigidTrans> transforms = extractRTs(k, optHom);
    for (RigidTrans trans : transforms) {
        std::cout << trans.getTransMat() << std::endl;
    }

    std::cout << transforms[0].getTransMat().inverse() * transforms[1].getTransMat() << std::endl;

    // chessboardPoints: 3 * 24; homographies: 3 * 3
    // std::cout << chessboardPoints.rows() << " " << chessboardPoints.cols() << " " << homographies[0].rows() << " " << homographies[0].cols();
    for (int cam = 0; cam < homographies.size(); cam++) {
        MatrixXd homography = optHom[cam];
        MatrixXd projectedPoints = homography * chessboardPoints;
        MatrixXd originalPoints = camPoints[cam];
        std::cout << "Camera " << cam << ":\n";
        for (int point = 0; point < projectedPoints.cols(); point++) {
            std::cout << projectedPoints(0, point) / projectedPoints(2, point) << ", " << projectedPoints(1, point) / projectedPoints(2, point)
                    << "\t" << originalPoints(0, point) << ", " << originalPoints(1, point) << "\n";
        }
        std::cout << std::endl;

        for (int point = 0; point < projectedPoints.cols(); point++) {
            drawCrosshair(images[cam], projectedPoints(0, point) / projectedPoints(2, point), projectedPoints(1, point) / projectedPoints(2, point), 10, 2, cv::Scalar(255, 0, 0));
        }

        cv::Mat scaled;
        cv::resize(images[cam], scaled, cv::Size(), 0.4, 0.4);
        cv::imshow("camera", scaled);
        cv::waitKey();
    }

    /*
    // Cameras relative to origin
    std::vector<MatrixXd> cameraRTs;
    for (int i = 0; i < cameras.size(); i++) {
        cameraRTs.push_back(createEmptyRT());
    }
    std::vector<MatrixXd> objectRTs;
    objectRTs.push_back(createEmptyRT());
    objectRTs[0](6) = 1;

    kc.globalOpt(cameras, cameraRTs, objectRTs);

    for (int i = 0; i < cameraRTs.size(); i++) {
        std::cout << "Camera " << i << ": " << cameraRTs[i] << std::endl;
    }
    */
}