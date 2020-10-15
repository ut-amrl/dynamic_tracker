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

// projectedPoints represents the 2D points to draw. It has two or three rows; the X is row 0 / row 2, the Y is row 1 / row 2.
// Each column is one point.
void drawPoints(cv::Mat background, Eigen::MatrixXd projectedPoints) {
    cv::Mat image;
    background.copyTo(image);

    for (int point = 0; point < projectedPoints.cols(); point++) {
        double x = projectedPoints(0, point);
        double y = projectedPoints(1, point);
        if (projectedPoints.rows() > 2) {
            x /= projectedPoints(2, point);
            y /= projectedPoints(2, point);
        }
        drawCrosshair(image, x, y, 10, 2, cv::Scalar(255, 0, 0));
    }

    cv::Mat scaled;
    cv::resize(image, scaled, cv::Size(), 0.4, 0.4);
    cv::imshow("camera", scaled);
    cv::waitKey();
}



std::vector<Eigen::MatrixXd> computeHomographiesFromHomogeneous(Eigen::MatrixXd modelPoints,
                                     std::vector<Eigen::MatrixXd> camPoints) {
  std::vector<Eigen::MatrixXd> homographies;
  for (int i = 0; i < camPoints.size(); i++) {
    Eigen::MatrixXd h =
        computeDLTHomography(modelPoints, camPoints[i]);
    homographies.push_back(h);
  }
  return homographies;
}

int main()
{
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    //kc.test();

    // Internal sizes - that is, the number of points where the squares touch
    int chessboardRows = 4;
    int chessboardCols = 6;
    double chessboardSpacing = 1; // TODO figure out what this is - one unit should be one millimeter
    kc.chessboard = ModelChessboard(chessboardRows, chessboardCols, chessboardSpacing);

    MatrixXd chessboard3dPoints = kc.chessboard.getModelCBC3D();
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < chessboard3dPoints.cols(); i++) {
        objectPoints.push_back(cv::Point3f(chessboard3dPoints(0, i), chessboard3dPoints(1, i), chessboard3dPoints(2, i)));
    }

    KFRCalibration handler(chessboardRows, chessboardCols);
    std::vector<Camera> cameras;
    std::vector<MatrixXd> camPoints;
    std::vector<cv::Mat> images;
    std::vector<MatrixXd> cameraIntrinsicMatrices;
    std::vector<std::vector<float>> cameraDistCoefficients;
    std::vector<std::vector<cv::Point2f>> cvPoints;
    for (int device = 0; device < 2; device++) {
        KinectWrapper wrapper(device, handler);
        k4a_calibration_intrinsic_parameters_t calibration = wrapper.getCalibration().color_camera_calibration.intrinsics.parameters;

        MatrixXd cameraMatrix = MatrixXd::Identity(3, 3);
        cameraMatrix(0, 0) = calibration.param.fx;
        cameraMatrix(1, 1) = calibration.param.fy;
        cameraMatrix(0, 2) = calibration.param.cx;
        cameraMatrix(1, 2) = calibration.param.cy;
        cameraIntrinsicMatrices.push_back(cameraMatrix);

        cameraDistCoefficients.push_back(std::vector<float>({calibration.param.k1, calibration.param.k2, calibration.param.k3,
            calibration.param.k4, calibration.param.k5, calibration.param.k6}));

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

        cvPoints.push_back(handler.corners);

        Camera current;
        current.projections[0] = MatrixXd(3, handler.corners.size());
        for(int i = 0; i < handler.corners.size(); i++) {
            current.projections[0](0, i) = handler.corners[i].x;
            current.projections[0](1, i) = handler.corners[i].y;
            current.projections[0](2, i) = 1.0;
        }
        camPoints.push_back(current.projections[0]);
        current.index = device;
        cameras.push_back(current);

        images.push_back(handler.image);

        //std::cout << handler.corners << std::endl;
    }

    float centroidX = 0;
    float centroidY = 0;
    for(int i = 0; i < camPoints[0].cols(); i++) {
        centroidX += camPoints[0] (0, i);
        centroidY += camPoints[0] (1, i);
    }

    centroidX /= camPoints[0].cols();
    centroidY /= camPoints[0].cols();

    float scale = 0;
    for(int i = 0; i < camPoints[0].cols(); i++) {
        float scaleX = camPoints[0] (0, i) - centroidX;
        float scaleY = camPoints[0] (1, i) - centroidY;
        scale += sqrt(scaleX * scaleX + scaleY * scaleY);
    }

    scale /= camPoints[0].cols();

    scale = sqrt(2.0) / scale;

    Eigen::MatrixXd isoMat(3,3);
    isoMat(0,0) = scale;
    isoMat(0,1) = 0.0;
    isoMat(0,2) = -scale * centroidX;

    isoMat(1,0) = 0.0;
    isoMat(1,1) = scale;
    isoMat(1,2) = -scale * centroidY;

    isoMat(2,0) = 0.0;
    isoMat(2,1) = 0.0;
    isoMat(2,2) = 1.0;

    for(size_t i = 0; i < camPoints.size(); i++) {
        Eigen::MatrixXd pts = camPoints[i];
        camPoints[i] = isoMat * pts;
    }

    Eigen::MatrixXd isoMatInv = isoMat.inverse();

    MatrixXd chessboardPoints = kc.chessboard.getModelCBH2D();
    std::vector<MatrixXd> homographies = computeHomographiesFromHomogeneous(chessboardPoints, camPoints);
    
    for(size_t i = 0; i < homographies.size(); i++) {
        MatrixXd H = isoMatInv * homographies[i];
        /*
        MatrixXd H(3,3);
        H(0,0) = -0.25758;
        H(0,1) = -0.0486164;
        H(0,2) = -0.735237;

        H(1,0) = -0.204497;
        H(1,1) = -0.0458009;
        H(1,2) = -0.588896;

        H(2,0) = -0.000124729;
        H(2,1) = -0.0000227149;
        H(2,2) = -0.000357682;
        */
        /*
        {{-0.25758, -0.0486164, -0.735237}, {-0.204497, -0.0458009, \
-0.588896}, {-0.000124729, -0.0000227149, -0.000357682}}
            */
        homographies[i] = H;
    }

    /*
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
    */
    
    std::vector<MatrixXd> chessboardToCamera;
    for (size_t i = 0; i < homographies.size(); i++) {
        chessboardToCamera.push_back(cameraIntrinsicMatrices[i].inverse() * homographies[i]);
        //std::cout << "homographies[" << i << "] = " << homographies[i] << std::endl;
        //std::cout << "chessboardToCamera[" << i << "] = " << chessboardToCamera[i] << std::endl;

    }

    // chessboardPoints: 3 * 24; homographies: 3 * 3
    // std::cout << chessboardPoints.rows() << " " << chessboardPoints.cols() << " " << homographies[0].rows() << " " << homographies[0].cols();
    //homographies.size()
    for (int cam = 0; cam < 1; cam++) {
        MatrixXd homography = homographies[cam];
        //MatrixXd projectedPoints = homography * chessboardPoints;
        MatrixXd projectedPoints = cameraIntrinsicMatrices[cam] * chessboardToCamera[cam] * kc.chessboard.getModelCBH2D();
        MatrixXd originalPoints = camPoints[cam];

        //std:: cout << projectedPoints << std::endl;
        std:: cout << originalPoints << std::endl;
        
        drawPoints(images[cam], projectedPoints);
        //drawPoints(images[cam], originalPoints);
       
    }

    MatrixXd betweenCameras = chessboardToCamera[1] * chessboardToCamera[0].inverse();

    std::cout << "Transformation from first camera to second camera:\n"
        << betweenCameras << std::endl;
    
    Vector3d r1 = betweenCameras.col(0), r2 = betweenCameras.col(1);
    r1.normalize();
    r2.normalize();
    Vector3d r3 = r1.cross(r2);

    MatrixXd rot(3, 3);
    rot.col(0) = r1;
    rot.col(1) = r2;
    rot.col(2) = r3;

    // Might need to do SVD or QR here if the point clouds look wrong

    MatrixXd transformation = MatrixXd::Identity(4, 4);
    transformation.block(0, 0, 3, 3) = rot;
    transformation.block(0, 3, 3, 1) = betweenCameras.col(2);

    std::cout << "Final rigid transformation:\n" << transformation << std::endl;

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