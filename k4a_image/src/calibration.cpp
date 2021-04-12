#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "vision_geometry/CameraIntrinsics.h"

#include "KinectWrapper.h"
#include "KFRCalibration.h"
#include "vision_geometry/CVUtil.h"
#include "vision_geometry/HomographyShortcuts.h"
#include "vision_geometry/HomCartShortcuts.h"
#include "vision_geometry/LinearAlgebraShortcuts.h"

#include "calibration.h"
    
template <typename T>
Eigen::Matrix<T, -1, -1> explicitConvertMatrix(Eigen::Matrix<double, -1, -1> input) {
    Eigen::Matrix<T, -1, -1> out(input.rows(), input.cols());
    for (int row = 0; row < input.rows(); row++) {
        for (int col = 0; col < input.cols(); col++) {
            out(row, col) = T(input(row, col));
        }
    }
    return out;
}

struct CostFunctor {
private:
    Eigen::MatrixXd intrinsics;
    Eigen::VectorXd objectPoint, expectedImagePoint;
public:
    CostFunctor(Eigen::MatrixXd intrinsics, Eigen::VectorXd objectPoint, Eigen::VectorXd expectedImagePoint): intrinsics(intrinsics),
        objectPoint(objectPoint), expectedImagePoint(expectedImagePoint) {
            expectedImagePoint /= expectedImagePoint(2);
        }
 
    template <typename T>
    bool operator()(const T* const r, const T* const x, T* residual) const {
        T rotMat[9];
        ceres::AngleAxisToRotationMatrix(r, rotMat);

        Matrix<T, -1, -1> extrinsics = Eigen::Matrix<T, -1, -1>(3, 4);
        extrinsics(0, 0) = rotMat[0];
        extrinsics(0, 1) = rotMat[3];
        extrinsics(0, 2) = rotMat[6];
        extrinsics(0, 3) = x[0];
        extrinsics(1, 0) = rotMat[1];
        extrinsics(1, 1) = rotMat[4];
        extrinsics(1, 2) = rotMat[7];
        extrinsics(1, 3) = x[1];
        extrinsics(2, 0) = rotMat[2];
        extrinsics(2, 1) = rotMat[5];
        extrinsics(2, 2) = rotMat[8];
        extrinsics(2, 3) = x[2];

        Matrix<T, -1, -1> projection = explicitConvertMatrix<T>(intrinsics) * extrinsics;
        Matrix<T, -1, -1> projectedPoint = projection * explicitConvertMatrix<T>(objectPoint);
        projectedPoint /= projectedPoint(2, 0);

        T xDiff = projectedPoint(0, 0) - expectedImagePoint(0, 0);
        T yDiff = projectedPoint(1, 0) - expectedImagePoint(1, 0);

        residual[0] = xDiff;
        residual[1] = yDiff;
        return true;
    }
};

// projectedPoints represents the 2D points to draw. It has two or three rows; the X is row 0 / row 2, the Y is row 1 / row 2.
// Each column is one point.
void drawPoints(cv::Mat background, Eigen::MatrixXd projectedPoints) {
    cv::Mat image;
    background.copyTo(image);

    // std::vector<cv::Point2d> cvPoints;
    for (int point = 0; point < projectedPoints.cols(); point++) {
        double x = projectedPoints(0, point);
        double y = projectedPoints(1, point);
        if (projectedPoints.rows() > 2) {
            x /= projectedPoints(2, point);
            y /= projectedPoints(2, point);
        }
        //cvPoints.push_back(cv::Point2d(x, y));
        if (point == 0) drawCrosshair(image, x, y, 10, 2, cv::Scalar(0, 0, 255));
        else if (point == 1) drawCrosshair(image, x, y, 10, 2, cv::Scalar(0, 255, 0));
        else drawCrosshair(image, x, y, 10, 2, cv::Scalar(255, 0, 0));
    }
    //cv::drawChessboardCorners(image, cv::Size2d(chessboardCols, chessboardRows), cvPoints, true);

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

void printMatrix(MatrixXd mat);

MatrixXd computeUnoptimizedTransformation(MatrixXd homography) {
    Vector3d r1 = homography.col(0), r2 = homography.col(1);
    double r1Norm = r1.norm(), r2Norm = r2.norm();
    r1.normalize();
    r2.normalize();
    Vector3d r3 = r1.cross(r2);

    Vector3d t = homography.col(2);
    t /= 0.5 * (r1Norm + r2Norm);    

    Matrix3d rot(3, 3);
    rot.col(0) = r1;
    rot.col(1) = r2;
    rot.col(2) = r3;
    
    MatrixXd transform = MatrixXd::Identity(4, 4);
    transform.block(0, 0, 3, 3) = rot;
    transform.block(0, 3, 3, 1) = t;

    return transform;
}

MatrixXd computeTransformation(MatrixXd homography, MatrixXd objectPoints, MatrixXd camPoints, 
MatrixXd cameraIntrinsics) {
    ceres::Problem problem;

    Vector3d r1 = homography.col(0), r2 = homography.col(1);
    double r1Norm = r1.norm(), r2Norm = r2.norm();
    r1.normalize();
    r2.normalize();
    Vector3d r3 = r1.cross(r2);

    Vector3d t = homography.col(2);
    t /= 0.5 * (r1Norm + r2Norm);

    Matrix3d rot(3, 3);
    rot.col(0) = r1;
    rot.col(1) = r2;
    rot.col(2) = r3;

    JacobiSVD<MatrixXd> svd(rot, ComputeFullU | ComputeFullV);
    rot = svd.matrixU() * svd.matrixV().transpose();
    //rot = rot.jacobiSvd().solve();

    MatrixXd transformation = MatrixXd::Identity(4, 4);
    transformation.block(0, 0, 3, 3) = rot;
    transformation.block(0, 3, 3, 1) = t;

    cv::Mat rot2(3, 3, CV_64F);
    rot2.at<double>(0, 0) = r1(0, 0);
    rot2.at<double>(0, 1) = r2(0, 0);
    rot2.at<double>(0, 2) = r3(0, 0);
    rot2.at<double>(1, 0) = r1(1, 0);
    rot2.at<double>(1, 1) = r2(1, 0);
    rot2.at<double>(1, 2) = r3(1, 0);
    rot2.at<double>(2, 0) = r1(2, 0);
    rot2.at<double>(2, 1) = r2(2, 0);
    rot2.at<double>(2, 2) = r3(2, 0);

    // Might need to do SVD or QR here if the point clouds look wrong

    std::cout << "Reprojected points" << std::endl;
    MatrixXd reprojected = transformation * objectPoints;
    printMatrix(reprojected);

    cv::Mat rotationTerms(3, 1, CV_64F);
    cv::Rodrigues(rot2, rotationTerms); // TODO is this the right parameters? We use the values in rotationTerms not rotVect

    //Eigen::AngleAxisd angleAxis(rot);
    //Eigen::Vector3d rodrigues = angleAxis.axis() * angleAxis.angle();

    double parameters[] = {
        rotationTerms.at<double>(0, 0),
        rotationTerms.at<double>(1, 0),
        rotationTerms.at<double>(2, 0),
        //rodrigues(0), rodrigues(1), rodrigues(2),
        transformation(0, 3),
        transformation(1, 3),
        transformation(2, 3)
    };

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    for (int i = 0; i < objectPoints.cols(); i++) {
        Eigen::Vector4d objectPoint = objectPoints.col(i);
        Eigen::Vector3d imagePoint = camPoints.col(i);
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CostFunctor, 2, 3, 3>(new CostFunctor(cameraIntrinsics, objectPoint, imagePoint));
        problem.AddResidualBlock(cost_function, NULL, &parameters[0], &parameters[3]);
    }

    // Run the solver!
    ceres::Solver::Options options;
    options.line_search_direction_type = ceres::BFGS;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10000;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "rotationTerms[0] : " << parameters[0] << std::endl;

    rotationTerms.at<double>(0, 0) = parameters[0];
    rotationTerms.at<double>(1, 0) = parameters[1];
    rotationTerms.at<double>(2, 0) = parameters[2];

    cv::Matx33d outputRotationMatrix;
    cv::Rodrigues(rotationTerms, outputRotationMatrix);
    transformation(0, 0) = outputRotationMatrix(0, 0);
    transformation(0, 1) = outputRotationMatrix(0, 1);
    transformation(0, 2) = outputRotationMatrix(0, 2);
    transformation(1, 0) = outputRotationMatrix(1, 0);
    transformation(1, 1) = outputRotationMatrix(1, 1);
    transformation(1, 2) = outputRotationMatrix(1, 2);
    transformation(2, 0) = outputRotationMatrix(2, 0);
    transformation(2, 1) = outputRotationMatrix(2, 1);
    transformation(2, 2) = outputRotationMatrix(2, 2);

    /*rodrigues(0) = parameters[0];
    rodrigues(1) = parameters[1];
    rodrigues(2) = parameters[2];
    angleAxis = AngleAxisd(rodrigues.norm(), rodrigues.normalized());
    transformation.block(0, 0, 3, 3) = angleAxis.toRotationMatrix();*/

    transformation(0, 3) = parameters[3];
    transformation(1, 3) = parameters[4];
    transformation(2, 3) = parameters[5];

    return transformation;
}

void printMatrix(MatrixXd mat) {
    for (size_t row = 0; row < mat.rows(); row++) {
        for (size_t col = 0; col < mat.cols(); col++) {
            std::cout << mat(row, col);
            if (col == mat.cols() - 1) {
                if (row == mat.rows() - 1) {
                    std::cout << ";\n";
                } else {
                    std::cout << ",\n";
                }
            } else {
                std::cout << ", ";
            }
        }
    } 
}

MatrixXd createIsometricConversionHomography(MatrixXd pts) {
    float centroidX = 0;
    float centroidY = 0;
    for(int i = 0; i < pts.cols(); i++) {
        centroidX += pts(0, i);
        centroidY += pts(1, i);
    }

    centroidX /= pts.cols();
    centroidY /= pts.cols();

    float scale = 0;
    for(int i = 0; i < pts.cols(); i++) {
        float scaleX = pts(0, i) - centroidX;
        float scaleY = pts(1, i) - centroidY;
        scale += sqrt(scaleX * scaleX + scaleY * scaleY);
    }

    scale /= pts.cols();

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

    return isoMat;
}

MatrixXd computeRTModelToCamera(MatrixXd modelPointsH2D, MatrixXd camPointsH2D, MatrixXd intrinsicHomography) {
    Eigen::MatrixXd isoMat = createIsometricConversionHomography(camPointsH2D);

    Eigen::MatrixXd normalizedHomography = computeDLTHomography(modelPointsH2D, isoMat * camPointsH2D);
    
    MatrixXd H = isoMat.inverse() * normalizedHomography;
    MatrixXd chessboardToCameraHomography = intrinsicHomography.inverse() * H;
    //return computeUnoptimizedTransformation(chessboardToCameraHomography);

    MatrixXd modelPointsH3D(4, modelPointsH2D.cols());
    modelPointsH3D.row(0) = modelPointsH2D.row(0);
    modelPointsH3D.row(1) = modelPointsH2D.row(1);
    // Leave row 2 as zeros
    modelPointsH3D.row(3) = modelPointsH2D.row(2);
    return computeTransformation(chessboardToCameraHomography, modelPointsH3D, camPointsH2D, intrinsicHomography);
}

std::string printMathematicaString(Eigen::MatrixXd A) {
    std::stringstream ss;
    ss << std::fixed;
    ss.precision(8);
    ss << "{";
    for(size_t row = 0; row < A.rows(); row++) {
        ss << "{";
        for(size_t col = 0; col < A.cols(); col++) {
            ss << A(row, col);
            if(col < (A.cols() - 1))
                ss << ",";
        }
        ss << "}";
        if(row < (A.rows() - 1))
            ss << ",";
    }
    ss << "}";
    return ss.str();
}

bool captureChessboardCorners(int device, int chessboardRows, int chessboardCols, MatrixXd *intrinsicsOut, MatrixXd *pointsOut) {
    KFRCalibration handler(chessboardRows, chessboardCols);
    KinectWrapper wrapper(device, handler);
    if (!wrapper.capture()) {
        return false;
    }

    k4a_calibration_intrinsic_parameters_t calibration = wrapper.getCalibration().color_camera_calibration.intrinsics.parameters;
    MatrixXd cameraMatrix = MatrixXd::Identity(3, 3);
    cameraMatrix(0, 0) = calibration.param.fx;
    cameraMatrix(1, 1) = calibration.param.fy;
    cameraMatrix(0, 2) = calibration.param.cx;
    cameraMatrix(1, 2) = calibration.param.cy;

    MatrixXd cornerList(3, handler.corners.size());
    for(int i = 0; i < handler.corners.size(); i++) {
        cornerList(0, i) = handler.corners[i].x;
        cornerList(1, i) = handler.corners[i].y;
        cornerList(2, i) = 1.0;
    }

    if (intrinsicsOut != NULL) {
        *intrinsicsOut = cameraMatrix;
    }
    if (pointsOut != NULL) {
        *pointsOut = cornerList;
    }
    return true;
}
