#ifndef KFR_CALIB_H
#define KFR_CALIB_H

#include "Camera.h"
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/ModelChessboard.h>
#include <vision_geometry/RigidTrans.h>

class KinectCalibrator {
public:
    KinectCalibrator(std::string path, CameraIntrinsics k);
    ~KinectCalibrator();
    void select();
    void solve();
    void globalOpt(std::vector<Camera> cams, std::vector<Eigen::MatrixXd> camRTs, std::vector<Eigen::MatrixXd> objRTs);

protected:
    std::vector<std::string> imgSets;
    CameraIntrinsics intrinsics;
    ModelChessboard chessboard;
};

#endif