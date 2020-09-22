#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "vision_geometry/CameraIntrinsics.h"

#include "KinectWrapper.h"
#include "KFRCalibration.h"

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
    v(6) = 1;
    return v;
}

int main()
{
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    kc.test();

    // Internal sizes - that is, the number of points where the squares touch
    int chessboardRows = 6;
    int chessboardCols = 4;
    double chessboardSpacing = 1.0; // TODO
    kc.chessboard = ModelChessboard(chessboardRows, chessboardCols, chessboardSpacing);

    KFRCalibration handler(chessboardRows, chessboardCols);
    std::vector<Camera> cameras;
    for (int device = 0; device < 2; device++) {
        KinectWrapper wrapper(device, handler);
        wrapper.capture();
        Camera current;
        current.projections[0] = MatrixXd(2, handler.corners.size());
        for(int i = 0; i < handler.corners.size(); i++) {
            current.projections[0](0, i) = handler.corners[i].x;
            current.projections[0](1, i) = handler.corners[i].y;
        }
        current.index = device;
        cameras.push_back(current);

        std::cout << handler.corners << std::endl;
    }

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
}