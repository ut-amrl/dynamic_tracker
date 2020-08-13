#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "vision_geometry/CameraIntrinsics.h"

CameraIntrinsics k(1834.17,
    1833.1, 0.,
    1910.01,
    1113.17);

int main()
{
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    kc.test();
}