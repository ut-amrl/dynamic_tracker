#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "vision_geometry/CameraIntrinsics.h"

CameraIntrinsics k(1,1,0,1,1);

int main(){
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    kc.test();
}