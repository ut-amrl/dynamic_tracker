#include "KinectCalibrator.h"
#include "ceres/ceres.h"
#include "vision_geometry/CameraIntrinsics.h"

#include "KinectWrapper.h"
#include "KFRCalibration.h"

CameraIntrinsics k(1834.17,
    1833.1, 0.,
    1910.01,
    1113.17);

int main()
{
    KinectCalibrator kc("/home/henry/Desktop/images/", k);
    //kc.select();
    kc.test();

    // Internal sizes - that is, the number of points where the squares touch
    KFRCalibration handler(6, 4);
    for (int device = 0; device < 2; device++) {
        KinectWrapper wrapper(device, handler);
        wrapper.capture();

        std::cout << handler.corners << std::endl;
    }
}