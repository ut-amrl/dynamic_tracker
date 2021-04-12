#include "K4ACaptureRecipient.h"
#include "apriltag_util.h"
#include "CalibrationManager.h"

#include <memory>

class KFRAprilTags : public K4ACaptureRecipient {
private:
    std::shared_ptr<apriltag_family_t> family;
    std::shared_ptr<apriltag_detector_t> detector;
    k4a_calibration_intrinsics_t intrinsics;
    int camIndex;
public:
    std::vector<Measurement> measurements;
    KFRAprilTags(int camIndex);
    void receiveFrame(k4a_capture_t capture);
    void getCalibration(k4a_calibration_t calib);
};
