#include "K4ACaptureRecipient.h"
#include "apriltag_util.h"

#include <memory>

class KFRAprilTags : public K4ACaptureRecipient {
private:
    std::shared_ptr<apriltag_detector_t> detector;
    std::shared_ptr<apriltag_family_t> family;
public:
    KFRAprilTags();
    void receiveFrame(k4a_capture_t capture);
};
