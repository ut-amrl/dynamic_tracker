#include "CalibrationManager.h"

Calibration consolidateMeasurements(std::vector<Measurement> &measurements) {
    std::map<Anchor, Eigen::MatrixXd> poses;

    Anchor origin = measurements[0].from();
    poses[origin] = Eigen::MatrixXd::Identity(4, 4);
    
    bool changed;
    do {
        changed = false;

        for (size_t i = 0; i < measurements.size(); i++) {
            Measurement &m = measurements[i];
            if (poses.count(m.from()) && !poses.count(m.to())) {
                poses[m.to()] = m.transform() * poses[m.from()];
                changed = true;
            }
            if (poses.count(m.to()) && !poses.count(m.from())) {
                poses[m.from()] = m.transform().inverse() * poses[m.to()];
                changed = true;
            }
        }
    } while (changed);

    return Calibration(poses);
}