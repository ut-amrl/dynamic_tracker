#include <Eigen/Eigen>

enum AnchorType {
    CAMERA, MARKER
};
class Anchor {
private:
    AnchorType type;
    int index;
public:
    Anchor(AnchorType type, int index): type(type), index(index) {}

    bool operator==(const Anchor &other) {
        return this->type == other.type && this->index == other.index;
    }

    bool operator<(const Anchor &other) const {
        if (this->type != other.type) {
            return this->type < other.type;
        }
        return this->index < other.index;
    }
};

class Measurement {
private:
    Anchor _from;
    Anchor _to;
    Eigen::MatrixXd _transform;
public:
    Measurement(Anchor from, Anchor to, Eigen::MatrixXd transform): _from(from), _to(to), _transform(transform) {}
    
    Anchor from() {return _from;}
    Anchor to() {return _to;}
    Eigen::MatrixXd transform() {return _transform;}
};

class Calibration {
private:
    // From an arbitrary origin to that position
    std::map<Anchor, Eigen::MatrixXd> poses;
public:
    Calibration(std::map<Anchor, Eigen::MatrixXd> poses): poses(poses) {}

    Eigen::MatrixXd translation(Anchor from, Anchor to) {
        return poses[to] * poses[from].inverse();
    }

    std::map<Anchor, Eigen::MatrixXd> posesWithOrigin(Anchor origin) {
        std::map<Anchor, Eigen::MatrixXd> out;
        for (std::map<Anchor, Eigen::MatrixXd>::iterator it = poses.begin(); it != poses.end(); it++) {
            out[it->first] = translation(origin, it->first);
        }
        return out;
    }
};

Calibration consolidateMeasurements(std::vector<Measurement> &measurements);