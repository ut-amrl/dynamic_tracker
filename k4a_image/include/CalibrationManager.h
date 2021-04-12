#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H
#include <Eigen/Eigen>
#include <iostream>

enum AnchorType {
    CAMERA, MARKER
};
class Anchor {
private:
    AnchorType _type;
    int _index;
public:
    Anchor(AnchorType type, int index): _type(type), _index(index) {}

    AnchorType type() const {
        return _type;
    }

    int index() const {
        return _index;
    }

    bool operator==(const Anchor &other) const {
        return this->_type == other._type && this->_index == other._index;
    }

    bool operator<(const Anchor &other) const {
        if (this->_type != other._type) {
            return this->_type < other._type;
        }
        return this->_index < other._index;
    }
};

class Measurement {
private:
    Anchor _from;
    Anchor _to;
    Eigen::MatrixXd _transform;
public:
    Measurement(Anchor from, Anchor to, Eigen::MatrixXd transform): _from(from), _to(to), _transform(transform) {}
    
    Anchor from() const {return _from;}
    Anchor to() const {return _to;}
    Eigen::MatrixXd transform() const {return _transform;}
};

class Calibration {
private:
    // From an arbitrary origin to that position
    std::map<Anchor, Eigen::MatrixXd> poses;
public:
    Calibration(std::istream &in);
    Calibration(const std::vector<Measurement> &measurements);

    static Calibration readFile(std::string file);

    bool has(Anchor anchor) {
        return poses.count(anchor) > 0;
    }

    Eigen::MatrixXd translation(Anchor from, Anchor to) {
        return poses[to] * poses[from].inverse();
    }

    Anchor firstCamera();

    std::map<Anchor, Eigen::MatrixXd> posesWithOrigin(Anchor origin);

    void write(std::ostream &out);
};

#endif