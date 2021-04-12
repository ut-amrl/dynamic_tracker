#include "CalibrationManager.h"
#include <fstream>

Calibration::Calibration(const std::vector<Measurement> &measurements) {
    Anchor origin = measurements[0].from();
    poses[origin] = Eigen::MatrixXd::Identity(4, 4);
    
    bool changed;
    do {
        changed = false;

        for (size_t i = 0; i < measurements.size(); i++) {
            const Measurement &m = measurements[i];
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
}

Anchor Calibration::firstCamera() {
    for (std::map<Anchor, Eigen::MatrixXd>::iterator it = poses.begin(); it != poses.end(); it++) {
        if (it->first.type() == CAMERA) {
            return it->first;
        }
    }
    throw "no cameras in this calibration ??";
}

std::map<Anchor, Eigen::MatrixXd> Calibration::posesWithOrigin(Anchor origin) {
    std::map<Anchor, Eigen::MatrixXd> out;
    for (std::map<Anchor, Eigen::MatrixXd>::iterator it = poses.begin(); it != poses.end(); it++) {
        out[it->first] = translation(origin, it->first);
    }
    return out;
}

Eigen::MatrixXd readMatrix(std::istream &input, int rows, int cols) {
    Eigen::MatrixXd mat(rows, cols);
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            input >> mat(row, col);
        }
    }
    return mat;
}

void writeMatrix(std::ostream &out, Eigen::MatrixXd matrix) {
    for (int row = 0; row < matrix.rows(); row++) {
        for (int col = 0; col < matrix.cols(); col++) {
            if (col > 0) {
                out << " ";
            }
            out << matrix(row, col);
        }
        out << "\n";
    }
}

Calibration::Calibration(std::istream &in) {
    int numPoses = 0;
    in >> numPoses;
    for (int i = 0; i < numPoses; i++) {
        int type;
        int index;
        in >> type;
        in >> index;

        Eigen::MatrixXd mat = readMatrix(in, 4, 4);
        poses[Anchor(AnchorType(type), index)] = mat;
    }
}

void Calibration::write(std::ostream &out) {
    out << poses.size() << "\n";
    for (std::map<Anchor, Eigen::MatrixXd>::iterator it = poses.begin(); it != poses.end(); it++) {
        out << it->first.type() << " "  << it->first.index() << "\n";
        writeMatrix(out, it->second);
    }
}

Calibration Calibration::readFile(std::string name) {
    std::ifstream in(name);
    return Calibration(in);
}
