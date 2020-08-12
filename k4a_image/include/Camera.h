#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <map>

using namespace Eigen;

struct Camera {
    Camera() {};
    std::map<int, MatrixXd> projections;
    int index;
};

#endif

