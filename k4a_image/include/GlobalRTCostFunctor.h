#ifndef GLOBAL_RT_COST_FUNCTOR_H
#define GLOBAL_RT_COST_FUNCTOR_H

#include "vision_geometry/HomCartShortcuts.h"
#include "vision_geometry/LinearAlgebraShortcuts.h"

#include <Eigen/Dense>

using namespace Eigen;

class GlobalRTCostFunctor {
public:
    GlobalRTCostFunctor(const Matrix3d &intrinsics, const Vector2d &image_corners, const Vector3d &world_corner)
    {
    }

    template <typename T>
    bool operator()(const T* const objRt, const T* const camRt, T* residuals) const
    {
        MatrixXd(3,4);
        return true;
    }

protected:
};

#endif
