#ifndef GLOBAL_RT_COST_FUNCTOR_H
#define GLOBAL_RT_COST_FUNCTOR_H

#include "vision_geometry/HomCartShortcuts.h"
#include "vision_geometry/LinearAlgebraShortcuts.h"

#include <Eigen/Dense>

using namespace Eigen;

class GlobalRTCostFunctor {
public:
    GlobalRTCostFunctor(const Matrix3d& intrinsics, const Vector2d& image_corners, const Vector4d& world_corner)
        : intrinsics_(intrinsics)
    {
        X_world = world_corner[0];
        Y_world = world_corner[1];
        Z_world = world_corner[2];
        W_world = world_corner[3];

        x_image = image_corners[0];
        y_image = image_corners[1];
    }

    template <typename T>
    bool operator()(const T* const camRt, const T* const objRt, T* residuals) const
    {
        // Cam pose params
        T camq[4];
        camq[0] = camRt[0];
        camq[1] = camRt[1];
        camq[2] = camRt[2];
        camq[3] = camRt[3];

        T camt[3];
        camt[0] = camRt[4];
        camt[1] = camRt[5];
        camt[2] = camRt[6];

        Quaternion<T> camQ = Map<const Eigen::Quaternion<T>>(camq);
        Matrix<T, 3, 1> camT = Map<const Eigen::Matrix<T, 3, 1>>(camt);
        Matrix<T, 3, 4> camRT;
        camRT.block(0, 0, 3, 3) = camQ.toRotationMatrix();
        camRT.block(0, 3, 3, 1) = camT;

        // Object pose params
        T objq[4];
        objq[0] = objRt[0];
        objq[1] = objRt[1];
        objq[2] = objRt[2];
        objq[3] = objRt[3];

        T objt[3];
        objt[0] = objRt[4];
        objt[1] = objRt[5];
        objt[2] = objRt[6];

        Quaternion<T> objQ = Map<const Eigen::Quaternion<T>>(objq);
        Matrix<T, 3, 1> objT = Map<const Eigen::Matrix<T, 3, 1>>(objt);
        Matrix<T, 4, 4> objRT = Matrix<T, 4, 4>::Identity();
        objRT.block(0, 0, 3, 3) = objQ.toRotationMatrix();
        objRT.block(0, 3, 3, 1) = objT;

        // Template other ponts
        Eigen::Matrix<T, 4, 1> worldPoint;
        worldPoint << T(X_world), T(Y_world), T(Z_world), T(W_world);

        Eigen::Matrix<T, 3, 3> intrinsics;
        intrinsics << T(intrinsics_(0, 0)), T(intrinsics_(0, 1)), T(intrinsics_(0, 2)),
            T(intrinsics_(1, 0)), T(intrinsics_(1, 1)), T(intrinsics_(1, 2)),
            T(intrinsics_(2, 0)), T(intrinsics_(2, 1)), T(intrinsics_(2, 2));

        Eigen::Matrix<T, 3, 1> res = intrinsics * camRT * objRT.inverse()  * worldPoint;

        residuals /= residuals[2];
        residuals[0] = res[0] - T(x_image);
        residuals[1] = res[1] - T(y_image);

        return true;
    }

protected:
    const Matrix3d& intrinsics_;
    double X_world;
    double Y_world;
    double Z_world;
    double W_world;
    double x_image;
    double y_image;
};

#endif
