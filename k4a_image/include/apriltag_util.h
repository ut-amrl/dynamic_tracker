#ifndef APRIL_TAG_UTIL_H
#define APRIL_TAG_UTIL_H

#include "apriltag.h"
#include "apriltag_pose.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

class AprilTagPose
{
private:
    // Size of tag in meters
    double _tagsize;
    // Camera's focal length in pixels
    double _fx;
    double _fy;
    // Camera's focal center in pixels
    double _cx;
    double _cy;

public:
    // Store fixed camera & tag parameters
    AprilTagPose(double tagsize, double fx, double fy, double cx, double cy) : _tagsize(tagsize), _fx(fx), _fy(fy), _cx(cx), _cy(cy) {}

    // Find pose of detection using fixed params
    // Returns error
    double getPose(apriltag_detection_t *det, apriltag_pose_t *pose)
    {
        apriltag_detection_info_t info = {det, _tagsize, _fx, _fy, _cx, _cy};
        return estimate_tag_pose(&info, pose);
    }
};

// Convert apriltag_pose_t to Eigen matrix
/*
Eigen::MatrixXd aprilTagPoseToEigen(apriltag_pose_t *pose)
{
    Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
    std::cout << "Converting " << pose->R->nrows << "x" << pose->R->ncols << "rotation matrix" << std::endl;
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            res(row, col) = pose->R->data[row * pose->R->nrows + col];
        }
    }
    for (int row = 0; row < 3; row++)
    {
        res(row, 3) = pose->t->data[row];
    }
    return res;
}
*/

#endif