#ifndef KFR_2_GRID_H
#define KFR_2_GRID_H


// k4a_image
#include "K4ACaptureRecipient.h"
#include "k4a_utils.h"
#include <k4a/k4a.h>
// ROS
#include <ros/ros.h>
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
// C++ / C
#include <cstdio>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <stdio.h>
// #include <sys/types.h>
// #include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <memory>


class KFR2Grid : public K4ACaptureRecipient {
public:
    KFR2Grid(k4a_calibration_t sensor_calibration);
    
    ~KFR2Grid();
    
    void receiveFrame(k4a_capture_t capture);

    sensor_msgs::LaserScan toLaser();

protected:
    k4a_image_t depth_image, xy_table, point_cloud;
    k4a_calibration_t calibration;
};

#endif