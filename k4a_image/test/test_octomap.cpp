
/*
 * test_octomap.cpp
 * A script for testing pulling pcl data from Azure Kinect
 * and creating an octomap. This octomap is then used to create a Gridmap 
 * before being sent down to ROS.
 *      Author: Ryan Gupta
 *   University of Texas at Austin, Human Centered Robotics Lab
 */

// OctoMap
#include "octomap/octomap.h"
#include "octomap/OccupancyOcTreeBase.h"
#include "octomap/Pointcloud.h"
#include <octomap/math/Utils.h>
// Grid map
#include <grid_map_core/GridMap.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
// k4a_image
#include "KinectWrapper.h"
#include "KFRDisplay.h"
// ROS
#include <ros/ros.h>
#include "ros/package.h"
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

using namespace octomap;
using namespace std;
using namespace cv;
using namespace Eigen;
using namespace grid_map;

static void generate_point_cloud(const k4a_image_t depth_image,
                                 const k4a_image_t xy_table,
                                 k4a_image_t point_cloud,
                                 int *point_count)
{
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}


static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}


int main(int argc, char **argv){

    // // Array of Kinect Wrappers
    // vector<shared_ptr<KinectWrapper>> kinects;
    // KFRDisplay kfrDisplay;
    // // Array of cv::Mat results from get_depth
    // vector<k4a_image_t> depth_images;
    // k4a_image_t tmp;
    // k4a_image_t xy_table = NULL;
    // k4a_image_t point_cloud = NULL;
    // int point_count = 0;
    // // Octomap Vars
    // Pointcloud* octocloud= new Pointcloud();
    // OcTree tree(0.1);
    // point3d origin(0.01f, 0.01f, 0.01f);

    // // Fill kinects
    // for (int i = 0; i < k4a_device_get_installed_count(); i++)
    // {
    //     kinects.push_back(make_shared<KinectWrapper>(i, kfrDisplay));
    // }

    // k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
    //                  kinects[0]->calibration.depth_camera_calibration.resolution_width,
    //                  kinects[0]->calibration.depth_camera_calibration.resolution_height,
    //                  kinects[0]->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
    //                  &xy_table);

    // create_xy_table(&kinects[0]->calibration, xy_table);

    // k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
    //              kinects[0]->calibration.depth_camera_calibration.resolution_width,
    //              kinects[0]->calibration.depth_camera_calibration.resolution_height,
    //              kinects[0]->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
    //              &point_cloud);

    // // Capture Depth image for each kinect
    // for (int i = 0; i < k4a_device_get_installed_count(); i++)
    // {
    //     tmp = kinects[i]->captureDepth();
    //     depth_images.push_back(tmp);
    //     // Create pointcloud
    //     generate_point_cloud(depth_images[i], xy_table, point_cloud, &point_count);
    //     // Get the data
    //     k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);
    //     int width = k4a_image_get_width_pixels(point_cloud);
    //     int height = k4a_image_get_height_pixels(point_cloud);
    //     for(int j = 0; j < width*height; ++j){
    //         // Fill ocotomap::Pointcloud
    //         octocloud->push_back(point_cloud_data[j].xyz.x, point_cloud_data[j].xyz.y, point_cloud_data[j].xyz.z);   
    //     }
    //     cout << "Octomap Pointcloud" << endl;
    //     // cout << octocloud << endl;
    //     tree.insertPointCloud(*octocloud, origin);
    // }

    // // I think at this point I have included that pointcloud as an OcTreeNode?
    // // Need to figure this relationship out of how do i get OcTreeNodes
    //     // from an OcTreeNode I can use method: getOccupancy() to obtain the occupancy probability of a node


    
    // //Do stuff with OcTree tree
    // for(Pointcloud::iterator it = octocloud->begin(); it != octocloud->end(); it++){
    // 	float x, y, z;
    //     x = (*it)(0);
    //     y = (*it)(1);
    //     z = (*it)(2);
      
    // }

    // // ros::init(argc, argv, "octomap_to_gridmap", ros::init_options::NoSigintHandler);
    // // ros::NodeHandle ros_node;

    // // Convert to grid map.
    // GridMap gridMap;    
    // bool res = GridMapOctomapConverter::fromOctomap(tree, "elevation", gridMap);


    // octocloud->clear();
    // tree.clear();

    
	
}
