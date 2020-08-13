/*
 * kinect_to_gridmap.cpp
 * An executable for 
 * 	1) pulling pointcloud data 
 * 	2) conversion to octomap
 * 	3) conversion to gridmap -> Send to ROS
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
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_msgs/GridMapList.h>
#include <grid_map_msgs/GridMap.h>
// k4a_image
#include "KinectWrapper.h"
#include "KFRDisplay.h"
#include "k4a_utils.h"
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
// using namespace k4a_utils;

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

	// Vector of KinectWrappers
    vector<shared_ptr<KinectWrapper>> kinects;
    KFRDisplay kfrDisplay;
    // Vars for grabbing point_cloud_data from each of 3 Kinects
    vector< k4a_image_t > depth_images;
    k4a_image_t tmp;
    k4a_image_t xy_table = NULL;
    k4a_image_t point_cloud = NULL;
    int point_count = 0;
    // Octomap Vars
    Pointcloud* octocloud= new Pointcloud();
    OcTree tree(0.1);
    point3d origin(0.01f, 0.01f, 0.01f);
    pose6d frame_origin(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // Gridmap Vars
    GridMap *gridMap;

    // ROS
    ros::init(argc, argv, "kinect_to_gridmap", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    ros::Publisher gmap_pub = n.advertise< grid_map_msgs::GridMapList >("/gridmap_raw", 1000);
    grid_map_msgs::GridMapList gmap_msg;
    grid_map_msgs::GridMap temp_gmap;

    // Fill kinects
    for (int i = 0; i < k4a_device_get_installed_count(); i++)
    {
        kinects.push_back(make_shared<KinectWrapper>(i, kfrDisplay));
    }
    // Initialize pointcloud structures
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     kinects[0]->calibration.depth_camera_calibration.resolution_width,
                     kinects[0]->calibration.depth_camera_calibration.resolution_height,
                     kinects[0]->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                     &xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                 kinects[0]->calibration.depth_camera_calibration.resolution_width,
                 kinects[0]->calibration.depth_camera_calibration.resolution_height,
                 kinects[0]->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                 &point_cloud);

    create_xy_table(&kinects[0]->calibration, xy_table);

    while(ros::ok()){
        ros::spinOnce();

	    for (int i = 0; i < k4a_device_get_installed_count(); i++)
	    {
	    	// Capture Depth image for each kinect
	        tmp = kinects[i]->captureDepth();
	        depth_images.push_back(tmp);
	        // Create pointcloud
	        generate_point_cloud(depth_images[i], xy_table, point_cloud, &point_count);
	        // Get the Pointcloud data from pool
	        k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);
	        int width = k4a_image_get_width_pixels(point_cloud);
	        int height = k4a_image_get_height_pixels(point_cloud);
	        for(int j = 0; j < width*height; ++j){
	        	if( isnan(point_cloud_data[j].xyz.x) ){
		      		point_cloud_data[j].xyz.x = 0.;
		      	}
		      	if( isnan(point_cloud_data[j].xyz.y) ){
		      		point_cloud_data[j].xyz.y = 0.;
		      	}
		      	if( isnan(point_cloud_data[j].xyz.z) ){
		      		point_cloud_data[j].xyz.z = 0.;
		      	}
	      	}
	      
	        // Fill ocotomap::Pointcloud
	        for(int j = 0; j < width*height; ++j){
	        	float x_, y_, z_; x_ = point_cloud_data[j].xyz.x; y_ = point_cloud_data[j].xyz.y; z_ = point_cloud_data[j].xyz.z;
	        	point3d point(x_, y_, z_);
	            octocloud->push_back(point);   
	        }
	        cout << "s1" << endl;
	        // Insert Pointcloud to OcTree
	        tree.insertPointCloud(*octocloud, frame_origin.trans());
	        cout << "s2" << endl;
	        // Clear octomap::Pointcloud
	    	
	    	// Convert tree -> gridmap
	    	bool res = GridMapOctomapConverter::fromOctomap(tree, "elevation", *gridMap);
	    	// Clear OcTree 
    		tree.clear();
    		octocloud->clear();
	    	
	    	// GridMap* gridMap -> grid_map_msgs::GridMap temp_gmap
	    	GridMapRosConverter::toMessage(*gridMap, temp_gmap);
	  		// Fill GridMapList
	    	gmap_msg.list[i] = temp_gmap;

	    }

	    
        // Publish to ROS
        gmap_pub.publish(gmap_msg);

        loop_rate.sleep();
    }



    return 0;
}