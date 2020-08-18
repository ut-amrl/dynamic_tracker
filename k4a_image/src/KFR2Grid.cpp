#include "KFR2Grid.h"
#include "k4a_utils.h"


KFR2Grid::KFR2Grid(k4a_calibration_t sensor_calibration){
	calibration = sensor_calibration;
}

KFR2Grid::~KFR2Grid(){}

void KFR2Grid::receiveFrame(k4a_capture_t capture){
	// Initialize pointcloud structures
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                     &xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                 calibration.depth_camera_calibration.resolution_width,
                 calibration.depth_camera_calibration.resolution_height,
                 calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                 &point_cloud);

    create_xy_table(&calibration, xy_table);

    k4a_capture_reference(capture);

    depth_image = k4a_capture_get_depth_image(capture);

    int point_count = 0;
    generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);


    // toLaser();

    // Release
    k4a_image_release(depth_image);
    k4a_image_release(point_cloud);
    k4a_image_release(xy_table);
    k4a_capture_release	(capture);	
}

sensor_msgs::LaserScan KFR2Grid::toLaser(){
	
	// Data we have:
	// k4a_image_t depth_image, xy_table, point_cloud;
	// We know how to get point_coud_data

	// Where pointclout_to_laserscan starts:
	// sensor_msgs::PointCloud2
	// With goal to fill:
	sensor_msgs::LaserScan output;

	// Questions: What info is in the header?
	// 			  -> Probably a timestamp necessary
	uint64_t time;
	time = k4a_image_get_system_timestamp_nsec(point_cloud);
}
