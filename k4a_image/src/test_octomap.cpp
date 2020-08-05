#include "octomap/octomap.h"
#include "octomap/OccupancyOcTreeBase.h"
#include "octomap/Pointcloud.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace octomap;
using namespace std;
using namespace cv;

int main(int argc, char **argv){

	// Need to create Octomap Pointcloud Object
	Pointcloud cloud;

	// Get count of cameras anbd open them 
	uint32_t device_count = k4a_device_get_installed_count();
	printf("Found %d connected devices:\n", device_count);	
	
	// Configure a stream of 4096x3072 BRGA color data at 15 frames per second
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps       = K4A_FRAMES_PER_SECOND_30;
	config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
		

	k4a_device_t device = NULL;

	for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
	{
		
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
	        {
			printf("%d: Failed to open kinect\n", deviceIndex);
			continue;
		}
		k4a_device_start_cameras(device, &config);
				   
	}


	// Anything here for now is only on the last device started
	// Capture a depth frame
	
	k4a_capture_t capture;

	switch (k4a_device_get_capture(device, &capture, 1000))//TIMEOUT_IN_MS))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
	     	printf("Timed out waiting for a capture\n");
	     	
	    	break;
	case K4A_WAIT_RESULT_FAILED:
		printf("Failed to read a capture\n");
		break;
	}

	k4a_image_t image = k4a_capture_get_depth_image(capture);
	if (image != NULL)
	{
		printf(" | Depth16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(image),
			k4a_image_get_width_pixels(image),
			k4a_image_get_stride_bytes(image));
	cout << "s1" << endl;	    
		cout << image << endl;

		// Release the image
		k4a_image_release(image);
	}// Get depth image
	

	// Release the capture
	k4a_capture_release(capture);

	//for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
	//{
		
	//	if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
	//        {
	//		printf(%d: Failed to open device\n, deviceIndex);
	//		continue;
	//	}
		k4a_device_stop_cameras(device);
		k4a_device_close(device);
				   
	//}	
	
}
