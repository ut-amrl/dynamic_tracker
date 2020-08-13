
// k4a_image
#include "KinectWrapper.h"
#include "KFRDisplay.h"
#include <math.h>
// C++ / C
#include <cstdio>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <stdio.h>
#include <string>
#include <fstream>
#include <sstream>
// #include <sys/types.h>
// #include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <memory>

using namespace std;
using namespace cv;
using namespace Eigen;


// namespace k4a_utils{
	static void generate_point_cloud(const k4a_image_t depth_image,
                                 const k4a_image_t xy_table,
                                 k4a_image_t point_cloud,
                                 int *point_count);


	static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table);

// }