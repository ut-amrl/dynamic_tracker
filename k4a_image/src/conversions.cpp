#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include "conversions.h"

cv::Mat kinect_to_cv(k4a_image_t &image) {
    k4a_image_reference(image);
    uint8_t *buffer = k4a_image_get_buffer(image);
    cv::Mat colorMat =
        cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image),
        CV_8UC4, (void *)buffer, cv::Mat::AUTO_STEP).clone();
    k4a_image_release(image);
    return colorMat;
}