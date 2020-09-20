#include "KFRDisplay.h"
#include "conversions.h"

KFRDisplay::KFRDisplay(int rows, int cols) : _rows(rows), _cols(cols) {
    //free(_colorMat.data);    
}

KFRDisplay::~KFRDisplay() {}

void KFRDisplay::receiveFrame(k4a_capture_t capture) {
    k4a_capture_reference(capture);
    k4a_image_t image = k4a_capture_get_color_image(capture);
    cv::Mat colorMat = kinect_to_cv(image);
    cv::imshow("Image", colorMat);
    int wait = cv::waitKey();
    k4a_image_release(image);
    k4a_capture_release	(capture);	
    //printf("res:%4dx%4d\n", rows, cols);
}