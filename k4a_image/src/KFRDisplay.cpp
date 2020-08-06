#include "KFRDisplay.h"

KFRDisplay::KFRDisplay(int rows, int cols) : _rows(rows), _cols(cols) {
    //free(_colorMat.data);    
}

KFRDisplay::~KFRDisplay() {}

void KFRDisplay::receiveFrame(k4a_capture_t capture) {
    k4a_capture_reference(capture);
    k4a_image_t image = k4a_capture_get_color_image(capture);
    uint8_t *buffer = k4a_image_get_buffer(image);
    cv::Mat colorMat =
        cv::Mat(_rows, _cols, CV_8UC4, (void *)buffer, cv::Mat::AUTO_STEP).clone();
    cv::imshow("Image", colorMat);
    int wait = cv::waitKey(1);
    k4a_image_release(image);
    k4a_capture_release	(capture);	
    //printf("res:%4dx%4d\n", rows, cols);
}