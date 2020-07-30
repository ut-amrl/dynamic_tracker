#include "KFRDisplay.h"

KFRDisplay::KFRDisplay()
{
}

KFRDisplay::~KFRDisplay()
{
}

void KFRDisplay::receiveFrame(cv::Mat &colorMat)
{
    cv::imshow("Image", colorMat);
    int wait = cv::waitKey(0);
    //printf("res:%4dx%4d\n", rows, cols);
}