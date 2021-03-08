#include "apriltag_util.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

int main()
{
    // tag36h11
    Mat frame = imread("/home/fri/Documents/henry/dynamic_tracker/k4a_image/test1.jpg");
    // imshow("3", frame);
    // waitKey(0);
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    // image_u8_t *im = image_u8_create_from_pnm("/home/fri/Documents/henry/dynamic_tracker/k4a_image/test1.jpg");
    cout << &im << endl;
    apriltag_detector_t *td = apriltag_detector_create();
    // apriltag_family_t *tf = tagStandard41h12_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);
    zarray_t *detections = apriltag_detector_detect(td, &im);

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
        // Do stuff with detections here.
    }
    // Cleanup.
    // tagStandard41h12_destroy(tf);
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);
    return 0;
}
