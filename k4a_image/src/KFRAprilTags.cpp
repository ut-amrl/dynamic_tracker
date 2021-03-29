#include "KFRAprilTags.h"
#include "tag36h11.h"
#include "conversions.h"

#include <stdio.h> // Testing only

KFRAprilTags::KFRAprilTags() {
    detector.reset(apriltag_detector_create(), &apriltag_detector_destroy);
    family.reset(tag36h11_create(), &tag36h11_destroy);
    apriltag_detector_add_family(detector.get(), family.get());
}

void KFRAprilTags::receiveFrame(k4a_capture_t capture) {
    k4a_capture_reference(capture);

    k4a_image_t image = k4a_capture_get_color_image(capture);
    cv::Mat colorMat = kinect_to_cv(image);

    cv::Mat grayMat;
    cv::cvtColor(colorMat, grayMat, CV_BGR2GRAY);

    image_u8 aprilImage = {
        .width = grayMat.cols,
        .height = grayMat.rows,
        .stride = grayMat.cols,
        .buf = grayMat.data
    };
    zarray_t *detections = apriltag_detector_detect(detector.get(), &aprilImage);

    printf("\n%d detections\n", zarray_size(detections));
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // If you need to save any detections, change the apriltag_detections_destroy line below

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
    }

    apriltag_detections_destroy(detections);
    k4a_image_release(image);
    k4a_capture_release(capture);
}