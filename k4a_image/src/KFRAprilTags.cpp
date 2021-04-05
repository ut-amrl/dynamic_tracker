#include "KFRAprilTags.h"
#include "tag36h11.h"
#include "conversions.h"

#include <stdio.h> // Testing only

KFRAprilTags::KFRAprilTags(int camIndex): camIndex(camIndex) {
    detector.reset(apriltag_detector_create(), &apriltag_detector_destroy);
    family.reset(tag36h11_create(), &tag36h11_destroy);
    apriltag_detector_add_family(detector.get(), family.get());
}

void KFRAprilTags::getCalibration(k4a_calibration_t calib) {
    intrinsics = calib.color_camera_calibration.intrinsics;
}

Eigen::MatrixXd matd_to_eigen(matd_t *mat) {
    Eigen::MatrixXd out(mat->nrows, mat->ncols);
    for (int r = 0; r < mat->nrows; r++) {
        for (int c = 0; c < mat->ncols; c++) {
            out(r, c) = MATD_EL(mat, r, c);
        }
    }
    return out;
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
    measurements.clear();
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // If you need to save any detections, change the apriltag_detections_destroy line below

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

        apriltag_detection_info_t info = {
            .det = det,
            .tagsize = 7.5 * 0.0254,
            .fx = intrinsics.parameters.param.fx,
            .fy = intrinsics.parameters.param.fy,
            .cx = intrinsics.parameters.param.cx,
            .cy = intrinsics.parameters.param.cy
        };
        apriltag_pose_t pose;
        double pose_err = estimate_tag_pose(&info, &pose);
        
        printf("measured pose at distance %.3f with error %.3f\n", matd_vec_mag(pose.t), pose_err);

        Eigen::MatrixXd rt = Eigen::MatrixXd::Identity(4, 4);
        rt.block(0, 0, 3, 3) = matd_to_eigen(pose.R);
        rt.block(0, 3, 3, 1) = matd_to_eigen(pose.t);

        measurements.push_back(Measurement(Anchor(CAMERA, camIndex), Anchor(MARKER, det->id), rt));
    }

    apriltag_detections_destroy(detections);
    k4a_image_release(image);
    k4a_capture_release(capture);
}