#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#include "KinectWrapper.h"
#include <vision_geometry/CVUtil.h>
#include <vision_geometry/Util.h>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/HomographyShortcuts.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>
#include <vision_geometry/HomCartShortcuts.h>
#include <sockets.h>
#include <config.h>
#include "KFRDisplay.h"

using namespace cv;
using namespace Eigen;
using namespace std;

// Capture images from all cameras and send corners (if found) back to server
void captureAllCorners(int sockfd, vector<shared_ptr<KinectWrapper>> &kinects)
{
    for (shared_ptr<KinectWrapper> kinect : kinects)
    {
        cv::Size s(CBOARD_ROWS, CBOARD_COLS);
        Mat image;// = kinect->capture();
        abort();//Sorry! This will need to be fixed. Look at KinectDisplay for a reference on how to do this the new way.
        // Kinect outputs BGRA so we need to convert to BGR for findChessboardCorners
        Mat convert;
        cvtColor(image, convert, CV_BGRA2BGR);
        Mat detections;
        MatrixXd camPoints;
        if (!detectFrameCorners(s, convert, detections, camPoints))
        {
            // indicate null if not found
            MatrixXd replace(2, 1);
            replace << -1, -1;
            camPoints = replace;
        }

        sendEigen(sockfd, camPoints);
    }
}

void captureImages(int sockfd, vector<shared_ptr<KinectWrapper>> &kinects, bool markCorners)
{
    // Mat image;
    // image = imread("/home/henry/Desktop/1593827043.jpg", CV_LOAD_IMAGE_COLOR);
    for (shared_ptr<KinectWrapper> kinect : kinects)
    {
        Mat image;// = kinect->capture();
        abort();//Sorry! This will need to be fixed. Look at KinectDisplay for a reference on how to do this the new way.
        if (markCorners)
        {
            cv::Size s(CBOARD_ROWS, CBOARD_COLS);
            Mat convert;
            cvtColor(image, convert, CV_BGRA2BGR);
            Mat detections;
            MatrixXd camPoints;
            detectFrameCorners(s, convert, detections, camPoints);
        }
        sendCV(sockfd, image);
    }
}

// Send sample eigen matrix
void sampleEigen(int sockfd)
{
    MatrixXd m(2, 3);
    m << 1.234, 2.432, 3.4574456,
        4, 5, 6.654;
    cout << m << endl;
    sendEigen(sockfd, m);
}

int main(int argc, char *argv[])
{
    int sockfd = initClientSocket();
    vector<shared_ptr<KinectWrapper>> kinects;
    KFRDisplay kfrDisplay(2160, 3840);
    for (int i = 0; i < k4a_device_get_installed_count(); i++)
    {
        kinects.push_back(make_shared<KinectWrapper>(i, kfrDisplay));
    }

    if (sockfd == -1)
    {
        printf("Encountered error starting client");
        return 1;
    }

    while (true)
    {
        char buffer[256];
        bzero(buffer, 256);
        int n = read(sockfd, buffer, sizeof(buffer));
        if (n < 0)
        {
            perror("ERROR reading from socket");
            return 1;
        }
        else if (n == 0)
        {
            perror("Socket closing...");
            return 1;
        }

        printf("Received message: %s\n", buffer);
        switch (buffer[0])
        {
        case 'a':
        {
            //captureAllCorners(sockfd, kinects);
            captureImages(sockfd, kinects, true);
            break;
        }
        case 'b':
        {
            captureImages(sockfd, kinects, false);
            break;
        }
        case 'c':
        {
            sampleEigen(sockfd);
            break;
        }
        case 'd':
        {
            captureAllCorners(sockfd, kinects);
            break;
        }
        }
    }
    close(sockfd);
    return 0;
}