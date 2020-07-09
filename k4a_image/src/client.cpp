#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#include <Eigen/Dense>
#include <iostream>
#include <config.h>
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

#include <memory>
#include <sockets.h>

using namespace cv;
using namespace Eigen;
using namespace std;

// Capture images from all cameras and send corners (if found) back to server
void captureAllCorners(int sockfd, vector<shared_ptr<KinectWrapper>> &kinects)
{
    for (shared_ptr<KinectWrapper> kinect : kinects)
    {
        cv::Size s(CBOARD_ROWS, CBOARD_COLS);
        Mat image = kinect->capture();
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

        int len = camPoints.rows() * camPoints.cols();
        double *result = (double *)malloc(sizeof(double) * len);
        result = camPoints.data();
        // Send size of array first
        if (write(sockfd, &len, sizeof(len)) < 0)
        {
            perror("ERROR writing to socket");
        }
        cout << "sent size" << endl;
        // Send array
        if (write(sockfd, result, sizeof(double) * len) < 0)
        {
            perror("ERROR writing to socket");
        }
        cout << "sent data" << endl;
    }
}

void image(int sockfd)
{
    Mat image;
    image = imread("/home/henry/Desktop/1593827043.jpg", CV_LOAD_IMAGE_COLOR);
    // imshow("image", image);
    // waitKey(0);

    vector<uchar> buf;
    imencode(".jpg", image, buf);
    Mat dst = imdecode(buf, 1);
    // imshow("decoded image", dst);
    // waitKey(0);
    sendCV(sockfd, image);
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
    for (int i = 0; i < k4a_device_get_installed_count(); i++)
    {
        kinects.push_back(make_shared<KinectWrapper>(i));
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
            //captureAllCorners(sockfd, kinects);
            image(sockfd);
            break;
        case 'b':
            sampleEigen(sockfd);
            printf("Secondary\n");
            break;
        }

        // switch: do something
    }
    close(sockfd);
    return 0;
}