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

using namespace cv;
using namespace Eigen;
using namespace std;

int initClientSocket()
{
    // Create the client socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
        return -1;
    }
    // Define the server address
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(IP);
    serv_addr.sin_port = htons(PORT);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("ERROR connecting");
        return -1;
    }
    // Server will send what connetion number this is
    char buffer[256];
    bzero(buffer, 256);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return -1;
    }
    // Tell server how many cameras
    int devices = htonl(k4a_device_get_installed_count());
    if (write(sockfd, &devices, sizeof(devices)) < 0)
    {
        perror("ERROR writing to socket");
        return -1;
    }
    printf("Successfully connected as client number: %s\n", buffer);
    return sockfd;
}

// Capture images from all cameras and send corners (if found) back to server
void captureAllCorners(int sockfd, vector<KinectWrapper> kinects)
{
    for (KinectWrapper kinect : kinects)
    {
        cv::Size s(CBOARD_ROWS, CBOARD_COLS);
        Mat image = kinect.capture();

        Mat detections;
        MatrixXd camPoints;
        if (!detectFrameCorners(s, image, detections, camPoints))
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

// Capture images from all cameras and send corners (if found) back to server
void sampleEigen(int sockfd)
{
    MatrixXd m(2, 3);
    m << 1.234, 2.432, 3.4574456,
        4, 5, 6.654;
    cout << m << endl;

    int len = m.rows() * m.cols();
    double *result = (double *)malloc(sizeof(double) * len);
    result = m.data();
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

int main(int argc, char *argv[])
{
    int sockfd = initClientSocket();
    vector<KinectWrapper> kinects;
    for (int i = 0; i < k4a_device_get_installed_count(); i++)
    {
        KinectWrapper kinect(i);
        kinects.push_back(kinect);
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
            captureAllCorners(sockfd, kinects);
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