#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#include <iostream>
#include <config.h>
#include <sockets.h>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace cv;

// Server
int initServerSocket(vector<client> &clients)
{
    // Create the server socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
        return -1;
    }
    // Option to reuse address right away
    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    // Define the server address
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);

    // Bind the socket to our specified port
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("ERROR on binding");
        return -1;
    }

    listen(sockfd, 5);

    int connectedClients = 0;
    cout << "Awaiting " << NUM_CLIENTS << " clients..." << endl;
    while (connectedClients < NUM_CLIENTS)
    {
        // Accept new connection
        int newsockfd = accept(sockfd, nullptr, nullptr);
        if (newsockfd < 0)
        {
            perror("ERROR on accept");
        }
        else
        {
            // Confirm client # to client
            char buffer[256];
            sprintf(buffer, "%d", connectedClients);
            if (write(newsockfd, buffer, sizeof(buffer)) < 0)
            {
                perror("ERROR writing to socket");
            }

            // Get number of cameras connected to client
            int cameras = 0;
            if (read(newsockfd, &cameras, sizeof(cameras)) < 0)
            {
                perror("ERROR reading from socket");
            }
            else
            {
                cameras = ntohl(cameras);
            }

            client newClient;
            newClient.fd = newsockfd;
            newClient.cameras = cameras;

            clients.push_back(newClient);
            cout << "Client #" << connectedClients << " connected with " << cameras << " kinects" << endl;
            connectedClients++;
        }
    }
    close(sockfd);
    return 0;
}

// Client
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

// Receive Eigen matrix sent by sendEigen
int readEigen(int sockfd, Eigen::MatrixXd &out)
{
    // Get size of byte array
    int size = 0;
    int n = read(sockfd, &size, sizeof(size));
    if (n < 0)
    {
        perror("ERROR reading from socket (readEigen size)");
        return -1;
    }

    // Receive array and convert back to 2xN eigen matrix
    double *recvArray = (double *)malloc(sizeof(double) * size);
    n = read(sockfd, recvArray, sizeof(double) * size);
    if (n < 0)
    {
        perror("ERROR reading from socket (readEigen data)");
        return -1;
    }
    out = Eigen::Map<Eigen::MatrixXd>(recvArray, 2, size / 2);
    free(recvArray);
}

// Send Eigen matrix to be received by readEigen
int sendEigen(int sockfd, Eigen::MatrixXd m)
{
    int len = m.rows() * m.cols();
    double *result = (double *)malloc(sizeof(double) * len);
    result = m.data();
    // Send size of array first
    if (write(sockfd, &len, sizeof(len)) < 0)
    {
        perror("ERROR writing to socket");
        return -1;
    }
    //cout << "sent size" << endl;
    // Send array
    if (write(sockfd, result, sizeof(double) * len) < 0)
    {
        perror("ERROR writing to socket");
        return -1;
    }
    //cout << "sent data" << endl;
}

int readCV(int sockfd, Mat &image)
{
    // Get size of byte array
    int size = 0;
    int n = read(sockfd, &size, sizeof(size));
    if (n < 0)
    {
        perror("ERROR reading from socket (readCV size)");
        return -1;
    }
    cout << "Waiting for " << size << " bytes" << endl;

    vector<uchar> buf;
    int bytesReceived = 0;
    // TODO figure out byte order problems and can increase this
    char recv_buf[1];
    while (bytesReceived < size)
    {
        int n = read(sockfd, recv_buf, sizeof(recv_buf));
        bytesReceived += n;
        if (n > 0)
        {
            for (unsigned int i = 0; i < sizeof(recv_buf); i++)
                buf.push_back(recv_buf[i]);
            // cout << "Received " << bytesReceived << " bytes..." << endl;
        }
        else if (n == 0)
        {
            if (bytesReceived < size)
            {
                printf("premature close, expected %u, only got %lu\n", size, buf.size());
                return -1;
            }
            else
            {
                cout << "got it" << endl;
                break;
            }
        }
        else
        {
            perror("recv");
        }
    }
    cout << "Received all " << bytesReceived << " bytes" << endl;
    image = cv::imdecode(buf, CV_LOAD_IMAGE_UNCHANGED);
}

int sendCV(int sockfd, Mat src)
{
    vector<uchar> buf;
    imencode(".jpg", src, buf);
    // Send size first
    int len = buf.size();
    if (write(sockfd, &len, sizeof(len)) < 0)
    {
        perror("ERROR writing to socket");
        return -1;
    }
    cout << "sent size" << endl;

    if (write(sockfd, buf.data(), buf.size()) < 0)
    {
        perror("ERROR writing to socket");
        return -1;
    }
    cout << "sent data" << endl;
}