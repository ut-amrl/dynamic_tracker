#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#include <Eigen/Dense>
#include <iostream>

#define IP "192.168.1.242"
#define PORT 12345

using namespace Eigen;
using namespace std;

// Capture images from all cameras and send corners (if found) back to server
void captureAllCorners(int sockfd)
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
    // Create the server socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("ERROR opening socket");
        return 1;
    }
    // Define the server address
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(IP);
    serv_addr.sin_port = htons(PORT);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("ERROR connecting");
        return 1;
    }
    // Server will send what connetion number this is
    char buffer[256];
    bzero(buffer, 256);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return 1;
    }
    // Tell server how many cameras
    int devices = htonl(3); // k4a_device_get_installed_count();
    if (write(sockfd, &devices, sizeof(devices)) < 0)
    {
        perror("ERROR writing to socket");
        return 1;
    }
    printf("Successfully connected as client number: %s\n", buffer);

    while (true)
    {
        bzero(buffer, 256);
        int n = read(sockfd, buffer, sizeof(buffer));
        if (n <= 0)
        {
            perror("ERROR reading from socket");
            return 1;
        }
        printf("Received message: %s\n", buffer);
        switch (buffer[0])
        {
        case 'a':
            captureAllCorners(sockfd);
            break;
        case 'b':
            printf("Secondary\n");
            break;
        }

        // switch: do something
    }
    close(sockfd);
    return 0;
}