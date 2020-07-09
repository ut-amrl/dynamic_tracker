#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <sockets.h>

using namespace std;
using namespace Eigen;
using namespace cv;

vector<client> clients;

vector<MatrixXd> captureAllCorners()
{
    vector<MatrixXd> result;
    for (client client : clients)
    {
        int sockfd = client.fd;
        // Iterate over cameras
        for (int i = 0; i < client.cameras; i++)
        {
            // Get size of byte array
            int size = 0;
            int n = read(sockfd, &size, sizeof(size));
            if (n < 0)
            {
                perror("ERROR reading from socket (captureImages size)");
            }

            // Receive array and convert back to 2xN eigen matrix
            double *recvArray = (double *)malloc(sizeof(double) * size);
            n = read(sockfd, recvArray, sizeof(double) * size);
            if (n < 0)
            {
                perror("ERROR reading from socket (captureImages data)");
            }
            MatrixXd corners = Map<MatrixXd>(recvArray, 2, size / 2);
            result.push_back(corners);
            free(recvArray);
        }
    }
    return result;
}

void image()
{
    for (client client : clients)
    {
        int sockfd = client.fd;
        // Iterate over cameras
        Mat image;
        cout << "reading... " << endl;
        readCV(sockfd, image);
        imshow("received image", image);
        waitKey(0);
    }
}

vector<MatrixXd> getSampleEigen()
{
    vector<MatrixXd> result;
    for (client client : clients)
    {
        MatrixXd corners;
        readEigen(client.fd, corners);
        result.push_back(corners);
    }
    return result;
}

int main(int argc, char *argv[])
{

    // Start server
    if (initServerSocket(clients) < 0)
    {
        cout << "Error starting server" << endl;
        return 1;
    }

    while (true)
    {
        char buffer[256];
        bzero(buffer, 256);
        printf("Please enter the message: \n");
        fgets(buffer, 256, stdin);

        // Message clients
        for (client client : clients)
        {
            int sockfd = client.fd;
            int n = send(sockfd, buffer, sizeof(buffer), MSG_NOSIGNAL);
            if (n < 0)
            {
                perror("ERROR writing to socket");
            }
        }

        switch (buffer[0])
        {
        case 'a':
        {
            // vector<MatrixXd> v = captureAllCorners();
            // if (v.size() > 0)
            //     cout << v[0] << endl;
            image();
        }
        break;
        case 'b':
        {
            vector<MatrixXd> v = getSampleEigen();
            cout << "Received:\n"
                 << v[0] << endl;
        }
        break;
        }
    }

    return 0;
}
