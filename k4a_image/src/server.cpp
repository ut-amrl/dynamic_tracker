#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include <Eigen/Dense>

#define PORT 12345
#define NUM_CLIENTS 1

using namespace std;
using namespace Eigen;

vector<int> clients;

int initSocket()
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
    int map[NUM_CLIENTS];
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

            map[newsockfd] = connectedClients;
            clients.push_back(newsockfd);
            cout << "Client #" << connectedClients << " connected with " << cameras << " kinects" << endl;
            connectedClients++;
        }
    }
    close(sockfd);
    return 0;
}

vector<MatrixXd> captureAllCorners()
{
    vector<MatrixXd> result;
    for (int sockfd : clients)
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
    }
    return result;
}

int main(int argc, char *argv[])
{

    initSocket();
    while (true)
    {
        char buffer[256];
        bzero(buffer, 256);
        printf("Please enter the message: \n");
        fgets(buffer, 256, stdin);

        // Message clients
        for (int sockfd : clients)
        {
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
            vector<MatrixXd> v = captureAllCorners();
            cout << v[0] << endl;
        }
        break;
        case 'b':
        {
        }
        break;
        }
    }

    return 0;
}
