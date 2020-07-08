#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#define PORT 12345
#define NUM_CLIENTS 3

using namespace std;

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
            char buffer[256];
            sprintf(buffer, "%d", connectedClients);
            if (write(newsockfd, buffer, sizeof(buffer)) < 0)
            {
                perror("ERROR writing to socket");
                return 1;
            }
            map[newsockfd] = connectedClients;
            clients.push_back(newsockfd);
            cout << "Client #" << connectedClients << " connected" << endl;
            connectedClients++;
        }
    }
    close(sockfd);
    return 0;
}

int main(int argc, char *argv[])
{

    initSocket();
    while (true)
    {
        char buffer[256];
        bzero(buffer, 256);
        printf("Please enter the message: ");
        fgets(buffer, 256, stdin);

        for (int sockfd : clients)
        {
            int n = send(sockfd, buffer, sizeof(buffer), MSG_NOSIGNAL);
            if (n < 0)
            {
                perror("ERROR writing to socket");
            }
        }
    }

    return 0;
}
