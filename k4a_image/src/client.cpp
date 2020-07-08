#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#define IP "IP_OF_SERVER"
#define PORT 12345

int main(int argc, char *argv[])
{
    char buffer[256];
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

    bzero(buffer, 256);
    int n = read(sockfd, buffer, sizeof(buffer));
    if (n < 0)
    {
        perror("ERROR reading from socket");
        return 1;
    }
    printf("Successfully connected as client number: %s\n", buffer);

    while (true)
    {
        bzero(buffer, 256);
        n = read(sockfd, buffer, sizeof(buffer));
        if (n <= 0)
        {
            perror("ERROR reading from socket");
            return 1;
        }
        printf("Received message: %s\n", buffer);
        // switch: do something
    }
    close(sockfd);
    return 0;
}