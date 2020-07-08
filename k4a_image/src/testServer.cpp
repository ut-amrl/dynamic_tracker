#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#include <unistd.h>

int main(int argc, char *argv[])
{
    char serverMessage[256] = "You have reached the server";

    // Create the server socket
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    // Define the server address
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to our specified IP and port
    bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

    listen(serverSocket, 5);

    int clientSocket;
    clientSocket = accept(serverSocket, NULL, NULL);

    // Send the message
    send(clientSocket, serverMessage, sizeof(serverMessage), 0);

    // Close the socket
    close(serverSocket);
    return 0;
}