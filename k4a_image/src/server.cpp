#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <sockets.h>

#include <sys/stat.h>

using namespace std;
using namespace Eigen;
using namespace cv;

vector<client> clients;

Mat scale(Mat in, double scaleFactor)
{
    Mat scaled;
    Size size(in.cols / scaleFactor, in.rows / scaleFactor);
    resize(in, scaled, size);
    return scaled;
}

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

void getCaptureImages(bool ask)
{
    cout << (ask ? "Reading images from clients WITH approval" : "Reading images from clients WITHOUT approval") << endl;
    // Create directory for images
    int set = std::time(nullptr);
    std::stringstream dir;
    dir << "../captures/"
        << "set_" << set << "/";
    mkdir(dir.str().c_str(), 0777);

    for (int clientNum = 0; clientNum < clients.size(); clientNum++)
    {
        client client = clients[clientNum];
        //Iterate over cameras
        for (int i = 0; i < client.cameras; i++)
        {
            char approval = 'y';
            Mat image;
            readCV(client.fd, image);
            std::stringstream name;
            name << "client_" << clientNum << "_cam_" << i;
            if (ask)
            {
                Mat small = scale(image, 4);
                imshow(name.str(), small);
                char c = waitKey(0);
                destroyAllWindows();
                if (c == 'f')
                {
                    approval = 'n';
                }
            }
            name << "_" << approval << ".jpg ";
            cout << "capturing " << name.str() << endl;
            imwrite(dir.str() + name.str(), image);
        }
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
        cout << "---------------------------" << endl;
        cout << "Reading from the keyboard" << endl;
        cout << "---------------------------" << endl;
        cout << "a: capture images from all cameras with marked corners" << endl;
        cout << "b: capture images from all cameras without marked corners" << endl;
        cout << "c: try sample eigen data" << endl;
        cout << "d: none" << endl;
        cout << "CTRL-C to quit" << endl;
        fgets(buffer, 256, stdin);
        cout << "---------------------------" << endl;
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
            getCaptureImages(false);
            break;
        }
        case 'b':
        {
            getCaptureImages(false);
            break;
        }
        case 'c':
        {
            vector<MatrixXd> v = getSampleEigen();
            cout << "Received:\n"
                 << v[0] << endl;
            break;
        }
        case 'd':
        {
            vector<MatrixXd> v = captureAllCorners();
            if (v.size() > 0)
                cout << v[0] << endl;
            break;
        }
        default:
        {
            cout << "Invalid input" << endl;
            break;
        }
        }
    }

    return 0;
}
