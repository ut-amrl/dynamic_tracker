#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

struct client
{
    int fd;
    int cameras;
};

int initServerSocket(std::vector<client> &clients);
int initClientSocket();
int readEigen(int sockfd, Eigen::MatrixXd &out);
int sendEigen(int sockfd, Eigen::MatrixXd m);
int readCV(int sockfd, cv::Mat &image);
int sendCV(int sockfd, cv::Mat image);