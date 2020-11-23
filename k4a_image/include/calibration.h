#include <Eigen/Eigen>

Eigen::MatrixXd computeRTModelToCamera(Eigen::MatrixXd modelPointsH2D, Eigen::MatrixXd camPointsH2D,
    Eigen::MatrixXd intrinsicHomography);
bool captureChessboardCorners(int device, int chessboardRows, int chessboardCols,
    Eigen::MatrixXd *intrinsicsOut, Eigen::MatrixXd *pointsOut);
void printMatrix(Eigen::MatrixXd matrix);

std::vector<Eigen::MatrixXd> calibrateFromFile(std::string file);
