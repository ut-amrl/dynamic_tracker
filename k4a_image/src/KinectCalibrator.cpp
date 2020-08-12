#include "KinectCalibrator.h"
#include "GlobalRTCostFunctor.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vision_geometry/HomographyShortcuts.h>
#include <vision_geometry/RigidTrans.h>
#include <vision_geometry/TransformShortcuts.h>
#include <vision_geometry/Util.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace ceres;

KinectCalibrator::KinectCalibrator(std::string path, CameraIntrinsics k)
    : intrinsics(k)
    , chessboard(4, 6, 23.0)
{
    imgSets = listFiles(path);
    sort(imgSets.begin(), imgSets.end());
}

KinectCalibrator::~KinectCalibrator()
{
}

void KinectCalibrator::select()
{
    cout << "================Usage================" << endl;
    cout << "Red indicates images that will not be included in calibration, green is opposite" << endl;
    cout << "Space:      Toggle selection" << endl;
    cout << "Left/Right: Move through images" << endl;
    cout << "n:          Go to next set" << endl;
    cout << "Esc:        Exit" << endl;

    const int COLS = 5;
    Size resizeTo(256, 256);
    for (auto set : imgSets) {
        // Create grid for set of images
        vector<string> imgList = listFiles(set + "/");

        int selection = 0;
        int key = -1;
        Mat grid;
        do {
            if (key == 83) {
                // right
                selection++;
                selection %= imgList.size();
            } else if (key == 81) {
                // left
                selection--;
                if (selection < 0) {
                    selection = imgList.size() - 1;
                }
            } else if (key == 27) {
                return;
            }
            int imgNum = 0;
            for (int i = 0; i < imgList.size() / COLS + 1; i++) {
                vector<Mat> row;
                for (int j = 0; j < 5; j++) {
                    if (imgNum >= imgList.size()) {
                        row.push_back(Mat::zeros(resizeTo, 16));
                    } else {
                        Mat resized;
                        string imgPath = imgList[imgNum];
                        resize(imread(imgPath), resized, resizeTo);

                        // render selector
                        if (selection == imgNum) {
                            rectangle(resized, Point(2, 2), Point(resizeTo.width - 2, resizeTo.height - 2), Scalar(0, 0, 0), 5);

                            if (key == 32) {
                                // space
                                if (imgPath[imgPath.size() - 5] == 'y') {
                                    imgList[imgNum][imgPath.size() - 5] = 'n';
                                } else {
                                    imgList[imgNum][imgPath.size() - 5] = 'y';
                                }
                                // Update file name
                                rename(imgPath.c_str(), imgList[imgNum].c_str());
                            }
                        }

                        // render red for no, green for yes
                        if (imgPath[imgPath.size() - 5] == 'y') {
                            rectangle(resized, Point(1, 1), Point(resizeTo.width - 1, resizeTo.height - 1), Scalar(0, 255, 0));
                        } else {
                            rectangle(resized, Point(1, 1), Point(resizeTo.width - 1, resizeTo.height - 1), Scalar(0, 0, 255));
                        }
                        row.push_back(resized);
                    }
                    imgNum++;
                }
                Mat rowImg;
                hconcat(row, rowImg);
                if (i == 0) {
                    grid = rowImg;
                } else {
                    vconcat(grid, rowImg, grid);
                }
            }

            imshow(set, grid);
        } while ((key = waitKey(0)) != 'n');
        destroyAllWindows();
    }
}

void KinectCalibrator::solve()
{
    cout << "Num sets: " << imgSets.size() << endl;
    int numCams = 4;
    int numBoards = imgSets.size();
    // Goal- optimize n-1 cameras' pose w.r.t cam 0
    //       (and also m chessboards' pose)
    vector<MatrixXd> camRTs;
    vector<RigidTrans> boards;
    for (int i = 0; i < numCams; i++) {
        MatrixXd m(3, 4);
        m << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0;
        camRTs.push_back(m);
    }
    for (int i = 0; i < numBoards; i++) {
        Quaterniond rot(1, 0, 0, 0);
        Vector3d trans(0, 0, 0);
        RigidTrans rt(rotVect(rot.toRotationMatrix()), trans);
        boards.push_back(rt);
    }
}

void KinectCalibrator::globalOpt(vector<Camera> cams, vector<MatrixXd> camRTs, vector<MatrixXd> objRTs)
{
    if (cams.size() != camRTs.size()) {
    }
    Problem problem;
    // Iterate through cameras and all the image points they each see
    for (Camera cam : cams) {
        for (auto x : cam.projections) {
            MatrixXd imgPts = x.second;
            MatrixXd worldPts = chessboard.getModelCBH2D();

            if (imgPts.cols() != worldPts.cols()) {
                cout << "Error: mismatch in mapping from world to img points" << endl;
            }
            for (int i = 0; i < imgPts.cols(); i++) {

                CostFunction* cost = new AutoDiffCostFunction<GlobalRTCostFunctor, 2, 7, 7>(new GlobalRTCostFunctor(intrinsics.getMat(), imgPts.col(i), worldPts.col(i)));
                // Param 1: camN to cam0 Param2: objN to cam0
                problem.AddResidualBlock(cost, NULL, camRTs.at(cam.index).data(), objRTs.at(x.first).data());

                // // We are argmining w.r.t an SE(3) element
                // ProductParameterization* se3_param = new ProductParameterization(
                //     new EigenQuaternionParameterization(), new IdentityParameterization(3));
                // problem.SetParameterization(se3data.data(), se3_param);
            }
        }
    }
    vector<double*> params;
    problem.GetParameterBlocks(&params);
    // Set local parameterizations
    for (double* param : params) {
        ProductParameterization* se3_param = new ProductParameterization(
            new EigenQuaternionParameterization(), new IdentityParameterization(3));
        problem.SetParameterization(param, se3_param);
    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;
}

