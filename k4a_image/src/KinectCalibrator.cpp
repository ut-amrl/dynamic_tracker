#include "KinectCalibrator.h"
#include <opencv2/opencv.hpp>
#include <vision_geometry/Util.h>
#include <stdio.h>

using namespace std;
using namespace cv;

KinectCalibrator::KinectCalibrator(std::string path)
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
            } else if (key == 27){
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