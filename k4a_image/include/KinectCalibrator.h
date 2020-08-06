#ifndef KFR_CALIB_H
#define KFR_CALIB_H
#include <string>
#include <vector>


class KinectCalibrator
{
public:
    KinectCalibrator(std::string path);
    ~KinectCalibrator();
    void select();
protected:
    std::vector<std::string> imgSets;
};

#endif