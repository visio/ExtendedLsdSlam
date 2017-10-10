#ifndef SLAMGROUNDTRUTH_H
#define SLAMGROUNDTRUTH_H

#include "opencv2/opencv.hpp"

// This class contane ground truth data from
// deifferen sources for use it in SLAM fusion

class SlamGroundTruth
{
public:
    SlamGroundTruth();

private:
    // Frame orignal ( Color version )
    cv::Mat originalFrame;
    // Frame graysacle version

    // Graund truth data
};

#endif // SLAMGROUNDTRUTH_H
