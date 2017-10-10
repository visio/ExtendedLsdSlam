#ifndef SLAMINPUTDATA_H
#define SLAMINPUTDATA_H

// This  class containe oridginal and additional input data
// for SLAM algorithm

#include "opencv2/opencv.hpp"

class SlamInputData
{
public:
    SlamInputData( cv::Mat frame );

//private:
    // Original frame
    cv::Mat m_originalFrame;
    // Understorted grayscale image
    cv::Mat m_grayscaleFrame;
};

#endif // SLAMINPUTDATA_H
