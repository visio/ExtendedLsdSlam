#ifndef SLAMINPUTDATA_H
#define SLAMINPUTDATA_H

// This  class containe oridginal and additional input data
// for SLAM algorithm

#include "opencv2/opencv.hpp"

class SlamInputData
{
public:
    // Constructors
    SlamInputData();
    SlamInputData( cv::Mat frame );

    // Set original frame
    void setFrame(cv::Mat *pFrame );

    // Return data validation state
    bool    vaild(){ return m_bVaild; }

//private:
    // Original frame
    cv::Mat m_originalFrame;
    // Understorted grayscale image
    cv::Mat m_grayscaleFrame;

private:
    // Valid flag
    bool    m_bVaild;

};

#endif // SLAMINPUTDATA_H
