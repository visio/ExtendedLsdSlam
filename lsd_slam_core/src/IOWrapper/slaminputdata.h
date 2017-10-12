#ifndef SLAMINPUTDATA_H
#define SLAMINPUTDATA_H

// This  class containe oridginal and additional input data
// for SLAM algorithm

#include "opencv2/opencv.hpp"

#include "util/SophusUtil.h"

class SlamInputData
{
public:
    // Constructors
    SlamInputData();
    SlamInputData( cv::Mat frame );

    // Set original frame
    void setFrame(cv::Mat *pFrame );

    // Set ground truth
    void setGroundTruth( Sim3* pTransform );

    // Return data validation state
    bool vaild(){ return m_bVaild; }

//private:
//    int     m_iID;              // frame ID
//    double  m_dTimeStamp;       // ROS timestamp

//    // frame resolution
//    int     m_iWidth;
//    int     m_iHeight;

//    // Camera intresetic matrix
//    const Eigen::Matrix3f&  m_mK;

    // Original frame
    cv::Mat m_originalFrame;
    // Understorted grayscale image
    cv::Mat m_grayscaleFrame;

    // Graund truth conversion data
    Sim3    m_graundTruth;

private:
    // Valid flag
    bool    m_bVaild;
};

#endif // SLAMINPUTDATA_H
