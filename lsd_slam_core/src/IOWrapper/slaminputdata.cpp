#include "slaminputdata.h"

SlamInputData::SlamInputData(cv::Mat frame) :
    m_originalFrame( frame )
{
    // Initialize output image
    m_grayscaleFrame = cv::Mat( frame.rows, frame.cols, CV_8U );
}
