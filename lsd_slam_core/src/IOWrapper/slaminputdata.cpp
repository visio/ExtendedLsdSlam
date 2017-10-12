#include "slaminputdata.h"

SlamInputData::SlamInputData() :
    m_bVaild        ( false )
{
    // Initialize variables
//    m_grayscaleFrame = cv::Mat( frame.rows, frame.cols, CV_8U );

}

SlamInputData::SlamInputData(cv::Mat frame) :
    m_bVaild        ( false ),
    m_originalFrame ( frame )
{
    // Set original frame
    setFrame( &frame );
}

void SlamInputData::setFrame(cv::Mat* pFrame)
{
    // Save original frame
    m_originalFrame = *pFrame;

    // Check chennels number
    if ( m_originalFrame.channels() == 1 )
        // Just save frame
        m_grayscaleFrame = m_originalFrame;
    else
        // Convert to grayscale
        cvtColor( m_originalFrame,
                  m_grayscaleFrame,
                  CV_RGB2GRAY           );

    // Set validate flag
    m_bVaild = true;
}

void SlamInputData::setGroundTruth(Sim3 *pTransform)
{
    // Debug info
    std::cout << "SlamInputData::setGroundTruth: Started..." << std::endl;

    // Save transform for graud truth
    m_graundTruth = *pTransform;
}
