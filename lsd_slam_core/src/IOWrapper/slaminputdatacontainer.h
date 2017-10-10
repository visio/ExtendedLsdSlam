#ifndef SLAMINPUTDATACONTAINER_H
#define SLAMINPUTDATACONTAINER_H

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include "IOWrapper/slaminputdata.h"

#include "util/SophusUtil.h"
#include "util/Undistorter.h"

using namespace lsd_slam;

class SlamInputDataContainer
{
public:
    // Constructor
    SlamInputDataContainer( std::string source );

    // Return frames number
    int framesCount(){ return (int)m_vFramesPath.size(); }

    // Set undistorter
    void setUndistorter( Undistorter*    undistorter );

    // Set ground truth data from Blender
    int setBlenderData( std::string source );

    // Get next frame data
    SlamInputData getData( int idx );

    // Get Frame file name
    std::string getFramePath( int idx );

private:
    std::string &ltrim(std::string &s);
    std::string &rtrim(std::string &s);
    std::string &trim (std::string &s);

    // Make list of file name from directory
    int getdir(std::string dir);
    // Make list of file name from file
    int getFile(std::string source);

private:
    // List of frame file name
    std::vector<std::string>    m_vFramesPath;
    // List of ground truth convertions
    std::vector<Sim3>           m_vGroundTruth;

    Undistorter*                m_pUndistorter;
};

#endif // SLAMINPUTDATACONTAINER_H
