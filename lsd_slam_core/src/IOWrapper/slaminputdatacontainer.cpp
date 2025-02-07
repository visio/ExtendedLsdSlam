#include "slaminputdatacontainer.h"

//#include <Eigen/Geometry>
//#include "sophus/sim3.hpp"
//#include "sophus/se3.hpp"

using namespace lsd_slam;

SlamInputDataContainer::SlamInputDataContainer(std::string source, Undistorter *pUndistorter)  :
    m_pUndistorter  ( pUndistorter  )
{
    // Try to open dource directory
    if( this->getdir(source) >= 0 )
    {
        printf("found %d image files in folder %s!\n", (int)m_vFramesPath.size(), source.c_str());
    }
    else if( this->getFile(source) >= 0)
    {
        printf("found %d image files in file %s!\n", (int)m_vFramesPath.size(), source.c_str());
    }
    else
    {
        printf("could not load file list! wrong path / file?\n");
    }
}

void SlamInputDataContainer::setUndistorter(Undistorter *pUndistorter)
{
    // Set undistorter pointer
    m_pUndistorter = pUndistorter;
}

int SlamInputDataContainer::setBlenderData(std::string source)
{
    // Make new file stream
    std::ifstream f(source.c_str());

    // if it was file and it has open well
    if(f.good() && f.is_open())
    {
        std::string sLine;              // Line from sile
//        std::string sTemp;            // String for temp digits

        std::string::size_type sz;      // alias of size_t

//        Sim3        tform;
        double tempBuff[4];

        // Debug info
//        std::cout << "SlamInputDataContainer::setBlenderData:Start read lines: " << std::endl;

        // till and off file
        while( !f.eof() )
        {
            // Get next line
            std::getline( f, sLine );
            // Debug info
//            std::cout << std::endl << sLine << std::endl;

            // trim string both side
            sLine = trim( sLine );
            // if line empty or commented
            if(sLine == "" || sLine[0] == '#')
                continue;

            Sim3::Point _translation;
            // Initialize transformation
            _translation[0] = std::stod ( sLine, &sz );
            sLine = sLine.substr(sz);
            _translation[1] = std::stod ( sLine, &sz);
            sLine = sLine.substr(sz);
            _translation[2] = std::stod ( sLine, &sz);
            sLine = sLine.substr(sz);

            // Read Quaternion
            tempBuff[0] = std::stod ( sLine, &sz );
            sLine = sLine.substr(sz);
            tempBuff[1] = std::stod ( sLine, &sz );
            sLine = sLine.substr(sz);
            tempBuff[2] = std::stod ( sLine, &sz );
            sLine = sLine.substr(sz);
            tempBuff[3] = std::stod ( sLine, &sz );
            sLine = sLine.substr(sz);

//            Eigen::Quaternion<Sim3::Scalar>   _rotation( tempBuff );
            Eigen::Quaternion<Sim3::Scalar>   _rotation( tempBuff[0],
                                                         tempBuff[1],
                                                         tempBuff[2],
                                                         tempBuff[3] );

            // Set in Sim3
            Sim3 tform( _rotation, _translation );

//            std::cout << std::endl << "Translation:" << std::endl;
//            std::cout << tform.translation()[0] << std::endl;
//            std::cout << tform.translation()[1] << std::endl;
//            std::cout << tform.translation()[2] << std::endl;

//            std::cout << std::endl << "Quaternion:" << std::endl;
//            std::cout << tform.quaternion().w() << std::endl;
//            std::cout << tform.quaternion().x() << std::endl;
//            std::cout << tform.quaternion().y() << std::endl;
//            std::cout << tform.quaternion().z() << std::endl;

            // Append path to path list
            m_vGroundTruth.push_back( tform );
        }

        // Close file
        f.close();

        // Return number of frames
        return (int)m_vGroundTruth.size();
    }
    else
    {
        // Debug info
        std::cout << "SlamInputDataContainer::setBlenderData: Can not open file: "<< source << std::endl;

        f.close();
        return -1;
    }

}

SlamInputData SlamInputDataContainer::getData(int idx)
{
    // Create data object
    SlamInputData data;

    // Read next image
    cv::Mat frame = cv::imread( m_vFramesPath[ idx ],
                                CV_LOAD_IMAGE_GRAYSCALE);

    if( m_pUndistorter )
    {
        int w = m_pUndistorter->getInputWidth();
        int h = m_pUndistorter->getInputHeight();

        // Check image size
        if( frame.rows != h || frame.cols != w )
        {
            if( frame.rows * frame.cols == 0 )
                printf( "failed to load image %s! skipping.\n",
                        m_vFramesPath[ idx ].c_str() );
            else
                printf( "image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                        m_vFramesPath[ idx ].c_str(),
                        w, h, frame.cols, frame.rows );

            // Debug info
            std::cout << "SlamInputDataContainer::getData: Invalid frame !!! " << std::endl;
            // return
            return data;
        }
    }

    // Set frame
    data.setFrame( &frame );

    // Append ground truth
    if( idx < m_vGroundTruth.size() )
    {
        // Debug info
        std::cout << "SlamInputDataContainer::getData: Append ground truth..." << std::endl;
        // Append information to input data
        data.setGroundTruth( &m_vGroundTruth[idx] );
    }

    // return data
    return data;
}

std::string SlamInputDataContainer::getFramePath(int idx)
{
    // Get path
    return m_vFramesPath[ idx ];
}

std::string &SlamInputDataContainer::ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

std::string &SlamInputDataContainer::rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

std::string &SlamInputDataContainer::trim (std::string &s) {
        return ltrim(rtrim(s));
}

// Make list of file name from directory
int SlamInputDataContainer::getdir (std::string dir)
{
    DIR *dp;
    struct dirent *dirp;

    // Try to open dir
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    // For all input files
    while ((dirp = readdir(dp)) != NULL)
    {
        // Read next name
        std::string name = std::string(dirp->d_name);

        // If it is not spacial symbols
        if(name != "." && name != "..")
            // Append it to list
            m_vFramesPath.push_back(name);
    }
    // Close directory
    closedir(dp);

    // Sort friles by name
    std::sort( m_vFramesPath.begin(), m_vFramesPath.end() );

    // Some changes in file path ???
    if( dir.at( dir.length() - 1 ) != '/' )
        dir = dir+"/";

    for( unsigned int i = 0; i< m_vFramesPath.size(); i++ )
    {
        if( m_vFramesPath[i].at(0) != '/' )
            m_vFramesPath[i] = dir + m_vFramesPath[i];
    }

    // Return number of frames
    return m_vFramesPath.size();
}

// Make list of file name from file
int SlamInputDataContainer::getFile (std::string source)
{
    // Make new file stream
    std::ifstream f(source.c_str());

    // if it was file and it has open well
    if(f.good() && f.is_open())
    {
        // till and off file
        while(!f.eof())
        {
            std::string l;
            // Get next line
            std::getline(f,l);

            // trim string both side
            l = trim(l);

            // if line empty or commented
            if(l == "" || l[0] == '#')
                continue;

            // Append path to path list
            m_vFramesPath.push_back(l);
        }

        // Close file
        f.close();

        size_t sp = source.find_last_of('/');
        std::string prefix;
        if(sp == std::string::npos)
            prefix = "";
        else
            prefix = source.substr(0,sp);

        for(unsigned int i = 0; i < m_vFramesPath.size(); i++ )
        {
            if(m_vFramesPath[i].at(0) != '/')
                m_vFramesPath[i] = prefix + "/" + m_vFramesPath[i];
        }

        // Return number of frames
        return (int)m_vFramesPath.size();
    }
    else
    {
        f.close();
        return -1;
    }
}
