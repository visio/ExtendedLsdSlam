/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

//#include "IOWrapper/slamgroundtruth.h"
//#include "IOWrapper/slaminputdata.h"
#include "IOWrapper/slaminputdatacontainer.h"

#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/Undistorter.h"
#include <ros/package.h>

#include "opencv2/opencv.hpp"

using namespace lsd_slam;
int main( int argc, char** argv )
{
    // init slam
	ros::init(argc, argv, "LSD_SLAM");

    // ********* Dinamic Reconfigure initializing
	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

    // Get current package path
	packagePath = ros::package::getPath("lsd_slam_core")+"/";

    // ************ READING CALIBRATION FILE ********************************************
	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
    std::string     calibFile;
    Undistorter*    undistorter = 0;
    if( ros::param::get("~calib", calibFile) )
	{
		 undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
		 ros::param::del("~calib");
	}

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using _calib:=FILE)\n");
		exit(0);
	}

	int w = undistorter->getOutputWidth();
	int h = undistorter->getOutputHeight();

	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);

	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

    // ************************* Open IMAGE Files ***********************************
    // Open image files: first try to open as file.
    std::string frameSource;
    if(!ros::param::get("~files", frameSource))
    {
        printf("need source files! (set using _files:=FOLDER)\n");
        exit(0);
    }
    // Delete parameter form list
    ros::param::del("~files");

    // Create container
    SlamInputDataContainer inputData( frameSource );

    // If understorter is present
    if( undistorter != 0 )
    {
        // set undestorter to container
        SlamInputDataContainer.setUndistorter( undistorter );
    }

    // ************************* My FLAGS ******************************************
    // open image files: first try to open as file.
    bool saveRes = false;
    // read parameters
    ros::param::get("~saveResults", saveRes);

    // Gtaund truth source file name
    std::string         sGTFileName;

    // Read parameters
    ros::param::get("~groundTruth", sGTFileName);

    // Build list of graund-truth data

    // ******************************************************************************
	// make output wrapper. just set to zero if no output is required.
    Output3DWrapper* outputWrapper = new ROSOutput3DWrapper( w, h, frameSource, saveRes );

	// make slam system
    SlamSystem* system = new SlamSystem( w, h, K, doSlam );
    // Set output wrapper
	system->setVisualization(outputWrapper);

	// get HZ
	double hz = 0;
    if( !ros::param::get("~hz", hz) )
		hz = 0;

	ros::param::del("~hz");

    // Make matrix
    cv::Mat image           = cv::Mat( h, w, CV_8U );
    int     runningIDX      = 0;
    float   fakeTimeStamp   = 0;

	ros::Rate r(hz);

    // Run reading loop
    SlamInputData frame;
//    int currentFileNumber   = ( files.size() - 20 );
    int currentFileNumber   = 0;            // Number of current file in vector of names
                                            // -1 for make 0 in first iteration
    int fileDirection       = 1;            // File sequnce direction for make revers motion
//    for( unsigned int i = 0; i < files.size(); i++ )

    std::cout << "Start image loop.."   << std::endl;
    std::cout << "File count: "         << inputData.framesCount()  << std::endl;
    std::cout << "Current number: "     << currentFileNumber        << std::endl;
    while( true )
    {
        // Read next image
//        cv::Mat imageDist = cv::imread( files[ currentFileNumber ],
//                                        CV_LOAD_IMAGE_GRAYSCALE         );

        // Read next frame
        frame = inputData.getData( currentFileNumber );

        // Check image resolution
        if( frame.m_originalFrame.rows != h_inp ||
            frame.m_originalFrame.cols != w_inp     )
		{
            if( frame.m_originalFrame.rows * frame.m_originalFrame.cols == 0 )
                printf( "failed to load image %s! skipping.\n",
                        inputData.getFramePath( currentFileNumber ).c_str() );
			else
                printf( "image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                        inputData.getFramePath( currentFileNumber ).c_str(),
                        w, h, frame.m_originalFrame.cols, frame.m_originalFrame.rows );
//			continue;
            break;
		}

        // Check image type
		assert(imageDist.type() == CV_8U);

        // Undistort image
        undistorter->undistort( imageDist, image );
		assert(image.type() == CV_8U);

        // Calculate
        if( runningIDX == 0 )
            system->randomInit( image.data, fakeTimeStamp, runningIDX ) ;
		else
            system->trackFrame( image.data, runningIDX, hz == 0, fakeTimeStamp );

		runningIDX++;
        fakeTimeStamp += 0.03;

		if(hz != 0)
			r.sleep();

        // Update file number       !!!
        currentFileNumber += fileDirection;
        std::cout << "Updated number: "     << currentFileNumber    << std::endl;

        // Check current number
        if( currentFileNumber > (int)files.size() - 1   ||
            currentFileNumber < 0                      )
        {
            // Change direction
            fileDirection *= -1;
            // Corect number
            currentFileNumber += fileDirection;

            std::cout << "Change image sequence.." << std::endl;
            std::cout << "Current file number: " <<  currentFileNumber  << std::endl;
            std::cout << "New direction value: " <<  fileDirection      << std::endl;
        }

        // Reset
        if( fullResetRequested )
		{
			printf("FULL RESET!\n");
			delete system;

            system = new SlamSystem ( w, h, K, doSlam   );
            system->setVisualization( outputWrapper     );

            fullResetRequested  = false;
            runningIDX          = 0;

            // Reset file squence
            currentFileNumber   = 0;
            fileDirection       = 1;
		}

		ros::spinOnce();

        // if somthing wrang with ros
        if( !ros::ok() )
			break;
	}

	system->finalize();

	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}
