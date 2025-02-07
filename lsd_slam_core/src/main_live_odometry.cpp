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


#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include <X11/Xlib.h>

using namespace lsd_slam;
int main( int argc, char** argv )
{
    // X Windows initializing
    XInitThreads();

    // init ROS LSD-SLAM
	ros::init(argc, argv, "LSD_SLAM");

    // ********s********** Configure dynamic callbacks ***********************************
	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

    // Get current package path
	packagePath = ros::package::getPath("lsd_slam_core")+"/";

    // ***************** Run Image reader stream *****************************************
    // Create image receiver stream
	InputImageStream* inputStream = new ROSImageStreamThread();

    // Get path to calibration file
	std::string calibFile;
	if(ros::param::get("~calib", calibFile))
	{
		ros::param::del("~calib");
        // Set calibration info for undistorter and calibration
		inputStream->setCalibration(calibFile);
	}
	else
		inputStream->setCalibration("");

    // Run input stream
	inputStream->run();

    // ************************* My FLAG FOR SAVING Results *****************************
    // open image files: first try to open as file.
    bool saveRes = false;
    ros::param::get("~saveResults", saveRes);

    // initialize output wrapper
    Output3DWrapper*    outputWrapper = new ROSOutput3DWrapper( inputStream->width(), inputStream->height() );
    LiveSLAMWrapper     slamNode(inputStream, outputWrapper);
	slamNode.Loop();

	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	return 0;
}
