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

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "util/settings.h"


#include "std_msgs/Float32MultiArray.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

#include <boost/filesystem.hpp>
#include <inttypes.h>

namespace lsd_slam
{

ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height, std::string imagePath, bool outputFiles)
{
        this->width  = width;
        this->height = height;

        liveframe_channel   = nh_.resolveName("lsd_slam/liveframes");
        liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

        keyframe_channel    = nh_.resolveName("lsd_slam/keyframes");
        keyframe_publisher  = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

        graph_channel   = nh_.resolveName("lsd_slam/graph");
        graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

        debugInfo_channel   = nh_.resolveName("lsd_slam/debug");
        debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

        pose_channel    = nh_.resolveName("lsd_slam/pose");
        pose_publisher  = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);

        publishLvl = 0;
        // Folders controll ***********************************************
        m_bSaveOutputs = outputFiles;
        if( m_bSaveOutputs )
        {
            // Make path to images
            boost::filesystem::path tempPath(imagePath);
            // Get base path
            tempPath = tempPath.parent_path();

            // Set new output directory
            m_pOutputFolders = new FolderController("slamOutput/", tempPath.string() );
            // Make output folder
            m_pOutputFolders->makeSubfolder("experiment");

            // Estimated camera positons file *********************************
    //        std::string save_folder;
    //        char        buf[500];
    //        int         k;
    //        save_folder = ros::package::getPath("lsd_slam_core")+"/slam_Coods_output/";
    //        // Folder clening **
    //        snprintf(buf,500,"rm -rf %s",save_folder.c_str());
    //        k = system(buf);
    //        snprintf(buf,500,"mkdir %s",save_folder.c_str());
    //        k = system(buf);
    //        save_folder = ros::package::getPath("lsd_slam_core")+"/slam_Coods_output/slamOriginCameraPositions.txt";

            boost::filesystem::create_directory( m_pOutputFolders->getCurrentSubfloder() + "/slam_Coods_output" );

            std::string filePath = m_pOutputFolders->getCurrentSubfloder() + "slam_Coods_output/slamOriginCameraPositions.txt" ;
            // File stream for estimated camera position data
            m_sCameraPositionsFileStream = new std::ofstream( filePath.c_str() );
            // First Frame
            *m_sCameraPositionsFileStream << "0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1" << std::endl;

            // Estimated KFs positons file *********************************
            filePath = m_pOutputFolders->getCurrentSubfloder() + "slam_Coods_output/slamOriginKeyframesCoods.txt";
            // File stream for estimated camera position data
            m_pKeyframesFileStream = new std::ofstream( filePath.c_str() );

            // KFs output folder *******************************************
            m_sOutputCoodsDir = m_pOutputFolders->getCurrentSubfloder() + "/slam_KFs_output/";
            boost::filesystem::create_directory( m_sOutputCoodsDir );

    //        // Folder clening **
    //        snprintf(buf,500,"rm -rf %s",m_sOutputCoodsDir.c_str());
    //        k = system(buf);
    //        snprintf(buf,500,"mkdir %s",m_sOutputCoodsDir.c_str());
    //        k = system(buf);

            // KFs output file name ****************************************
            // Path to dir with part of file name
            m_sOutputKFsFilePart = m_sOutputCoodsDir +  "KeyFrame";

            // Set template
            xyzFilenameFormat = "%s_%010"PRIu32".xyz";

            // Mesuring buffer length
            int max_name_len = snprintf( NULL,
                                         0,
                                         xyzFilenameFormat,
                                         m_sOutputKFsFilePart.data(),
                                         UINT32_MAX         );


            // Reserve buffer
            xyzFilename = (char*)calloc( max_name_len + 1, sizeof(char) );
        }
}

ROSOutput3DWrapper::~ROSOutput3DWrapper()
{
}


void ROSOutput3DWrapper::publishKeyframe(Frame* f)
{
    // Make new message
	lsd_slam_viewer::keyframeMsg fMsg;

    // Lock data
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

    // Save ID
    fMsg.id         = f->id();
    // Save timestamp
    fMsg.time       = f->timestamp();
    // Is it keyframe?
	fMsg.isKeyframe = true;

    // Save frame size
    int wide    = f->width(publishLvl);
    int h       = f->height(publishLvl);

    // Save camera parameters
	memcpy(fMsg.camToWorld.data(),f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = f->fx(publishLvl);
	fMsg.fy = f->fy(publishLvl);
	fMsg.cx = f->cx(publishLvl);
    fMsg.cy = f->cy(publishLvl);

    // Save frame size to message
    fMsg.width  = wide;
	fMsg.height = h;

    // Make new buffer for point cloud
    fMsg.pointcloud.resize(wide*h*sizeof(InputPointDense));

    // receive pointer
	InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

    const float* idepth     = f->idepth     (publishLvl);
    const float* idepthVar  = f->idepthVar  (publishLvl);
    const float* color      = f->image      (publishLvl);

    for(int idx=0;idx < wide*h; idx++)
	{
        pc[idx].idepth      = idepth    [idx];
        pc[idx].idepth_var  = idepthVar [idx];

		pc[idx].color[0] = color[idx];
		pc[idx].color[1] = color[idx];
		pc[idx].color[2] = color[idx];
		pc[idx].color[3] = color[idx];
	}

	keyframe_publisher.publish(fMsg);

    //************************************************ My part
    if( m_bSaveOutputs )
    {
        // Make new file name
        sprintf( xyzFilename, xyzFilenameFormat, m_sOutputKFsFilePart.data(), f->id() );

        // open file stream
        std::ofstream* pXyzStream = new std::ofstream( xyzFilename );

        // Save id of frame
        *pXyzStream << f->id() << std::endl;

        // Save image size
        int wight = f->width ( publishLvl );
        int hight = f->height( publishLvl );

        *pXyzStream << wight << " " << hight << std::endl;

        // Copy camera params
        *pXyzStream << f->fx( publishLvl ) << " ";
        *pXyzStream << f->fy( publishLvl ) << " ";
        *pXyzStream << f->cx( publishLvl ) << " ";
        *pXyzStream << f->cy( publishLvl ) << " ";
        *pXyzStream << std::endl;

        // Temp Solution for translation
        double x, y, z, w, s;

        //
        Sim3 camToWorld = f->getScaledCamToWorld();

        x = camToWorld.translation()[0];
        y = camToWorld.translation()[1];
        z = camToWorld.translation()[2];

        *pXyzStream             << " " << x << " " << y << " " << z;

        *m_pKeyframesFileStream << f->id() << " " << x << " " << y << " " << z;

        x = camToWorld.quaternion().x();
        y = camToWorld.quaternion().y();
        z = camToWorld.quaternion().z();
        w = camToWorld.quaternion().w();

        s = camToWorld.scale();

        if (  w < 0)
        {
            x *= -1;
            y *= -1;
            z *= -1;
            w *= -1;
        }

    //    *pXyzStream             <<  " " << w << " " << x << " " << y << " " << z << " " << s << std::endl;

        *m_pKeyframesFileStream <<  " " << w << " " << x << " " << y << " " << z << " " << s << std::endl;

        // Depth INFO save to file
        /*const float**/ idepth     = f->idepth     (publishLvl);
        /*const float**/ idepthVar  = f->idepthVar  (publishLvl);
        /*const float**/ color      = f->image      (publishLvl);

        for( int idx = 0; idx < wight * hight; idx++ )
        {
            *pXyzStream << idepth    [idx] << " ";
            *pXyzStream << idepthVar [idx] << " ";

            *pXyzStream << color[idx] << " ";
            *pXyzStream << color[idx] << " ";
            *pXyzStream << color[idx] << " ";
            *pXyzStream << color[idx] << " ";

            *pXyzStream << std::endl;
        }

        // close file stream
        pXyzStream->flush();
        pXyzStream->close();
    }
}

void ROSOutput3DWrapper::publishTrackedFrame(Frame* kf)
{
	lsd_slam_viewer::keyframeMsg fMsg;

    fMsg.id         = kf->id();
    fMsg.time       = kf->timestamp();
	fMsg.isKeyframe = false;

	memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);

	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);

    fMsg.width  = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	fMsg.pointcloud.clear();

	liveframe_publisher.publish(fMsg);

	SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

	geometry_msgs::PoseStamped pMsg;

	pMsg.pose.position.x = camToWorld.translation()[0];
	pMsg.pose.position.y = camToWorld.translation()[1];
	pMsg.pose.position.z = camToWorld.translation()[2];

	pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	if (pMsg.pose.orientation.w < 0)
	{
		pMsg.pose.orientation.x *= -1;
		pMsg.pose.orientation.y *= -1;
		pMsg.pose.orientation.z *= -1;
		pMsg.pose.orientation.w *= -1;
	}

	pMsg.header.stamp = ros::Time(kf->timestamp());
	pMsg.header.frame_id = "world";
	pose_publisher.publish(pMsg);


        //**************************************************************
    if( m_bSaveOutputs )
    {
        float sim3T[3], sim3Q[4], sim3S;
        float  se3T[3],  se3Q[4],  se3S;

        float  tempT[3],  tempQ[4],  tempS;

    //    float test[7];
    //    float tx, ty, tz, qx, qy, qz, qw, s;

        Sim3    Sim3camToWorld  = kf->getScaledCamToWorld();

        sim3T[0] = Sim3camToWorld.translation()[0];
        sim3T[1] = Sim3camToWorld.translation()[1];
        sim3T[2] = Sim3camToWorld.translation()[2];

        sim3Q[0] = Sim3camToWorld.quaternion().w();
        sim3Q[1] = Sim3camToWorld.quaternion().x();
        sim3Q[2] = Sim3camToWorld.quaternion().y();
        sim3Q[3] = Sim3camToWorld.quaternion().z();

        if (  sim3Q[0] < 0)
        {
            sim3Q[0] *= -1;
            sim3Q[1] *= -1;
            sim3Q[2] *= -1;
            sim3Q[3] *= -1;
        }

        sim3S =  Sim3camToWorld.scale();

        *m_sCameraPositionsFileStream << kf->id()   << " " << sim3T[0]  << " " << sim3T[1] << " " << sim3T[2]
                                                    << " " << sim3Q[0]  << " " << sim3Q[1] << " " << sim3Q[2] << " " << sim3Q[3]
                                                    << " " << sim3S;

    //    memcpy (test, Sim3camToWorld.cast<float>().data(), sizeof(float)*7 );

    //    tempT[0] = Sim3camToWorld.translation()[0];
    //    tempT[1] = Sim3camToWorld.translation()[1];
    //    tempT[2] = Sim3camToWorld.translation()[2];

    //    tempQ[0] = Sim3camToWorld.so3().unit_quaternion().w();
    //    tempQ[1] = Sim3camToWorld.so3().unit_quaternion().x();
    //    tempQ[2] = Sim3camToWorld.so3().unit_quaternion().y();
    //    tempQ[3] = Sim3camToWorld.so3().unit_quaternion().z();

        // Convert to SE3
        SE3     SE3camToWorld   = se3FromSim3( Sim3camToWorld );

        se3T[0] = SE3camToWorld.translation()[0];
        se3T[1] = SE3camToWorld.translation()[1];
        se3T[2] = SE3camToWorld.translation()[2];

        se3Q[0] = SE3camToWorld.unit_quaternion().w();
        se3Q[1] = SE3camToWorld.unit_quaternion().x();
        se3Q[2] = SE3camToWorld.unit_quaternion().y();
        se3Q[3] = SE3camToWorld.unit_quaternion().z();


        if (  se3Q[0] < 0)
        {
            se3Q[0] *= -1;
            se3Q[1] *= -1;
            se3Q[2] *= -1;
            se3Q[3] *= -1;
        }

        *m_sCameraPositionsFileStream   << " "  << se3T[0]  << " " << se3T[1] << " " << se3T[2]
                                        << " "  << se3Q[0]  << " " << se3Q[1] << " " << se3Q[2] << " " << se3Q[3];
    //                                    << std::endl;

    //    se3S =  SE3camToWorld.scale();

        tempQ[0] = SE3camToWorld.so3().unit_quaternion().w();
        tempQ[1] = SE3camToWorld.so3().unit_quaternion().x();
        tempQ[2] = SE3camToWorld.so3().unit_quaternion().y();
        tempQ[3] = SE3camToWorld.so3().unit_quaternion().z();

    //    x = camToWorld.so3().unit_quaternion().x();
    //    y = camToWorld.so3().unit_quaternion().y();
    //    z = camToWorld.so3().unit_quaternion().z();
    //    w = camToWorld.so3().unit_quaternion().w();

    //    s =  camToWorld.scale();

    //    if (  w < 0)
    //    {
    //        x *= -1;
    //        y *= -1;
    //        z *= -1;
    //        w *= -1;
    //    }

    //    *m_sCameraPositionsFileStream << " " << w << " " << x << " " << y  << " " << z  << " " << s;

        Sophus::Matrix4f m      = Sim3camToWorld.matrix().cast<float>();
        Sophus::Matrix4f se3M   = SE3camToWorld.matrix().cast<float>();

        *m_sCameraPositionsFileStream   << " " << m(0,0) << " " << m(0,1) << " " << m(0,2)  << " " << m(0,3)
                                        << " " << m(1,0) << " " << m(1,1) << " " << m(1,2)  << " " << m(1,3)
                                        << " " << m(2,0) << " " << m(2,1) << " " << m(2,2)  << " " << m(2,3)
                                        << " " << m(3,0) << " " << m(3,1) << " " << m(3,2)  << " " << m(3,3) << std::endl;

    }
}



void ROSOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	lsd_slam_viewer::keyframeGraphMsg gMsg;

	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
		constraintData[i].err = sqrt(err.dot(err));
	}
	graph->edgesListsMutex.unlock();

	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	}
	graph->keyframesAllMutex.unlock_shared();

	graph_publisher.publish(gMsg);
}

void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		msg.data.push_back((float)(data[i]));

	debugInfo_publisher.publish(msg);
}

}
