/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "GuiWrapper.h"
#include <QtGui/QApplication>
#include <QtCore/QDir>

#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/image_encodings.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/highgui/highgui.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util3d_conversions.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UTimer.h>

#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/GetMap.h"

#include "PreferencesDialogROS.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

float max3( const float& a, const float& b, const float& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

GuiWrapper::GuiWrapper(int & argc, char** argv) :
		app_(0),
		mainWindow_(0),
		frameId_("base_link"),
		waitForTransform_(false),
		cameraNodeName_(""),
		lastOdomInfoUpdateTime_(0),
		depthScanSync_(0),
		depthSync_(0),
		depthOdomInfoSync_(0),
		stereoSync_(0),
		stereoScanSync_(0),
		stereoOdomInfoSync_(0)
{
	ros::NodeHandle nh;
	app_ = new QApplication(argc, argv);

	QString configFile = QDir::homePath()+"/.ros/rtabmapGUI.ini";
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-d") == 0)
		{
			++i;
			if(i < argc)
			{
				configFile = argv[i];
			}
			break;
		}
	}

	configFile.replace('~', QDir::homePath());

	ROS_INFO("rtabmapviz: Using configuration from \"%s\"", configFile.toStdString().c_str());
	uSleep(500);
	mainWindow_ = new MainWindow(new PreferencesDialogROS(configFile));
	mainWindow_->setWindowTitle(mainWindow_->windowTitle()+" [ROS]");
	mainWindow_->show();
	bool paused = false;
	nh.param("is_rtabmap_paused", paused, paused);
	mainWindow_->setMonitoringState(paused);
	app_->connect( app_, SIGNAL( lastWindowClosed() ), app_, SLOT( quit() ) );

	ros::NodeHandle pnh("~");

	// To receive odometry events
	bool subscribeLaserScan = false;
	bool subscribeDepth = false;
	bool subscribeOdomInfo = false;
	bool subscribeStereo = false;
	int queueSize = 10;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_); // set to use odom from TF
	pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
	pnh.param("subscribe_odom_info", subscribeOdomInfo, subscribeOdomInfo);
	pnh.param("subscribe_stereo", subscribeStereo, subscribeStereo);
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("camera_node_name", cameraNodeName_, cameraNodeName_); // used to pause the rtabmap/camera when pausing the process
	this->setupCallbacks(subscribeDepth, subscribeLaserScan, subscribeOdomInfo, subscribeStereo, queueSize);

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	infoTopic_.subscribe(nh, "info", 1);
	mapDataTopic_.subscribe(nh, "mapData", 1);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(MyInfoMapSyncPolicy(queueSize), infoTopic_, mapDataTopic_);
	infoMapSync_->registerCallback(boost::bind(&GuiWrapper::infoMapCallback, this, _1, _2));
}

GuiWrapper::~GuiWrapper()
{
	if(depthSync_)
	{
		delete depthSync_;
	}
	if(depthScanSync_)
	{
		delete depthScanSync_;
	}
	if(depthOdomInfoSync_)
	{
		delete depthOdomInfoSync_;
	}
	if(stereoSync_)
	{
		delete stereoSync_;
	}
	if(stereoScanSync_)
	{
		delete stereoScanSync_;
	}
	if(stereoOdomInfoSync_)
	{
		delete stereoOdomInfoSync_;
	}
	delete infoMapSync_;
	delete mainWindow_;
	delete app_;
}

int GuiWrapper::exec()
{
	return app_->exec();
}

void GuiWrapper::infoMapCallback(
		const rtabmap_ros::InfoConstPtr & infoMsg,
		const rtabmap_ros::MapDataConstPtr & mapMsg)
{
	//ROS_INFO("rtabmapviz: RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics stat;

	// Info
	rtabmap_ros::infoFromROS(*infoMsg, stat);

	// MapData
	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::map<int, Signature> signatures;
	std::multimap<int, Link> links;

	rtabmap_ros::mapDataFromROS(*mapMsg, poses, links, signatures, mapToOdom);

	stat.setMapCorrection(mapToOdom);
	stat.setPoses(poses);
	stat.setSignatures(signatures);
	stat.setConstraints(links);

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::processRequestedMap(const rtabmap_ros::MapData & map)
{
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	Transform mapToOdom;

	rtabmap_ros::mapDataFromROS(map, poses, constraints, signatures, mapToOdom);

	RtabmapEvent3DMap e(signatures,
				poses,
				constraints);
	QMetaObject::invokeMethod(mainWindow_, "processRtabmapEvent3DMap", Q_ARG(rtabmap::RtabmapEvent3DMap, e));
}

void GuiWrapper::handleEvent(UEvent * anEvent)
{
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		const rtabmap::ParametersMap & defaultParameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parameters = ((rtabmap::ParamEvent *)anEvent)->getParameters();
		bool modified = false;
		ros::NodeHandle nh;
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			//save only parameters with valid names
			if(defaultParameters.find((*i).first) != defaultParameters.end())
			{
				nh.setParam((*i).first, (*i).second);
				modified = true;
			}
			else if((*i).first.find('/') != (*i).first.npos)
			{
				ROS_WARN("Parameter %s is not used by the rtabmap node.", (*i).first.c_str());
			}
		}
		if(modified)
		{
			ROS_INFO("Parameters updated");
			std_srvs::Empty srv;
			if(!ros::service::call("update_parameters", srv))
			{
				ROS_ERROR("Can't call \"update_parameters\" service");
			}
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEventCmd") == 0)
	{
		std_srvs::Empty emptySrv;
		rtabmap::RtabmapEventCmd * cmdEvent = (rtabmap::RtabmapEventCmd *)anEvent;
		rtabmap::RtabmapEventCmd::Cmd cmd = cmdEvent->getCmd();
		if(cmd == rtabmap::RtabmapEventCmd::kCmdResetMemory)
		{
			if(!ros::service::call("reset", emptySrv))
			{
				ROS_ERROR("Can't call \"reset\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPause)
		{
			if(cmdEvent->getInt())
			{
				// Pause the camera if the rtabmap/camera node is used
				if(!cameraNodeName_.empty())
				{
					std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause true", cameraNodeName_.c_str());
					system(str.c_str());
				}

				// Pause visual_odometry
				ros::service::call("pause_odom", emptySrv);

				// Pause rtabmap
				if(!ros::service::call("pause", emptySrv))
				{
					ROS_ERROR("Can't call \"pause\" service");
				}
			}
			else
			{
				// Resume rtabmap
				if(!ros::service::call("resume", emptySrv))
				{
					ROS_ERROR("Can't call \"resume\" service");
				}

				// Pause visual_odometry
				ros::service::call("resume_odom", emptySrv);

				// Resume the camera if the rtabmap/camera node is used
				if(!cameraNodeName_.empty())
				{
					std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause false", cameraNodeName_.c_str());
					system(str.c_str());
				}
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdTriggerNewMap)
		{
			if(!ros::service::call("trigger_new_map", emptySrv))
			{
				ROS_ERROR("Can't call \"trigger_new_map\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapLocal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapGlobal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphLocal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal)
		{
			rtabmap_ros::GetMap getMapSrv;
			getMapSrv.request.global = cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapGlobal || cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal;
			getMapSrv.request.optimized = cmdEvent->getInt();
			getMapSrv.request.graphOnly = cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal || cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphLocal;
			if(!ros::service::call("get_map", getMapSrv))
			{
				ROS_WARN("Can't call \"get_map\" service");
				this->post(new RtabmapEvent3DMap(1)); // service error
			}
			else
			{
				processRequestedMap(getMapSrv.response.data);
			}
		}
		else
		{
			ROS_WARN("Not handled command (%d)...", cmd);
		}
	}
	else if(anEvent->getClassName().compare("OdometryResetEvent") == 0)
	{
		std_srvs::Empty srv;
		if(!ros::service::call("reset_odom", srv))
		{
			ROS_ERROR("Can't call \"reset_odom\" service, (will only work with rtabmap/visual_odometry node.)");
		}
	}
}

Transform GuiWrapper::getTransform(const std::string & fromFrameId, const std::string & toFrameId, const ros::Time & stamp) const
{
	// TF ready?
	Transform localTransform;
	try
	{
		if(waitForTransform_ && !stamp.isZero())
		{
			//if(!tfBuffer_.canTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			if(!tfListener_.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			{
				ROS_WARN("Could not get transform from %s to %s after 1 second!", fromFrameId.c_str(), toFrameId.c_str());
				return localTransform;
			}
		}

		tf::StampedTransform tmp;
		tfListener_.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
		localTransform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
	}
	return localTransform;
}

void GuiWrapper::commonDepthCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	if(UTimer::now() - lastOdomInfoUpdateTime_ > 0.1 &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics())
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(!(imageMsg.get() == 0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(depthMsg.get() == 0 ||
				 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
				 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
				 depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
			return;
		}

		std_msgs::Header odomHeader;
		if(odomMsg.get())
		{
			odomHeader = odomMsg->header;
		}
		else
		{
			if(scanMsg.get())
			{
				odomHeader = scanMsg->header;
			}
			else if(cameraInfoMsg.get())
			{
				odomHeader = cameraInfoMsg->header;
			}
			else if(depthMsg.get())
			{
				odomHeader = depthMsg->header;
			}
			else if(imageMsg.get())
			{
				odomHeader = imageMsg->header;
			}
			odomHeader.frame_id = odomFrameId_;
		}

		Transform odomT = getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp);
		cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
		if(odomMsg.get())
		{
			UASSERT(odomMsg->pose.covariance.size() == 36);
			if(!(odomMsg->pose.covariance[0] == 0 &&
				 odomMsg->pose.covariance[7] == 0 &&
				 odomMsg->pose.covariance[14] == 0 &&
				 odomMsg->pose.covariance[21] == 0 &&
				 odomMsg->pose.covariance[28] == 0 &&
				 odomMsg->pose.covariance[35] == 0))
			{
				covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->pose.covariance.data()).clone();
			}
		}
		if(odomHeader.frame_id.empty())
		{
			ROS_ERROR("Odometry frame not set!?");
			return;
		}

		//for sync transform
		if(odomT.isNull())
		{
			return;
		}

		CameraModel cameraModel;
		if(cameraInfoMsg.get())
		{
			Transform localTransform = getTransform(frameId_, cameraInfoMsg->header.frame_id, cameraInfoMsg->header.stamp);
			if(localTransform.isNull())
			{
				return;
			}
			// sync with odometry stamp
			if(odomHeader.stamp != cameraInfoMsg->header.stamp)
			{
				Transform sensorT = getTransform(odomHeader.frame_id, frameId_, cameraInfoMsg->header.stamp);
				if(sensorT.isNull())
				{
					return;
				}
				localTransform = odomT.inverse() * sensorT * localTransform;
			}

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfoMsg);
			if(!localTransform.isNull())
			{
				cameraModel = CameraModel(model.fx(), model.fy(), model.cx(), model.cy(), localTransform);
			}
		}

		cv::Mat rgb;
		if(imageMsg.get())
		{
			if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				rgb = cv_bridge::toCvCopy(imageMsg, "mono8")->image;
			}
			else
			{
				rgb = cv_bridge::toCvCopy(imageMsg, "bgr8")->image;
			}
		}

		cv::Mat depth;
		if(depthMsg.get())
		{
			depth = cv_bridge::toCvCopy(depthMsg)->image;
		}

		cv::Mat scan;
		if(scanMsg.get() != 0)
		{
			// make sure the frame of the laser is updated too
			if(getTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp).isNull())
			{
				return;
			}

			//transform in frameId_ frame
			sensor_msgs::PointCloud2 scanOut;
			laser_geometry::LaserProjection projection;
			projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(scanOut, *pclScan);

			// sync with odometry stamp
			if(odomHeader.stamp != scanMsg->header.stamp)
			{
				if(!odomT.isNull())
				{
					Transform sensorT = getTransform(odomHeader.frame_id, frameId_, scanMsg->header.stamp);
					if(sensorT.isNull())
					{
						return;
					}
					Transform t = odomT.inverse() * sensorT;
					pclScan = util3d::transformPointCloud(pclScan, t);

				}
			}
			scan = util3d::laserScanFromPointCloud(*pclScan);
		}

		rtabmap::OdometryInfo info;
		if(odomInfoMsg.get())
		{
			info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
		}

		rtabmap::OdometryEvent odomEvent(
			rtabmap::SensorData(
					scan,
					scanMsg.get()?(int)scanMsg->ranges.size():0,
					rgb,
					depth,
					cameraModel,
					odomHeader.seq,
					rtabmap_ros::timestampFromROS(odomHeader.stamp)),
			odomT,
			covariance,
			info);

		QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent));
	}
}

void GuiWrapper::commonStereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	// limit 10 Hz max
	if(UTimer::now() - lastOdomInfoUpdateTime_ > 0.1 &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics())
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		UASSERT(leftImageMsg.get() && rightImageMsg.get());
		UASSERT(leftCamInfoMsg.get() && rightCamInfoMsg.get());

		if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			 leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			 leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
		   !(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			 rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			 rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		std_msgs::Header odomHeader;
		if(odomMsg.get())
		{
			odomHeader = odomMsg->header;
		}
		else
		{
			if(scanMsg.get())
			{
				odomHeader = scanMsg->header;
			}
			else
			{
				odomHeader = leftCamInfoMsg->header;
			}
			odomHeader.frame_id = odomFrameId_;
		}

		Transform odomT = getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp);
		cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
		if(odomMsg.get())
		{
			UASSERT(odomMsg->pose.covariance.size() == 36);
			if(!(odomMsg->pose.covariance[0] == 0 &&
				 odomMsg->pose.covariance[7] == 0 &&
				 odomMsg->pose.covariance[14] == 0 &&
				 odomMsg->pose.covariance[21] == 0 &&
				 odomMsg->pose.covariance[28] == 0 &&
				 odomMsg->pose.covariance[35] == 0))
			{
				covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->pose.covariance.data()).clone();
			}
		}
		if(odomHeader.frame_id.empty())
		{
			ROS_ERROR("Odometry frame not set!?");
			return;
		}

		//for sync transform
		if(odomT.isNull())
		{
			return;
		}

		Transform localTransform = getTransform(frameId_, leftCamInfoMsg->header.frame_id, leftCamInfoMsg->header.stamp);
		if(localTransform.isNull())
		{
			return;
		}
		// sync with odometry stamp
		if(odomHeader.stamp != leftCamInfoMsg->header.stamp)
		{
			Transform sensorT = getTransform(odomHeader.frame_id, frameId_, leftCamInfoMsg->header.stamp);
			if(sensorT.isNull())
			{
				return;
			}
			localTransform = odomT.inverse() * sensorT * localTransform;
		}

		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*leftCamInfoMsg, *rightCamInfoMsg);
		rtabmap::StereoCameraModel stereoModel(
				model.left().fx(),
				model.left().fy(),
				model.left().cx(),
				model.left().cy(),
				model.baseline(),
				localTransform);

		// left
		cv_bridge::CvImageConstPtr ptrImage;
		cv::Mat left;
		if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			left = cv_bridge::toCvCopy(leftImageMsg, "mono8")->image;
		}
		else
		{
			left = cv_bridge::toCvCopy(leftImageMsg, "bgr8")->image;
		}

		// right
		cv::Mat right = cv_bridge::toCvCopy(rightImageMsg, "mono8")->image;

		cv::Mat scan;
		if(scanMsg.get() != 0)
		{
			// make sure the frame of the laser is updated too
			if(getTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp).isNull())
			{
				return;
			}

			//transform in frameId_ frame
			sensor_msgs::PointCloud2 scanOut;
			laser_geometry::LaserProjection projection;
			projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(scanOut, *pclScan);

			// sync with odometry stamp
			if(odomHeader.stamp != scanMsg->header.stamp)
			{
				if(!odomT.isNull())
				{
					Transform sensorT = getTransform(odomHeader.frame_id, frameId_, scanMsg->header.stamp);
					if(sensorT.isNull())
					{
						return;
					}
					Transform t = odomT.inverse() * sensorT;
					pclScan = util3d::transformPointCloud(pclScan, t);

				}
			}
			scan = util3d::laserScanFromPointCloud(*pclScan);
		}

		rtabmap::OdometryInfo info;
		if(odomInfoMsg.get())
		{
			info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
		}

		rtabmap::OdometryEvent odomEvent(
			rtabmap::SensorData(
					scan,
					scanMsg.get()?(int)scanMsg->ranges.size():0,
					left,
					right,
					stereoModel,
					odomHeader.seq,
					rtabmap_ros::timestampFromROS(odomHeader.stamp)),
			odomT,
			covariance,
			info);

		QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent));
	}
}

// With odom msg
void GuiWrapper::defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg)
{
	commonDepthCallback(
			odomMsg,
			sensor_msgs::ImageConstPtr(),
			sensor_msgs::ImageConstPtr(),
			sensor_msgs::CameraInfoConstPtr(),
			sensor_msgs::LaserScanConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::depthCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			odomMsg,
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::depthOdomInfoCallback(
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			odomMsg,
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			odomInfoMsg);
}

void GuiWrapper::depthScanCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			odomMsg,
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			scanMsg,
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::stereoScanCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			odomMsg,
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			scanMsg,
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::stereoOdomInfoCallback(
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			odomMsg,
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			odomInfoMsg);
}

void GuiWrapper::stereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			odomMsg,
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

// With odom TF
void GuiWrapper::depthTFCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			nav_msgs::OdometryConstPtr(),
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::depthOdomInfoTFCallback(
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			nav_msgs::OdometryConstPtr(),
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			odomInfoMsg);
}

void GuiWrapper::depthScanTFCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	commonDepthCallback(
			nav_msgs::OdometryConstPtr(),
			imageMsg,
			depthMsg,
			cameraInfoMsg,
			scanMsg,
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::stereoScanTFCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			nav_msgs::OdometryConstPtr(),
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			scanMsg,
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::stereoOdomInfoTFCallback(
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			nav_msgs::OdometryConstPtr(),
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			odomInfoMsg);
}

void GuiWrapper::stereoTFCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
{
	commonStereoCallback(
			nav_msgs::OdometryConstPtr(),
			leftImageMsg,
			rightImageMsg,
			leftCameraInfoMsg,
			rightCameraInfoMsg,
			sensor_msgs::LaserScanConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

void GuiWrapper::setupCallbacks(
		bool subscribeDepth,
		bool subscribeLaserScan,
		bool subscribeOdomInfo,
		bool subscribeStereo,
		int queueSize)
{
	ros::NodeHandle nh; // public
	ros::NodeHandle pnh("~"); // private

	if(subscribeDepth && subscribeStereo)
	{
		ROS_WARN("\"subscribe_depth\" already true, ignoring \"subscribe_stereo\".");
	}
	if(!subscribeDepth && !subscribeStereo && subscribeLaserScan)
	{
		ROS_WARN("Cannot subscribe to laser scan without depth or stereo subscription...");
	}

	if(subscribeDepth)
	{
		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

		if(odomFrameId_.empty())
		{
			odomSub_.subscribe(nh, "odom", 1);
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), scanSub_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
				depthScanSync_->registerCallback(boost::bind(&GuiWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else if(subscribeOdomInfo)
			{
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				depthOdomInfoSync_ = new message_filters::Synchronizer<MyDepthOdomInfoSyncPolicy>(MyDepthOdomInfoSyncPolicy(queueSize), odomInfoSub_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
				depthOdomInfoSync_->registerCallback(boost::bind(&GuiWrapper::depthOdomInfoCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						odomInfoSub_.getTopic().c_str());
			}
			else
			{
				depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
				depthSync_->registerCallback(boost::bind(&GuiWrapper::depthCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomSub_.getTopic().c_str());
			}
		}
		else
		{
			// use TF as odom
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				depthScanTFSync_ = new message_filters::Synchronizer<MyDepthScanTFSyncPolicy>(MyDepthScanTFSyncPolicy(queueSize), scanSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
				depthScanTFSync_->registerCallback(boost::bind(&GuiWrapper::depthScanTFCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else if(subscribeOdomInfo)
			{
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				depthOdomInfoTFSync_ = new message_filters::Synchronizer<MyDepthOdomInfoTFSyncPolicy>(MyDepthOdomInfoTFSyncPolicy(queueSize), odomInfoSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
				depthOdomInfoTFSync_->registerCallback(boost::bind(&GuiWrapper::depthOdomInfoTFCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomInfoSub_.getTopic().c_str());
			}
			else
			{
				depthTFSync_ = new message_filters::Synchronizer<MyDepthTFSyncPolicy>(MyDepthTFSyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
				depthTFSync_->registerCallback(boost::bind(&GuiWrapper::depthTFCallback, this, _1, _2, _3));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str());
			}
		}
	}
	else if(subscribeStereo)
	{
		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
		imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

		if(odomFrameId_.empty())
		{
			odomSub_.subscribe(nh, "odom", 1);
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				stereoScanSync_ = new message_filters::Synchronizer<MyStereoScanSyncPolicy>(MyStereoScanSyncPolicy(queueSize), scanSub_, odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoScanSync_->registerCallback(boost::bind(&GuiWrapper::stereoScanCallback, this, _1, _2, _3, _4, _5, _6));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else if(subscribeOdomInfo)
			{
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				stereoOdomInfoSync_ = new message_filters::Synchronizer<MyStereoOdomInfoSyncPolicy>(MyStereoOdomInfoSyncPolicy(queueSize), odomInfoSub_, odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoOdomInfoSync_->registerCallback(boost::bind(&GuiWrapper::stereoOdomInfoCallback, this, _1, _2, _3, _4, _5, _6));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						odomInfoSub_.getTopic().c_str());
			}
			else
			{
				stereoSync_ = new message_filters::Synchronizer<MyStereoSyncPolicy>(MyStereoSyncPolicy(queueSize), odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoSync_->registerCallback(boost::bind(&GuiWrapper::stereoCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomSub_.getTopic().c_str());
			}
		}
		else
		{
			//use odom TF
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				stereoScanTFSync_ = new message_filters::Synchronizer<MyStereoScanTFSyncPolicy>(MyStereoScanTFSyncPolicy(queueSize), scanSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoScanTFSync_->registerCallback(boost::bind(&GuiWrapper::stereoScanTFCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else if(subscribeOdomInfo)
			{
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				stereoOdomInfoTFSync_ = new message_filters::Synchronizer<MyStereoOdomInfoTFSyncPolicy>(MyStereoOdomInfoTFSyncPolicy(queueSize), odomInfoSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoOdomInfoTFSync_->registerCallback(boost::bind(&GuiWrapper::stereoOdomInfoTFCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomInfoSub_.getTopic().c_str());
			}
			else
			{
				stereoTFSync_ = new message_filters::Synchronizer<MyStereoTFSyncPolicy>(MyStereoTFSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				stereoTFSync_->registerCallback(boost::bind(&GuiWrapper::stereoTFCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str());
			}
		}
	}
	else // default odom only
	{
		defaultSub_ = nh.subscribe("odom", 1, &GuiWrapper::defaultCallback, this);

		ROS_INFO("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				odomSub_.getTopic().c_str());
	}
}

