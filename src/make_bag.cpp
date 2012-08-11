/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 3 June 2012
 *********************************************************************
 *
 * Status
 *  untested
 * 
 *
 * Change Log:
 *  3 June 2012 Created
 *
 **********************************************************************
 * This is a simple robot rosbag capture program. Told it would do a better
 * job of grabbing kinect data and having a good time step than using
 * the commandline version.
 */

//---- ROS -------------------------------------------
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>                // odometry
#include <geometry_msgs/Twist.h>              // command and velocity
#include <geometry_msgs/Point.h>              // servo motors
#include <sensor_msgs/Imu.h>                  // IMU messages
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <soccer/Imu.h>

//---- C++ ----------------------------------------------
#include <string>

// Global variables
rosbag::Bag bag;
ros::Time stamp;

/**
 * Grab depth image and camera_info
 */
void grabDepth(const sensor_msgs::ImageConstPtr& msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg){
	stamp = ros::Time::now();
	bag.write("kinect_camera/depth",stamp,msg);
	bag.write("kinect_camera/camera_info",stamp,info_msg);
}

/**
 * Grab rgb image ... don't get camera_info, got it in depth image
 * so use that one
 */
void grabRGB(const sensor_msgs::ImageConstPtr& msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg){
	stamp = ros::Time::now();
	bag.write("kinect_camera/rgb",stamp,msg);
}

/**
 * Grab the kinect's IMU data, useful for knowing its orientation
 */
void grabKinectIMU(const sensor_msgs::Imu::ConstPtr& msg){
	stamp = ros::Time::now();
	bag.write("kinect_camera/imu",stamp,msg);
}

/**
 * Grab the robot's IMU data
 */
void grabIMU(const soccer::Imu::ConstPtr& msg){
	stamp = ros::Time::now();
	bag.write("imu",stamp,msg);
}

int main(int argc, char *argv[])
{   
	// init ROS stuff  
	ros::init(argc, argv, "makebag");  
	ros::NodeHandle nh;
	ROS_INFO("Start");
	
	// setup all of the callbacks
	std::string filename = "~/ros_sandbox/"; //ros::package::getPath("soccer");
	filename.append("test.bag");
	bag.open(filename, rosbag::bagmode::Write);
	
	image_transport::ImageTransport transport(nh);
    image_transport::CameraSubscriber depth_sub = transport.subscribeCamera("/kinect_camera/depth/image_raw", 1, &grabDepth);
    image_transport::CameraSubscriber rgb_sub = transport.subscribeCamera("/kinect_camera/rgb/image_raw", 1, &grabRGB);
  
	ros::Subscriber imu = nh.subscribe("imu",10,grabIMU);
	ros::Subscriber kinectImu = nh.subscribe("kinect_camera/imu",10,grabKinectIMU);
	
	// go!!
	ros::spin();
	
	return 0;
}



