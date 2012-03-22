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
 * Author: Kevin J. Walchko on 6 Mar 2011
 *********************************************************************
 *
 * Status
 * X   All sensor msgs published
 * X   Teleop through Twist msg - simplistic, need better control
 * X   Publish TF for robot base
 * O   Publish TF for sensors
 * O   RVIZ model
 * O   Integrate new I2C IMU (accel, gyo, compass) for use with EFK
 *        encoders - x,y,heading
 *        imu - roll,pitch,accel,gyro
 *        compass - heading
 * O   Enable timed events (beeps, lights) like wiimote
 * X   Enable simulation capability (random messages)
 * O   Enable simulation capability (dynamics)
 * 
 *
 * Change Log:
 *  6 Mar 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>         // transforms
#include <nav_msgs/Odometry.h>                // odometry
#include <geometry_msgs/Twist.h>              // command and velocity
#include <geometry_msgs/Point.h>              // servo motors
#include <sensor_msgs/Imu.h>                  // IMU messages


#include <std_msgs/String.h> // for simulation
#include <string> // C++ for CerealPort
#include <sstream> // to concatonate c++ strings together


#include <iostream>
#include <Eigen/Dense>

#include "../kevin.h"
#include "soccer/Imu.h"
#include "../MadgwickAHRS/MadgwickAHRS.h"

using namespace kevin;

///////////////////////////////////////////////////////////////////////////////


class Navigation {
public:
    Navigation(ros::NodeHandle& n){
        
        // Publish ------------------------------------
        //serial_pub = n.advertise<std_msgs::String>("/serial_fromRobot", 50);
        
        // Subcriptions -------------------------------
        serial_sub  = n.subscribe<soccer::Imu>("/imu", 1, &Navigation::callback, this);
	
	    setBeta(0.01);
    }
    
    void callback(const soccer::Imu::ConstPtr& imu){
        //extern volatile float beta = 0.01;
        
		//--- from design note for tilt compensated compass ---
		mx = imu->mags.x;
		my = imu->mags.y;
		mz = imu->mags.z;
		double norm = sqrt(mx*mx+my*my+mz*mz);
		mx /= norm;
		my /= norm;
		mz /= norm;
		
		ax = imu->accels.x;
		ay = imu->accels.y;
		az = imu->accels.z;
		norm = sqrt(ax*ax+ay*ay+az*az);
		ax /= norm;
		ay /= norm;
		az /= norm;
		//ROS_INFO("Accel: %2.2f %2.2f %2.2f",ax,ay,az);
		
		pitch = asin(-ax);
		
		gx = imu->gyros.x;
		gy = imu->gyros.y;
		gz = imu->gyros.z;
		
		//--- tilt compensated compass design note ---
		if( fabs(pitch) >= (89.0*M_PI/180.0) ) roll = 0.0;
		else roll = asin(ay/cos(pitch));
		
		double xh = mx*cos(pitch)+mz*sin(pitch);
		double yh = mx*sin(roll)*sin(pitch)+my*cos(roll)-mz*sin(roll)*cos(pitch);
		yaw = atan2(yh,xh); // << not working right!!! [FIXME 20120312 kjw]
		
		ROS_INFO("RPY[deg]: %3.1f %3.1f %3.1f",roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
		//--- end design note ---
		
		//--- Madgwick AHRS : this doesn't work ---
		MadgwickAHRSupdate(gx,gy,gz,ax,ay,ay,mx,my,mz);
		//ROS_INFO("AHRS Quat: %1.3f %1.3f %1.3f %1.3f",q0,q1,q2,q3);
		double r = atan2(2.0*q2*q3-2.0*q0*q1, 2.0*q0*q0+2.0*q3*q3-1.0);
		double p = -asin(2.0*q1*q3+2.0*q0*q2);
		double y = atan2(2.0*q1*q2-2.0*q0*q3, 2.0*q0*q0+2.0*q1*q1-1.0);
		ROS_INFO("Mad RPY:  %3.1f %3.1f %3.1f",r*180.0/M_PI,p*180.0/M_PI,y*180.0/M_PI);
		//--- end Madgwick ---
    }
    
    
protected:
    
    // ROS connections for serial sim
    //ros::NodeHandle n;
    //ros::Publisher serial_pub;
    ros::Subscriber serial_sub;
    
    double ax,ay,az;
    double gx,gy,gz;
    double mx,my,mz;
    //double px,py,pz;
    //double 
    double roll,pitch,yaw;
    
};


///////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// 
// *****************************************************************************


int main( int argc, char** argv )
{
	ros::init(argc, argv, "scout");
	
	ros::NodeHandle n;
	ros::Rate r(100);
	
	ROS_INFO("... Start Simulation ...");
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	Navigation nav(n);
	
	ros::spin();
	
	
	ROS_INFO("End Simulation");
}


