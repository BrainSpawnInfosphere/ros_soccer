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

#ifndef __SHARED_MEMORY_H__
#define __SHARED_MEMORY_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>         // transforms
//#include <nav_msgs/Odometry.h>                // odometry
//#include <geometry_msgs/Twist.h>              // command and velocity
//#include <geometry_msgs/Point.h>              // servo motors
#include <sensor_msgs/Imu.h>                  // IMU messages
#include <std_msgs/String.h> // for simulation

#include <iostream>
#include <string>
#include <math.h>

//#include <Eigen/Dense>

#include "kevin.h"
#include "soccer/Imu.h"
#include "soccer/Battery.h"
//#include "MadgwickAHRS/MadgwickAHRS.h"

#include "MessageDB.hpp"

using namespace kevin;

////////////////////////////////////////////////////////////////////
// SharedMemory is the generic repository of shared memory for the
// robot. It's primary function is to transform data from bytes (int8)
// into shorts (int16) and keep track of all data sent from the robot.
//
// \note not thread safe
////////////////////////////////////////////////////////////////////
class SoccerMessageDB : public MessageDB { 
public:
    typedef union {
        short int16[10];
        byte int8[20];
    } buffer_t;
    
    SoccerMessageDB(void){        
        
	}
	
	void init(ros::NodeHandle n, std::string svc){
	    MessageDB::init(n,svc);
	    
		// Publish --------------------------------
		imu_pub = n.advertise<soccer::Imu>("/imu", 50);
		battery_pub = n.advertise<soccer::Battery>("/battery", 50);
        ros_imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 50);
	    
        reset();
	}
	
    void reset(){ // not sure this is useful in any way
        last_time = ros::Time::now();
    }
    
    bool getSensorData(){
	    std::string data;
	    std::string s = "<s>";
	    
	    bool ok = getMessage(s,data);
	    if(!ok) ROS_ERROR("Error getting <s>");
	    
	    ok = setSensorData(data);
	    //ROS_INFO("getSensors: %s",data.c_str());
	    
	    if(!ok) ROS_ERROR("Error formmatting data <s>");
	    
	    return ok;
    }
        
    
    bool setSensorData(std::string& data){
        // ensure we have the right amount of data
        //if(data.size() != 20) return false;
        
        // copy each byte into buffer_t struct
        // 0 1  2   3..23 24
        // < s size  data >
        for(unsigned int i=0;i<20;i++) mem.int8[i] = (byte) data[3+i];
        
        // now grab int16 out and stuff into memory
	    imu.accels.x = double(mem.int16[0])/1023.0;
	    imu.accels.y = double(mem.int16[1])/1023.0;
	    imu.accels.z = double(mem.int16[2])/1023.0;
	    
	    imu.gyros.x = double(mem.int16[3])/1023.0;
	    imu.gyros.y = double(mem.int16[4])/1023.0;
	    imu.gyros.z = double(mem.int16[5])/1023.0;
	    
	    imu.mags.x = double(mem.int16[6])/1023.0;
	    imu.mags.y = double(mem.int16[7])/1023.0;
	    imu.mags.z = double(mem.int16[8])/1023.0;
	    
	    batt.volts = double(mem.int16[9])*5.0/1023;
	    batt.amps = 0.2; // [FIXME] insert current meter
	    
	    ros::Duration dt = ros::Time::now() - last_time;
	    batt.power += batt.volts*batt.amps*dt.toSec()/3600.0;
	    last_time = ros::Time::now();
	    
	    	
		//////////////////////////////////////////////////////////////////////////
#if 0
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = robot.nav.pos(0);
		odom_trans.transform.translation.y = robot.nav.pos(1);
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(robot.nav.pos(2));
		tf_broadcaster.sendTransform(odom_trans);
		
#endif
	    
	    return true;
	}
	
	// just a test, memory should read [256 1 256] if it
	// is working.
	void test(){
        mem.int8[0] = 0;
        mem.int8[1] = 1;
        mem.int8[2] = 1;
        mem.int8[3] = 0;
        mem.int8[4] = 0;
        mem.int8[5] = 1;
        ROS_INFO("Shared Memory Test [256 1 256]: %d %d %d",
                    mem.int16[0],
                    mem.int16[1],
                    mem.int16[2]);
        
        exit(0);
	}
    
    void publishIMU(){
		imu.header.stamp = ros::Time::now();
		imu.header.frame_id = "imu";
		
		imu_pub.publish(imu);
		
		// ******************************************************************************************
		//publish IMU
		sensor_msgs::Imu simu;
		simu.header.stamp = ros::Time::now();
		simu.header.frame_id = "imu";
		
		//--- from design note for tilt compensated compass ---
		double xm = imu.mags.x;
		double ym = imu.mags.y;
		double zm = imu.mags.z;
		double norm = sqrt(xm*xm+ym*ym+zm*zm);
		xm /= norm;
		ym /= norm;
		zm /= norm;
		
		double xa = imu.accels.x;
		double ya = imu.accels.y;
		double za = imu.accels.z;
		norm = sqrt(xa*xa+ya*ya*za*za);
		xa /= norm;
		ya /= norm;
		za /= norm;
		
		double pitch = asin(-xa);
		double roll;
		
		if(pitch == M_PI/2.0 || pitch == -M_PI/2.0) roll = 0.0;
		else roll = asin(ya/cos(pitch));
		
		double xh = xm*cos(pitch)+zm*sin(pitch);
		double yh = xm*sin(roll)*sin(pitch)+ym*cos(roll)-zm*sin(roll)*cos(pitch);
		double heading = atan2(yh,xh); // << not working right!!! [FIXME 20120312 kjw]
		
		ROS_INFO("RPY: %3.1f %3.1f %3.1f",roll*180.0/M_PI,pitch*180.0/M_PI,heading*180.0/M_PI);
		
		//--- end design note ---
		
		/* this doesn't work
		MadgwickAHRSupdate(gx,gy,gz,xa,ya,ya,xm,ym,zm);
		ROS_INFO("AHRS Quat: %g %g %g %g",q0,q1,q2,q3);
		double r = atan2(2.0*q2*q3-2.0*q0*q1, 2.0*q0*q0+2.0*q3*q3-1.0);
		double p = -asin(2.0*q1*q3+2.0*q0*q2);
		double y = atan2(2.0*q1*q2-2.0*q0*q3, 2.0*q0*q0+2.0*q1*q1-1.0);
		ROS_INFO("RPY: %g %g %g",r,p,y);
		*/
		

		// IMU orientation estimate -- fix this
		//imu.orientation = tf::createQuaternionMsgFromYaw(heading);
		simu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,heading);
		simu.orientation_covariance[0] = 0.01; //xx;
		simu.orientation_covariance[4] = 0.01; //yy;
		simu.orientation_covariance[8] = 0.01; //zz;
		
		// gyro
		simu.angular_velocity.x = imu.gyros.x;
		simu.angular_velocity.y = imu.gyros.y;
		simu.angular_velocity.z = imu.gyros.z;
		
		// from data sheet
		simu.angular_velocity_covariance[0] = 0.01; //xx;
		simu.angular_velocity_covariance[4] = 0.01; //yy;
		simu.angular_velocity_covariance[8] = 0.01; //zz;
		
		// accel
		simu.linear_acceleration.x = imu.accels.x;
		simu.linear_acceleration.y = imu.accels.y;
		simu.linear_acceleration.z = imu.accels.z;
		
		// from data sheet +- 60 mg
		simu.linear_acceleration_covariance[0] = 0.06; //xx;
		simu.linear_acceleration_covariance[4] = 0.06; //yy;
		simu.linear_acceleration_covariance[8] = 0.06; //zz;
		
		ros_imu_pub.publish(simu);
    }
	
	void publishBattery(){
		batt.header.stamp = ros::Time::now();
		batt.header.frame_id = "battery";
		
		battery_pub.publish(batt);
    }
    
protected:    
    
    int drop;
    
	ros::Publisher imu_pub;
	ros::Publisher battery_pub;
	ros::Publisher ros_imu_pub;
    
    soccer::Imu imu;
    soccer::Battery batt;
    buffer_t mem;
	ros::Time last_time;
};

#endif