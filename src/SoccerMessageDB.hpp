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
#include <sensor_msgs/MagneticField.h>        // Compass messages
#include <std_msgs/String.h> // for simulation

#include <iostream>
#include <string>
#include <string.h>
#include <math.h>

//#include <Eigen/Dense>

#include "kevin.h"
//#include "soccer/Imu.h"
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
	    
        reset();
	}
	
    void reset(){ // not sure this is useful in any way
        last_time = ros::Time::now();
    }
    
	tf::Quaternion magneticRPY(const sensor_msgs::MagneticField& mag, const sensor_msgs::Imu& imu){
		
		//--- from design note for tilt compensated compass ---
		double xm = mag.magnetic_field.x;
		double ym = mag.magnetic_field.y;
		double zm = mag.magnetic_field.z;
		double norm = sqrt(xm*xm+ym*ym+zm*zm);
		xm /= norm;
		ym /= norm;
		zm /= norm;
		
		double xa = imu.linear_acceleration.x;
		double ya = imu.linear_acceleration.y;
		double za = imu.linear_acceleration.z;
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
		
		// FIXME
		tf::Quaternion q; // = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,heading);
		//tf::createQuaternionMsgFromYaw(heading);
		
		return q;
	}
	
    bool getSensorData(sensor_msgs::Imu& imu, sensor_msgs::MagneticField& mag,
    		soccer::Battery& batt, unsigned int& drop){
    		
	    std::string data;
	    std::string s = "<s>";
	    
	    bool ok = getMessage(s,data);
	    if(!ok) return false;
	    
	    // FIXME[] check valid message format !!!
	    
        // ensure we have the right amount of data
        //if(data.size() != 20) return false;
        
        // copy each byte into buffer_t struct
        // 0 1  2   3..23 24
        // < s size  data >
        //for(unsigned int i=0;i<20;i++) mem.int8[i] = (byte) data[3+i];
        memcpy(&mem,&data[3],20);
        
        // now grab int16 out and stuff into memory
        // All of these are 2's complement numbers ... loose 1b to sign
	    // covert to g's, imu set to 2 g's max, 11b number (+-2047) 1b sign
	    imu.linear_acceleration.x = double(mem.int16[0])/2047.0*2.0;
	    imu.linear_acceleration.y = double(mem.int16[1])/2047.0*2.0;
	    imu.linear_acceleration.z = double(mem.int16[2])/2047.0*2.0;
	    
	    // covert to rads/sec, imu set to 250 dps max, 15b number (+-32767) 1b sign
	    imu.angular_velocity.x = double(mem.int16[3])*M_PI/180.0/32767.0*250.0;
	    imu.angular_velocity.y = double(mem.int16[4])*M_PI/180.0/32767.0*250.0;
	    imu.angular_velocity.z = double(mem.int16[5])*M_PI/180.0/32767.0*250.0;
	    
	    // covert to gauss, imu set to 1.3 gauss max, 11b number (+-2047) 1b sign
	    mag.magnetic_field.x = double(mem.int16[6])/2047.0*1.3;
	    mag.magnetic_field.y = double(mem.int16[7])/2047.0*1.3;
	    mag.magnetic_field.z = double(mem.int16[8])/2047.0*1.3;
	    
		// fixme:
		//imu.orientation = magneticRPY(mag,imu);
	    
	    batt.volts = double(mem.int16[9])*5.0/1023;
	    batt.amps = 0.2; // [FIXME] insert current meter
	    
	    ros::Duration dt = ros::Time::now() - last_time;
	    batt.power += batt.volts*batt.amps*dt.toSec()/3600.0;
	    last_time = ros::Time::now();
	    
	    return ok;
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
    
    
    
protected:    
    
    //unsigned short drop; // drop sensors
    
	//ros::Publisher imu_pub;
	//ros::Publisher battery_pub;
	//ros::Publisher ros_imu_pub;
    
    //sensor_msgs::Imu imu;
    //sensor_msgs::MagneticField mag;
    //sensor_msgs::Imu imu;
    //soccer::Battery batt;
    buffer_t mem; // this is a union I defined
	ros::Time last_time;
};

#endif