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
//#include <tf/transform_broadcaster.h>         // transforms
//#include <nav_msgs/Odometry.h>                // odometry
//#include <geometry_msgs/Twist.h>              // command and velocity
//#include <geometry_msgs/Point.h>              // servo motors
//#include <sensor_msgs/Imu.h>                  // IMU messages
#include <std_msgs/String.h> // for simulation

#include <iostream>
#include <string>
//#include <math.h>

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

public:    
    SoccerMessageDB(void){        
        last_time = ros::Time::now();
        
        reset();
	}
	
    void reset(){ // not sure this is useful in any way
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
    
    int drop;
    
//private:
    soccer::Imu imu;
    soccer::Battery batt;
    buffer_t mem;
	ros::Time last_time;
};

#endif