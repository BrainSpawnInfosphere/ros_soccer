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

#include <iostream>
#include <sstream> // to concatonate c++ strings together
#include <string> // C++ for CerealPort

#include <Eigen/Dense>

#include "Serial.h"
#include "kevin.h"
#include "soccer/Imu.h"
#include "soccer/Battery.h"

using namespace kevin;

// Navigation -----------------------
#define DISTANCE_PER_CNT 5.4978f // mm/cnt 42*PI/24 = Dia*PI/CNT - linear distance
#define RADIANS_PER_CNT 0.0618424f // rads/cnt (42mm/3.5")*PI/24 = D/W*PI/CNT
#define TWO_PI 2.0*M_PI
#define EARTH_GRAVITY 9.81
#define AXLE_LENGTH 0.127 // 5 inches in meters

///////////////////////////////////////////////////////////////////////////////
//------------------------//
// Robot commands
#define CMD_START       'g' // <g>
#define CMD_HALT        'h' // <h>
#define CMD_MOTOR       'm' // <md0123> d=dir motor_number=0123
#define CMD_PLAY_SOUND  'p' // <px> x=sound number
#define CMD_RESET       'r' // <rx> x=battery(b), errors(e)
#define CMD_SENSORS     's' // <s>
#define CMD_VERSION     'v' // <v>

//------------------------//
// Size of data returns
#define ALL_SENSORS_SIZE 20

#define BATTERY_CAPACITY (4.8*1000) // mAhrs
#define BATTERY_VOLTAGE  4.8 // V

//------------------------//
#define ROS_LOOP_RATE_HZ 20

///////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////
// Navigation will transform the IMU (and other) sensor's inofrmation
// into position, velocity, and orientation of the robot. It will 
// contain the kalman filter which blends both the noisey data 
// measurements and the current estimate into the next future estimate.
//
// \note uses OpenCV kalman filter
// \note not thread safe
////////////////////////////////////////////////////////////////////
/*
class Navigation {
public:
    Navigation(){;}
    void updatePosition(double dt){
        pos += vel*dt;
    }
    
    void reset(){
        pos.setZero();
        vel.setZero();
    }
    
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d accels;
    Eigen::Vector3d gyros;
    Eigen::Vector3d mags;
};

*/

////////////////////////////////////////////////////////////////////
// SharedMemory is the generic repository of shared memory for the
// robot. It's primary function is to transform data from bytes (int8)
// into shorts (int16) and keep track of all data sent from the robot.
//
// \note not thread safe
////////////////////////////////////////////////////////////////////
class SharedMemory {
public:
    typedef union {
        short int16[10];
        byte int8[20];
    } buffer_t;

public:    
    SharedMemory(void){        
        last_time = ros::Time::now();
        
        reset();
	}
	
    void reset(){ // not sure this is useful in any way
    }
    
    bool set(std::string& data){
        // ensure we have the right amount of data
        if(data.size() != 20) return false;
        
        // copy each byte into buffer_t struct
        for(unsigned int i=0;i<data.size();i++) mem.int8[i] = (byte) data[i];
        
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
    
    //double battV; // V
    //double battA; // mA
    //double power; // Whr
    int drop;
    
//private:
    soccer::Imu imu;
    soccer::Battery batt;
    buffer_t mem;
    //Navigation nav;
	ros::Time last_time;
};

bool operator==(const geometry_msgs::Twist& a, const geometry_msgs::Twist& b){
    bool ans = false;
    
    ans = (a.linear.x == b.linear.x && a.linear.y == b.linear.y && a.linear.z == b.linear.z);
    ans = ans && (a.angular.x == b.angular.x && a.angular.y == b.angular.y && a.angular.z == b.angular.z);
     
    return ans;
}

////////////////////////////////////////////////////////////////////
// cRobot is the hardware driver which handles all commands between 
// ROS and the robot.
////////////////////////////////////////////////////////////////////

class cRobot {
public:
	cRobot(ros::NodeHandle n, bool s=false) : phi(4,3), sp(s){
		
		// setup simple defaults
		init(1.0,1.0,1.0, degToRad(45.0) );
		
		// setup message database
		sp.setMessage('s',ALL_SENSORS_SIZE); // sensors
		sp.setMessage('e',0);  // error
		sp.setMessage('v',20); // version
		sp.setMessage('g',0);  // go
		sp.setMessage('h',0);  // halt ... emergency stop
		sp.setMessage('d',1); // drop sensor
		
		imu_pub = n.advertise<soccer::Imu>("/imu", 50);
		battery_pub = n.advertise<soccer::Battery>("/battery", 50);
	}
	
	~cRobot(void){
	    if(sp.isOpened()) closeSerialPort();
    }
	
	void init(double m, double i, double r, double a){
		angle = a;   // wheel angle (rads)
		mass = m;    // mass (kg)
		inertia = i; // robot inertia (kg-m^2)
		radius = r;  // robot body radius (m)
		
		desiredVelStates.setZero();
		motorVel.setZero();
		
		// setup coupling matrix
		phi << -sin(angle), cos(angle), 1.0,
		       -sin(M_PI-angle), cos(M_PI-angle), 1.0,
		       -sin(M_PI+angle), cos(M_PI+angle), 1.0,
		       -sin(2.0*M_PI-angle), cos(2.0*M_PI-angle), 1.0;
	}
	
	/**
	 * Max rpm is ~70 (70*2*PI/60=7.3304 rads/sec), so scale numbers to that
	 */
	byte toPWM(const double d){
	    byte c = 0;
	    
	    const double maxpwm = 255.0;
	    const double maxv = 7.3304; // need to figure out max ... this is wrong
	    
	    if(d>maxv) ROS_ERROR("toPWM: input too high");
	    
	    double scaled = maxpwm*abs(d)/maxv;
	    
	    c = static_cast<byte>(scaled);
	    
	    return c;
	}
	
	/**
	 * Sends motor commands stored in desiredVelState to the robot.
	 *
	 * make pwm separate class/function
	 */
	bool sendControl(void){
	    bool ret = true;
	    motorVel = phi*desiredVelStates;
	    
	    // create message and send to uC
	    byte dir = 0 | ((int)sign(motorVel(3))<<3 | (int)sign(motorVel(2))<<2 | (int)sign(motorVel(1))<<1 | (int)sign(motorVel(0)) );
	    byte buffer[8];
	    buffer[0] = '<';
	    buffer[1] = 'm';
	    buffer[2] = dir;
	    buffer[3] = toPWM(motorVel(0));
	    buffer[4] = toPWM(motorVel(1));
	    buffer[5] = toPWM(motorVel(2));
	    buffer[6] = toPWM(motorVel(3));
	    buffer[7] = '>';
	    
	    std::string str;
        str.assign((char*)buffer,8);
	    sp.write(str);
	    
	    usleep(1000); // wait for 1 msec
	    
	    sp.flush(); // shouldn't have to do this!
	    
	    /*
	    std::string error;
	    bool ok = sp.getMessageFromSerial('e',error);
	    while(ok){
	        ok = sp.getMessageFromSerial('e',error);
	        ROS_INFO("control: got error");
	        ret = false;
	    }
	    */
	    return ret;
	}
	
	void reset(){ sp.write("<r>"); }
	
	/**
	 * Waits for a defined number of bytes before returning true. If
	 * serial.available() fails missCnt number of times, then the 
	 * funtion returns false.
	 *
	 * move to serial
	 */
	bool waitForData(unsigned int size, int missCnt=30){
	    int cnt = 0;
	    bool ok = true;
	    while( sp.available() < size ){
	        if(cnt++ > missCnt){
	            ok = false;
	            break; // not sure how big to make this??
	        }
	        usleep(100);
	    }
	    
	    return ok;
	}
	
	// sensors
	bool getSensors(){
	    bool ok = true;
	    
	    std::string str("<s>");
	    sp.write(str);
	    waitForData(23); // wait for at least 3 bytes in input buffer
	    
	    std::string data;
	    
	    ok = sp.getMessageFromSerial('s',data);
	    
	    if(!ok){
	        sp.flush();
	        return false;
	    }
	    
	    memory.set(data);
	    
	    publishIMU();
	    publishBattery();
	    
	    return ok;
	    
	}
	
	// serial comm pass through to open the serial port
	bool openSerialPort(std::string &port_name, int baud){
    
        try{ sp.open(port_name.c_str(), baud); }
        //catch(cereal::Exception& e){ return false; }
        catch(...){ return false; }
        
        return true;
	}
        
        
    /**
     * Callback for receiving Twist messages from joystick or motion
     * planner.
     */
    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        // do we have this command already?
        geometry_msgs::Twist a = *cmd_vel;
        if(previous_twist == a) return;
        
        // what should these limits be?
        double x = limit(cmd_vel->linear.x, -1.0, 1.0);
        double y = limit(cmd_vel->linear.y, -1.0, 1.0);
        double z = limit(cmd_vel->angular.z, -1.0, 1.0);
        
        
        //const double band = 1.0;
        //x = deadband(x,-band,band);
        //y = deadband(y,-band,band);
        
        ROS_INFO("Got cmdVel: %g, %g, %g", x, y, z);
        
        //setState(x,y,z);
	    desiredVelStates(0) = x;
	    desiredVelStates(1) = y;
	    desiredVelStates(2) = z;
	    
	    previous_twist = *cmd_vel;
    }
    
    bool ready(){
        
        for(int i=0;i<1000;i++){
            if(sp.available() >= 3) break;
            usleep(3000);
        }
        
        std::string msg;
        bool ok = sp.getMessageFromSerial('g',msg);
        
        return ok;
    }

protected:
	
	bool closeSerialPort(){
	    std::string msg("<h>");
	    ROS_INFO("Closed serial port");
		sp.write(msg); // send stop command	
		sp.close();
		return true; 
	}
	
	void publishIMU(){
		memory.imu.header.stamp = ros::Time::now();
		memory.imu.header.frame_id = "imu";
		
		//float a = (float) nav.gyaw * 180.0/M_PI;
		
		imu_pub.publish(memory.imu);
    }
	
	void publishBattery(){
		memory.batt.header.stamp = ros::Time::now();
		memory.batt.header.frame_id = "battery";
		
		//float a = (float) nav.gyaw * 180.0/M_PI;
		
		battery_pub.publish(memory.batt);
    }
	
	double mass;
	double inertia; 
	double radius; 
	double angle;
	Eigen::Vector3d desiredVelStates; // [vx vy radius*w]
	Eigen::Vector4d motorVel;
	Eigen::MatrixXd phi; // converts velocities to motor speeds matrix
	Serial sp; 
	
	geometry_msgs::Twist previous_twist;
	ros::Publisher imu_pub;
	ros::Publisher battery_pub;
	
    SharedMemory memory;
};

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
// The main code is composed of basic ROS stuff to get things
// setup and going.
////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
	ros::init(argc, argv, "soccer");
	
	ros::NodeHandle n;
	ros::Rate r(ROS_LOOP_RATE_HZ);
	
	cRobot robot(n,false);
	
    //setup robot
    double mass = 5.0;   // mass
    double radius = 0.30;  // radius
    double inertia = mass*radius*radius; // inertia		
    robot.init(mass,inertia,radius, degToRad(45.0) );
    
    /* Command line work TODO
     - serial port
     - run sim (T/F)
     - debug (T/F)
     */
     std::string port;
     
     if(argc < 2) port.assign("/dev/cu.usbserial-A7004IPE");        
     else port.assign(argv[1]);
    
    bool ok = robot.openSerialPort(port,115200);
    
    if(ok){
        ROS_INFO("Opened serial port [%s]",port.c_str());
    }
    else{
        ROS_FATAL("Couldn't open serial port [%s]",port.c_str());
        ROS_BREAK();
        return -1;
    }
    
    // wait 1 sec to uC to get up and running
    ok = robot.ready();
    
    if(ok){
        ROS_INFO("ready");
    }
    else{
        ROS_FATAL("something wrong ... exiting");
        ROS_BREAK();
        return -1;
    }
	
	///////////////////////////////////////////////
	
	/* TODO pub/sub
	 - imu
	 - sensors
	 X twist command
	 */
	
	// Publish ------------------------------------
	//ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	
	// Publish transforms -------------------------
	//tf::TransformBroadcaster tf_broadcaster;
	
	// Subcriptions -------------------------------
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cRobot::cmdVelReceived, &robot);
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	// Main Loop -- go until ^C terminates
	while (ros::ok()){
	
	    //if(cnt++ > 50) ros::shutdown(); //exit(1);
	    
	    //ROS_INFO("loop");
	    
		// Main loop functions
		ok = robot.getSensors();
		if(!ok) ROS_ERROR("Couldn't get sensor data");
		//else ROS_INFO("got sensor data");
		
		//handleDanger(sensors); // if fault conditions are seen, safe robot immediately
		
		//navigation(sensors, nav);
		
		robot.sendControl();
				
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


		ros::spinOnce();
		r.sleep();
	}
}


