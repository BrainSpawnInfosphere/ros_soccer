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
 * O   Publish TF for robot base
 * O   Publish TF for sensors
 * O   RVIZ model
 * O   Integrate new I2C IMU (accel, gyo, compass) for use with EFK
 *        imu - accel,gyro
 *        magnometer - heading
 * O   Enable timed events (beeps, lights) like wiimote
 * X   Enable simulation capability for serial
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
#include <math.h>

#include <Eigen/Dense>

#include "serial_node/serial.h"
#include "kevin.h"
#include "soccer/Imu.h"
#include "soccer/Battery.h"
#include "MadgwickAHRS/MadgwickAHRS.h"


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
#define ALL_SENSORS_SIZE 24
#define VERSION_SIZE 14

#define BATTERY_CAPACITY (4.8*1000) // mAhrs
#define BATTERY_VOLTAGE  4.8 // V

//------------------------//
#define ROS_LOOP_RATE_HZ 20

///////////////////////////////////////////////////////////////////////////////

// value???
class Sensor {
public:
    Sensor();
    
    double convert(const int a){
        double adc = 1023.0; // 10 bit
        double gain = 0.3*5.0; // 300 mg/V
        return ( gain*static_cast<double>(a)/adc );
    }
};


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

bool operator==(const geometry_msgs::Twist& a, const geometry_msgs::Twist& b){
    bool ans = false;
    
    ans = (a.linear.x == b.linear.x && a.linear.y == b.linear.y && a.linear.z == b.linear.z);
    ans = ans && (a.angular.x == b.angular.x && a.angular.y == b.angular.y && a.angular.z == b.angular.z);
     
    return ans;
}

/**
 * This is a simple db that maps message types to data sizes. This
 * might be over kill ... could just do some DEFINE's to achive
 * the same thing.
 */
class MessageDB {
public:
    MessageDB(){
        ;
    }
    
    void init(ros::NodeHandle n, std::string svc){
		client = n.serviceClient<serial_node::serial>(svc);
	}
    
    inline bool getMessage(std::string& str){
        std::string ans;
        return getMessage(str,ans);
    }
    
    bool getMessage(std::string& str, std::string& ans){
        
        //ROS_INFO("sending: %s",str.c_str());
        
        // get the message character
        char msg = str[1];
        
        int size = 0;
        
        // did we find the message and get its size?
        if(!getMessageSize(msg,size)) return false;
        //ROS_INFO("mdb: found msg size[%d]",size);
        
	    serial_node::serial srv;
        srv.request.str = str;
        srv.request.size = size;
        srv.request.time = 100; // how do i do this?
        
        // did we get a response?
        //if(!client.call(srv)) return false; // what does this return?
        client.call(srv);
        //ROS_INFO("mdb: call worked");
        //ROS_INFO("mdb: resp: %s",srv.response.str.c_str());
        
        // if expecting data return ... copy string
        if(size > 0) ans = srv.response.str;
        
        return true;
    }
    
    /**
     * Add message to database
     */
    void setMessage(char m, int size){
        messages[m] = size;
    }
    
    /**
     * Returns the message size or if message is not found, returns false
     */
    bool getMessageSize(char m, int& size){
        it = messages.find(m);
        
        if(it == messages.end()){
            size = 0;
            ROS_ERROR("Msg not found in database");
            return false;
        }
        
        size = it->second;
        
        return true;
    }

protected:
    std::map<char,int> messages;  // database of valid message sizes
    std::map<char,int>::iterator it; // database iterator
    
	ros::ServiceClient client;
};

inline int psign(const double d){
    return (d < 0.0 ? 0 : 1);
}

////////////////////////////////////////////////////////////////////
// cRobot is the hardware driver which handles all commands between 
// ROS and the robot.
////////////////////////////////////////////////////////////////////

class cRobot {
public:
	cRobot(ros::NodeHandle n, std::string svc) : phi(4,3){
		
		// setup simple defaults
		init(1.0,1.0,1.0, degToRad(45.0) );
		
		// setup message database
		mdb.setMessage('d',1);  // drop sensor
		mdb.setMessage('e',0);  // error
		mdb.setMessage('g',0);  // go
		mdb.setMessage('h',0);  // halt ... emergency stop
		mdb.setMessage('m',0);  // motor commands
		mdb.setMessage('s',ALL_SENSORS_SIZE); // sensors
		mdb.setMessage('r',0);  // reset
		mdb.setMessage('v',VERSION_SIZE); // version
		mdb.init(n,svc);
		
		// Publish --------------------------------
		imu_pub = n.advertise<soccer::Imu>("/imu", 50);
		battery_pub = n.advertise<soccer::Battery>("/battery", 50);
        ros_imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 50);
		
		// Services -------------------------------
		//client = n.serviceClient<serial_node::serial>("uc0_serial");
		
        // Subcriptions -------------------------------
        cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cRobot::cmdVelReceived, this);
	
	}
	
	~cRobot(void){
	    //if(sp.isOpened()) closeSerialPort();
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
		
		//std::cout<<phi<<std::endl;
		
		//exit(0);
	}
	
	/**
	 * Max rpm is ~70 (70*2*PI/60=7.3304 rads/sec), so scale numbers to that
	 */
	byte toPWM(const double d){
	    byte c = 0;
	    
	    const double maxpwm = 255.0;
	    //const double maxv = 7.3304; // need to figure out max ... this is wrong
	    
	    //if(d>maxv) ROS_ERROR("toPWM: input too high");
	    
	    //double scaled = maxpwm*fabs(d)/maxv;
	    double scaled = maxpwm*fabs(d);
	    
	    //std::cout<<d<<'\t'<<scaled<<std::endl;
	    
	    c = static_cast<byte>(scaled);
	    //c = (unsigned char) scaled;
	    
	    // dead band -- too small and the motor whine
	    c = (c < 40 ? 0 : c);
	    
	    return c;
	}
	
	/**
	 * Sends motor commands stored in desiredVelState to the robot.
	 *
	 * make pwm separate class/function
	 */
	bool sendControl(void){
	    bool ret = true;
	    Eigen::Vector4d m;
	    m = phi*desiredVelStates;
		
		if( m == motorVel ) return true;
		
		motorVel = m;
		
		//std::cout<<"desired: "<<desiredVelStates<<std::endl;
		//std::cout<<"ans: "<<phi*desiredVelStates<<std::endl;
		//std::cout<<"motorVel: "<<motorVel<<std::endl;
		//std::cout<<"sign 3: "<<(int)sign(motorVel(3))<<std::endl;
	    
	    // create message and send to uC
	    byte dir = 0 | (psign(motorVel(3))<<3 | psign(motorVel(2))<<2 | psign(motorVel(1))<<1 | psign(motorVel(0)) );
	    byte buffer[8];
	    buffer[0] = '<';
	    buffer[1] = 'm';
	    buffer[2] = dir;
	    buffer[3] = toPWM(motorVel(0));
	    buffer[4] = toPWM(motorVel(1));
	    buffer[5] = toPWM(motorVel(2));
	    buffer[6] = toPWM(motorVel(3));
	    buffer[7] = '>';
	    
	    //ROS_INFO("send cmd[%d]: %d %d %d %d", (int)buffer[2],
	    //    (int)buffer[3],(int)buffer[4],(int)buffer[5],(int)buffer[6]);
	    
	    std::string msg;
	    msg.assign((char*)buffer,8);
	    
	    ret = mdb.getMessage(msg);
	    
	    return ret;
	}
	
	void reset(){
	    std::string s("<r>");
	    mdb.getMessage(s);
    }
	
	// sensors
	bool getSensors(){
	    bool ok = true;
	    std::string data;
	    std::string s = "<s>";
	    
	    ok = mdb.getMessage(s,data);
	    if(!ok) ROS_ERROR("Error getting <s>");
	    
	    ok = memory.set(data);
	    //ROS_INFO("getSensors: %s",data.c_str());
	    
	    if(!ok) ROS_ERROR("Error formmatting data <s>");
	    
	    publishIMU();
	    publishBattery();
	    
	    return ok;
	    
	} 
    
    /**
     * Callback for receiving Twist messages from joystick or motion
     * planner.
     */
    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        //ROS_INFO("got it!!");
        
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
        
        //ROS_INFO("Got cmdVel: %g, %g, %g", x, y, z);
        
        //setState(x,y,z);
	    desiredVelStates(0) = x;
	    desiredVelStates(1) = y;
	    desiredVelStates(2) = z;
	    
		//std::cout<<phi<<std::endl;
		//std::cout<<desiredVelStates<<std::endl;
		//std::cout<<phi*desiredVelStates<<std::endl;
	    
	    //exit(0);
	    
	    previous_twist = *cmd_vel;
    }
    
    /**
     * Simple "is the robot ready" function ... cleans out serial buffer
     */
    bool ready(){
        std::string msg = "<v>";
        std::string ans;
        bool ok = mdb.getMessage(msg,ans);
        
        std::cout<<"\n+-------------------------------------------\n";
        std::cout<<"|   Robot ready? "<<(ok ? "yes " : "no ")<<std::endl;
        std::cout<<"\n+-------------------------------------------\n";
        
        return ok;
    }
    
    /**
     * The main robot loop
     */
    void loop(){
        ros::Rate r(ROS_LOOP_RATE_HZ);

        // Main Loop -- go until ^C terminates
        while (ros::ok()){
        
            //if(cnt++ > 50) ros::shutdown(); //exit(1);
            
            //ROS_INFO("loop");
            
            // Main loop functions
            bool ok = getSensors();
            if(!ok) ROS_ERROR("==[[ Couldn't get sensor data ]]==");
            //else ROS_INFO("got sensor data");
            
            //handleDanger(sensors); // if fault conditions are seen, safe robot immediately
            
            //navigation(sensors, nav);
            
            sendControl();

            ros::spinOnce();
            r.sleep();
        }
	}

protected:
	
	
	void publishIMU(){
		memory.imu.header.stamp = ros::Time::now();
		memory.imu.header.frame_id = "imu";
		
		imu_pub.publish(memory.imu);
		
		// ******************************************************************************************
		//publish IMU
		sensor_msgs::Imu imu;
		imu.header.stamp = ros::Time::now();
		imu.header.frame_id = "imu";
		
		//--- from design note for tilt compensated compass ---
		double xm = memory.imu.mags.x;
		double ym = memory.imu.mags.y;
		double zm = memory.imu.mags.z;
		double norm = sqrt(xm*xm+ym*ym+zm*zm);
		xm /= norm;
		ym /= norm;
		zm /= norm;
		
		double xa = memory.imu.accels.x;
		double ya = memory.imu.accels.y;
		double za = memory.imu.accels.z;
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
		imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,heading);
		imu.orientation_covariance[0] = 0.01; //xx;
		imu.orientation_covariance[4] = 0.01; //yy;
		imu.orientation_covariance[8] = 0.01; //zz;
		
		// gyro
		imu.angular_velocity.x = memory.imu.gyros.x;
		imu.angular_velocity.y = memory.imu.gyros.y;
		imu.angular_velocity.z = memory.imu.gyros.z;
		
		// from data sheet
		imu.angular_velocity_covariance[0] = 0.01; //xx;
		imu.angular_velocity_covariance[4] = 0.01; //yy;
		imu.angular_velocity_covariance[8] = 0.01; //zz;
		
		// accel
		imu.linear_acceleration.x = memory.imu.accels.x;
		imu.linear_acceleration.y = memory.imu.accels.y;
		imu.linear_acceleration.z = memory.imu.accels.z;
		
		// from data sheet +- 60 mg
		imu.linear_acceleration_covariance[0] = 0.06; //xx;
		imu.linear_acceleration_covariance[4] = 0.06; //yy;
		imu.linear_acceleration_covariance[8] = 0.06; //zz;
		
		ros_imu_pub.publish(imu);
    }
	
	void publishBattery(){
		memory.batt.header.stamp = ros::Time::now();
		memory.batt.header.frame_id = "battery";
		
		battery_pub.publish(memory.batt);
    }
	
	double mass;
	double inertia; 
	double radius; 
	double angle;
	
	Eigen::Vector3d desiredVelStates; // [vx vy radius*w]
	Eigen::Vector4d motorVel;
	Eigen::MatrixXd phi; // converts velocities to motor speeds matrix
	
	geometry_msgs::Twist previous_twist;
	ros::Publisher imu_pub;
	ros::Publisher battery_pub;
	//ros::ServiceClient client;
	ros::Subscriber cmd_vel_sub;
	ros::Publisher ros_imu_pub;
	
    SharedMemory memory;
    MessageDB mdb;
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
	
	//ros::NodeHandle n("~");
	ros::NodeHandle n;
	std::string svc_name;
	
     if(argc == 1) svc_name = "/uc0_serial";        
     else svc_name.assign(argv[1]);
	cRobot robot(n,svc_name);
	
    //setup robot
    double mass = 5.0;   // mass
    double radius = 0.30;  // radius
    double inertia = mass*radius*radius; // inertia		
    robot.init(mass,inertia,radius, degToRad(45.0) );
    
    // wait 1 sec to uC to get up and running
    bool ok = robot.ready();
    
    if(ok){
        ROS_INFO("ready");
    }
    else{
        ROS_FATAL("something wrong ... exiting");
        ROS_BREAK();
        return -1;
    }
    
    // setup RPY
    q0 = 1.0;
    q1 = q2 = q3 = 0.0;
	
	robot.loop();
}


				
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
