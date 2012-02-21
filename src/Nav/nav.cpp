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

#include "kevin.h"

using namespace kevin;

///////////////////////////////////////////////////////////////////////////////

//typedef unsigned int uint;

class cRK4 {
public:

	cRK4(uint x_size, uint y_size) : m_x(x_size), m_u(y_size) {
        xx.resize(x_size);
        k1x.resize(x_size);
        k2x.resize(x_size);
        k3x.resize(x_size);
        k4x.resize(x_size);
        
        cRK4::reset();
    }
    
	virtual ~cRK4(void){;}
	
	virtual Eigen::VectorXd& eqns(Eigen::VectorXd& x, Eigen::VectorXd& u,double time, double dt) = 0;
	
	
	Eigen::VectorXd& integrate(double time, double dt){
        
        xx = m_x;
        k1x=eqns(xx,m_u,time,0.0); 
        xx = m_x+.5*dt*k1x;
        k2x=eqns(xx,m_u,time+.5*dt,.5*dt); 
        xx = m_x+.5*dt*k2x;
        k3x=eqns(xx,m_u,time+.5*dt,.5*dt);
        xx = m_x+dt*k3x;
        k4x=eqns(xx,m_u,time+dt,dt);
        m_x = m_x + (dt/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
        
        //time += dt;
        
        return m_x;
    }

	virtual void reset(void){
        k1x.setZero();
        k2x.setZero();
        k3x.setZero();
        k4x.setZero();
        xx.setZero();
        m_x.setZero();
        m_u.setZero();
        //time = 0.0;
    }
	
	inline Eigen::VectorXd& getState(void){return m_x;}
	inline void setState(const Eigen::VectorXd& x){m_x = x;}
	
	inline Eigen::VectorXd& getControl(void){return m_u;}
	inline void setControl(const Eigen::VectorXd& u){m_u = u;}
	
private:
	Eigen::VectorXd k1x,k2x,k3x,k4x,xx;
	Eigen::VectorXd m_x, m_u;
};

//------------------------//
#define ROS_LOOP_RATE_HZ 30

//------------------------//


/**
 * state = [ x y theta xd yd w ]
 * 
 * phi = [0 0 0 1 0 0]
 *       [0 0 0 0 1 0]
 *       [0 0 0 0 0 1]
 *       [ ... 0 ... ]
 *       [ ... 0 ... ]
 *       [ ... 0 ... ]
 *
 * B = [
 */
class Robot : public cRK4 {
public:
    Robot() : cRK4(3,4), A(3,3), B(3,4), pos(3), vel(3), u(4), R_bw(3,3), inv_M(3,3){
        init();
        
        // Publish ------------------------------------
        serial_pub = n.advertise<std_msgs::String>("/serial_fromRobot", 50);
        
        // Subcriptions -------------------------------
        serial_sub  = n.subscribe<std_msgs::String>("/serial_toRobot", 1, &Robot::cmdReceived, this);
	
    }
    
    bool init(){
        
        const double p = degToRad(45.0); // wheel angle
        const double M = 5; // robot mass
        const double R = in_to_mm(7.0/2.0)/1000.0; // radius robot
        const double m = .5; // wheel+stator mass
        const double r = in_to_mm(2.5/2.0)/1000.0; // wheel radius
        const double Jw = m*r*r/2.0; // inertia wheel
        const double Jr = M*R*R/2.0; // inertia robot
        const double a = .5; // wheel viscous friction factor
        const double k = .5; // driving gain of motor
      
#if 0
        Eigen::MatrixXd one(3,3), gamma(3,4), beta(3,3), eye(3,3);
        
        eye << 1,0,0, 0,1,0, 0,0,1;
        
        one << sin(p), 0, 0,
               0, cos(p), 0,
               0, 0, R;
        
        gamma << -1, -1, 1, 1,
                  1, -1, -1, 1,
                  1, 1, 1, 1;
        
        beta << 1/M, 0, 0,
                0, 1/M, 0,
                0, 0, 1/Jr;
        
        u.setZero();
        setControl(u);
        
        inv_M = eye + Jw/(r*r)*beta*one*one*4.0;
        inv_M = inv_M.inverse();
        
        B = k/r*beta*one*gamma;
        
        A = a/(r*r)*beta*one*one;
#else

        Eigen::MatrixXd DC(3,3), inv_M(3,3), Mass(3,3), D(3,4), one(3,3), gamma(3,4);
        
        one << sin(p), 0, 0,
               0, cos(p), 0,
               0, 0, R;
        
        gamma << -1, -1, 1, 1,
                  1, -1, -1, 1,
                  1, 1, 1, 1;
        
        D = one*gamma;
        
        DC = 4.0*one*one;
              
        A = a/(r*r)*DC;
        
        B = k/r*D;
              
        Mass << M, 0, 0,
                0, M, 0,
                0, 0, Jr;
                
        inv_M = Mass + Jw/(r*r)*DC;
        inv_M = inv_M.inverse();
        
        u.setZero();
        setControl(u);
#endif
#if 0 
        std::cout<<"\nA = "<<A<<"\nB = "<<B<<"\nM = "<<inv_M<<std::endl;
        std::cout<<"\n SS A = "<<inv_M*A<<"\nB = "<<inv_M*B<<std::endl;
        exit(0);
#endif

        pos.setZero();
        vel.setZero();
        
        return true;
    }
    
    virtual Eigen::VectorXd& eqns(Eigen::VectorXd& x, Eigen::VectorXd& u,double time, double dt){
        static Eigen::VectorXd out(3);
        
        out = inv_M*(B*u - A*x);
        
        return out;
	}
	
	void getWorldState(Eigen::VectorXd& p, Eigen::VectorXd& v){
	    // simple stupid integration
	    // need to calc rotation from body to world
	    double t = pos(2); 
	    
	    R_bw << cos(t), -sin(t), 0,
	            sin(t), cos(t), 0,
	            0, 0, 1;
	            
	    p = R_bw*pos;
	    v = R_bw*vel;
	    
	    //p = pos;
	    //v = vel;
	}
	
    bool parseSerial(const std::string& s){
        bool ret = false;
        std_msgs::String msg;
        
        if(s[0] == '<'){
            byte dir = 0;
            ret = true;
    
            switch(s[1]){
                case 'h': // halt
                    u.setZero();
                    setControl(u);
                    ROS_INFO("HALT");
                    break;
                case 'm': // motor command
                    dir = (byte)s[2];
                    for(int i=0;i<4;i++) u(i) = (double)(bit(dir,i))*((double)static_cast<byte>(s[3+i]))/255.0;
                    setControl(u);
                    
                    std::cout<<u<<std::endl;
                    
                    ROS_INFO("control");
                    
                    break;
                case 'r': // reset system
                    init();
                    break;
                case 's':
                    printState();
                    
                    //usleep(1000);
                    
                    // < Batt Current >
                    msg.data.assign("< a b c >");
                    serial_pub.publish(msg);
                    
                    break;
                default:
                    ret = false;
            }
        }
        
        return ret;
    }
    
	Eigen::VectorXd& integrate(double time, double dt){
	    vel = cRK4::integrate(time,dt);
	    /*
	    // simple stupid integration
	    // need to calc world orientation so we can calc
	    // the rotation from body to world for position
	    double t = pos(2) + vel(2)*dt; 
	    
	    R_bw << cos(t), -sin(t), 0,
	            sin(t), cos(t), 0,
	            0, 0, 1;
	            
	    pos = pos + R_bw*vel*dt;
	    pos(2) = t; // fix this, it got messed up from above??
	    */
	    
	    // calculate pos in body frame ... never use this directly, convert
	    // to world frame first!
	    pos = pos + vel*dt;
	    
	    return vel; // note these velocities are body frame
	}
    
    //const Eigen::VectorXd& getState(void){ return state; }
    void printState(void){
        Eigen::VectorXd p(3), v(3);
        
        getWorldState(p,v);
        
        std::cout<<"Pos ["<<p(0)<<" "<<p(1)<<" "<<p(2)<<"]"<<std::endl;
        std::cout<<"Vel ["<<v(0)<<" "<<v(1)<<" "<<v(2)<<"]"<<std::endl;
    }    
    
protected:

    
    void cmdReceived(const std_msgs::String::ConstPtr& msg){
        if(msg->data.size() > 2){
            //ROS_INFO("cmdReceived: %s",msg->data.c_str());
            parseSerial(msg->data.c_str());
        }
        else {
            ROS_INFO("cmdReceived Error: %s",msg->data.c_str());
        }
    }

    //int pwm[4];
    Eigen::VectorXd u;
    Eigen::MatrixXd A,B,inv_M, R_bw;
    Eigen::VectorXd pos, vel;
    
    // ROS connections for serial sim
    ros::NodeHandle n;
    ros::Publisher serial_pub;
    ros::Subscriber serial_sub;
    
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
	
	
    Robot robot;
	
	#if 1
	Eigen::VectorXd u(4);
	u << 0.1, 0.1, 0.1, 0.1;
	robot.setControl(u);
	#endif
	
	int cnt = 0;
	
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
		// Main loop functions
		
		if(cnt++ > 30) exit(0);
		
		current_time = ros::Time::now();
		
		robot.integrate(current_time.toSec(),(current_time - last_time).toSec());
		
		last_time = current_time;
		
		robot.printState();
		
        //std::cout<<'['<<robot.getControl()<< ']'<<std::endl;
		
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("End Simulation");
}


