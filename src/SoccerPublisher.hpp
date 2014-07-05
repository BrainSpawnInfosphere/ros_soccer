

#ifndef __SOCCER_PUBLISHER_H__
#define __SOCCER_PUBLISHER_H__


#include <geometry_msgs/Twist.h>  // command and velocity

class SoccerPublisher {
public:
	SoccerPublisher(void){
		;
	}
	
	void init(ros::NodeHandle &n){
		// Publishers ---------------------------------
		imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
		mag_pub = n.advertise<sensor_msgs::MagneticField>("/magnatometer", 50);
		battery_pub = n.advertise<soccer::Battery>("/battery", 50);
		
        // Subcriptions -------------------------------
        //cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd0", 1, &cRobot::cmdVelReceived, this);
	
    }
	
	void publishMagneticField(sensor_msgs::MagneticField& mag){
		mag.header.stamp = ros::Time::now();
		mag.header.frame_id = "magnetic_field";
		
		mag_pub.publish(mag);
	}
	
	void publishIMU(sensor_msgs::Imu& imu){
		imu.header.stamp = ros::Time::now();
		imu.header.frame_id = "imu";
		
		imu_pub.publish(imu);
    }
	
	void publishBattery(soccer::Battery& batt){
		batt.header.stamp = ros::Time::now();
		batt.header.frame_id = "battery";
		
		battery_pub.publish(batt);
    }
    
    void publishTF(){
#if 0
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
	}
	
	
    /**
     * Callback for receiving Twist messages from joystick or motion
     * planner.
     *//*
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
        
        //ROS_INFO("cmdVelReceived(): %g, %g, %g", x, y, z);
        
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
    */

protected:
	//geometry_msgs::Twist previous_twist;
	ros::Subscriber cmd_vel_sub;
	
	ros::Publisher imu_pub;
	ros::Publisher battery_pub;
	ros::Publisher mag_pub;
};

#endif