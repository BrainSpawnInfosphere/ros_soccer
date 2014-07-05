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

#ifndef __MESSAGE_DATABASE_H__
#define __MESSAGE_DATABASE_H__

#include <ros/ros.h>
#include <std_msgs/String.h> // for simulation
#include <ip_serial/serial.h>

#include <iostream>
#include <string> 




/**
 * This is a simple db that establishes a service connection to 
 * ip_serial and sends/receives strings back and forth.
 */
class MessageDB {
public:
    // Constructor ... does nothing
    MessageDB(){
        ;
    }
    
    // sets up the client service to a serial node
    // n - node handle created in main()
    // svc - name of ip_serial to connect too
    void init(ros::NodeHandle n, std::string svc){
		client = n.serviceClient<ip_serial::serial>(svc);
	}
	
	
	bool validMessageFormat(std::string m){
		if(m[0] != '<') return false;
		if(m[m.size()-1] != '>') return false;
	
		if(m.size() > 3) if((int)m[2]+4 != (int)m.size()) return false;
	
		return true;
	}
	
    
    // Send a message but get no response back
    // rename sendReply() or sendMsg()?
    //inline bool getMessage(std::string& str){
    //    std::string ans;
    //    return getMessage(str,ans);
    //}
    
    // Send a message and get a response back
    // str - message out
    // ans - reply back
    // sendReceiveMsg()?
    bool getMessage(std::string& str, std::string& ans){
        
        //ROS_INFO("sending: %s",str.c_str());
        if( !validMessageFormat(str) ) {
        	ROS_ERROR("invalid message format: %s", str.c_str());
        	return false;
        }
        
        // get the message character
        char msg = str[1];
        
        // did we find the message?
        // error printed inside function
        if(!validMessage(msg)) return false;
        
        
        //ROS_INFO("mdb: found msg size[%d]",size);
        
	    ip_serial::serial srv;
        srv.request.str = str;
        
        // call service
        client.call(srv);
        
        //ROS_INFO("mdb: call worked");
        //ROS_INFO("mdb: resp: %s",srv.response.str.c_str());
        
        ans = srv.response.str;
        
        return true;
    }
    
    /**
     * Add message to database
     * m - new message char
     * size - size of returned message back: 0-255
     */
    void setMessage(char m, int size){
        messages[m] = size;
    }

protected:    

	bool validMessage(const char m){
        std::map<char,int>::iterator it = messages.find(m);
        
        if(it == messages.end()){
            ROS_ERROR("validMessage(): Msg not found in database: %c", m);
            return false;
        }
        
        return true;
    }

    // change to std::vector
    std::map<char,int> messages;  // database of valid message sizes
    //std::map<char,int>::iterator it; // database iterator
    
	ros::ServiceClient client; // ROS client service
};


#endif