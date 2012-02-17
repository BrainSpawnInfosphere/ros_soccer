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
 * Change Log:
 *  6 Mar 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <sstream> // to concatonate c++ strings together
#include <iostream>
#include <string> // C++ for CerealPort
#include <queue> // C++ FIFO for Serial sim
#include <map> // messages


#include <ros/ros.h>
#include <std_msgs/String.h> // for simulation


//#include <cereal_port/CerealPort.h> // C++ serial port
#include "kSerial.h"


/**
 * Wrapper class around serial port code so it can be simulated if needed.
 */
class Serial {
public:

    #define SERIAL_BUFFER_SIZE 64

    Serial(bool s=false){
        sim = s;
        opened = false;
    }
    
    ~Serial(void){
        close();
    }
    
    
    bool write(const std::string& s){
        if(sim){
            std_msgs::String Str;
            Str.data = s;
            serial_pub.publish(Str);
            return true;
        }
        else {
            sp.write(s.c_str(),s.size());
        }
        
        return true;
    }
    
    int readBetween(std::string& data, char start, char end){
        int err = 0;
        
        if(sim){
            if(stack.empty()) err = -1;
            else {
                data = stack.front();
                stack.pop();
            }
        }
        else {
        /*
            bool ok = sp.readBetween(&data, start, end);
            if(ok) err = data.size();
            
            else */ err = -1;
        }
        
        if( err > 0 ) ROS_INFO("%s",data.c_str());
        
        return err;
    }
    
    
    int readBytes(std::string& data, int len){
        int err = 0;
        
        if(len > SERIAL_BUFFER_SIZE){
            ROS_ERROR("Need to increase Serial buffer length");
            return -1;
        }
        
        if(sim){
            if(stack.empty()) err = -1;
            else {
                data = stack.front();
                stack.pop();
                
                err = data.size();
            }
        }
        else {
            //ROS_INFO("readBytes");
            memset(buffer,0,SERIAL_BUFFER_SIZE);
            err = sp.readBytes(buffer, len);
            data.assign(buffer,len);
        }
        
        if( err > 0 ) ROS_INFO("%s",data.c_str());
        
        return err;
    }
    
    
    /**
     * Pass through to open serial port
     */
    bool open(const char* port, int baud){
        if(sim){
            init(); // setup publishers
        }
        else {            
            try{ sp.open(port, baud); }
            //catch(cereal::Exception& e){ return false; }
            catch(...){ return false; }
        }
        opened = true;
        return true;
    }
    
    /**
     * Pass through to close serial port
     */
    bool close(){
        if(sim);
        else {
            try{ sp.close(); }
            //catch(cereal::Exception& e){ return false; }
            catch(...){ return false; }
		}
		
		opened = false;
		
		return true;
    }
    
    inline const bool isOpened(){ return opened; }
    
    /**
     * Pass through to determine how many bytes are in the input buffer
     */
    unsigned int available(void){
        unsigned int avail = 0;
        if(sim){
            ; // fix me
            if(stack.empty()) return false;
            else {
                //data = stack.front();
                //stack.pop();
                
                //err = data.size();
                avail = (stack.front()).size();
            }
        }
        else {
            avail = sp.available();
        }
        
        return avail;
    }
    
    /**
     * Pass through to flush the input buffer
     */
    void flush(void){
        if(sim);
        else {
            sp.flush();
        }
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
    
    /**
     * Grabs a message from a serial port.
     * 1. get start char '<'
     * 2. gets message size (n) from database
     * 3. reads n bytes, return false if <e> is found
     * 4. get end char '>'
     * Basically return false if any error occurs
     * Note: the returned msg string only contains the message and
     * not the start/end chars (< or >).
     */
    bool getMessageFromSerial(char m, std::string& msg){
        int size = 0;
        bool ok = getMessageSize(m,size); 
        
        if(!ok) return false; // msg not found    
        
        if(sim){
            if(stack.empty()) return false;
            else {
                std::string data = stack.front();
                stack.pop();
                
                //err = data.size();
                unsigned int spos;
                
                spos = data.find_first_of('<') + 1;
                
                msg.assign(data,spos,size);
                ROS_INFO("Debug getMsg: %s",msg.c_str());
            }
        }
        else {
            ok = sp.readUntilChar('<');
            
            if(!ok) return false; // couldn't find start char
            memset(buffer,0,size); // only clear what we are using
            //buffer[0] = '<';
            //ROS_INFO("found <");
            
            char c = 0; /*
            ok = sp.readByte(c);
            //ROS_INFO("found 2: %c",c);
        
            if(!ok || m != c) return false; // wrong msg or bad read
            buffer[1] = c;
            ROS_INFO("good msg");
            */
            int cnt = 0;
            while(cnt < size){
                ok = sp.readByte(c);
                if(!ok) return false; // bad read
                if(cnt == 0) 
                    if(c == 'e'){
                        //ROS_INFO("Found error");
                        sp.readByte(c); // read end char '>'
                        return false;
                    }
                buffer[cnt++] = c; // change 1 -> 2 if add msg char
                //ROS_INFO("found %d: %c",cnt,c);
            }
            
            ok = sp.readByte(c);
            if(!ok || c != '>') return false; // bad read or didn't find end char
            //buffer[1+cnt++] = c;
            
            msg.assign(buffer,size);
            
        }
        
        return true;
    }
    
protected:
    
    void cmdReceived(const std_msgs::String::ConstPtr& msg){
        std::string data = msg->data;
        stack.push(data);
    }
    
    bool init(){
        ROS_INFO("Setting up pub/sub serial_to/fromRobot");
        
        // Publish ------------------------------------
        serial_pub = n.advertise<std_msgs::String>("/serial_toRobot", 50);
        
        // Subcriptions -------------------------------
        serial_sub  = n.subscribe<std_msgs::String>("/serial_fromRobot", 1, &Serial::cmdReceived,this);
        
        opened = true;
        return opened;
    }

    // for sim
    ros::Publisher serial_pub;
    ros::Subscriber serial_sub;
    ros::NodeHandle n;
    
    // real serial port code
    //cereal::CerealPort sp;
    kSerial sp;
    char buffer[SERIAL_BUFFER_SIZE];
    
    bool sim; // is this a simulation or real?
    bool opened;
    
    std::queue<std::string> stack; // stores sim messages
    std::map<char,int> messages;  // database of valid message sizes
    std::map<char,int>::iterator it; // database iterator
};

#endif