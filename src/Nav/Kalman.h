/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Kevin J. Walchko.
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
 * Author: Kevin J. Walchko on 8/21/2011
 *********************************************************************
 *
 * Simple kalman filter using OpenCV 2.2 to track objects.
 *
 * Change Log:
 * 21 Aug 2011 Created
 * 19 Feb 2012 Modified to do navigation
 *
 **********************************************************************
 *
 * 
 *
 */


#ifndef __KALMAN_FILTER_NAV_H__
#define __KALMAN_FILTER_NAV_H__

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>


///////////////////////////////////////////////////////

/*

namespace kevin {

class EKF : public KalmanFilter {
    virtual void init(){
        kalman = new cv::KalmanFilter(x,y,0);
    }
    
    virtual cv::Mat& update(const cv::Point??& p){
        linearize(p,dt);
        Kalman::update(const cv::Point2f& p);
    }
    
protected:
    void linearize(cv::Pointx& p, double dt){
        //filter->transitionMatrix.at<double>(x,y) = ??
        
        int m = kf->transitionMatrix.rows;
        
        cv::Mat linstate(m);
        cv::Mat ta(m); // above
        cv::Mat tb(m); // below
        
        for(int i=0; i<size ;i++){
            linstate = p;
            linstate(i) += dt;
            ta = f(linstate,u,dist);
            
            linstate = p;
            linstate(i) -= dt;
            tb = f(linstate,u,dist);
            
            linstate = (ta-tb)/(2.0*dt);
            
            kf->transitionMatrix.setColumn(i,linstate);
        }   
    }
    
    cv::Point?& (*f)(cv::Point&, cv::Point&, cv::Point&);
};

} // end kevin namespace

*/

/**
 * Wraper class for cv::KalmanFilter. Creates a simple discrete filter.
 */
class Kalman
{
public:
    
    Kalman(unsigned int m, unsigned int n, float rate);
    //Kalman();
    ~Kalman() { delete filter; }
    
    //virtual void init()=0; // create dynamics here 
    void init(cv::Mat& Q, cv::Mat& R);
    
    //virtual cv::Mat& update(const cv::Point??& p)=0;
    void set(geometry_msgs::Pose& camera); // position estimate
    void set(geometry_msgs::Pose& pose); // imu
    void set(geometry_msgs::Wrench& wrench); // control forces/torques
    const cv::Mat& update(void);
    
protected:
    // why do i have N and M???
	//const unsigned int N; //! dimension of transition matrix: NxN
	//const unsigned int M; //! length of measurement vector
	
	cv::KalmanFilter* filter; // why pointer?
	
	// remove these
	unsigned int numActiveFrames; // not sure there is value in tracking these?
	unsigned int numInactiveFrames;
};





/**
 * There is a measurement, so update the predicted state, then correct the 
 * measurement with it.
 *
 * @param  DetectionRect&
 * @return statePost&
 */
const cv::Mat& Kalman::update()
{
	
	// Tracking center mass point
	cv::Mat measurement(M, 1, CV_32F);
	measurement.at<float>(0,0) = p.x;
	measurement.at<float>(1,0) = p.y;
	
	filter->predict();
	const cv::Mat& statePost = filter->correct(measurement);
	
	return statePost; // new estimate of state
}

/**
 * There is no measurement, return the predicted state
 *
 * @return statePre&
 */
const cv::Mat& Kalman::update(void)
{
	
	const cv::Mat& statePre = filter->predict();
	
	return statePre; // predicted state
}


/**
 * Kalman constructor, sets up defaults so the filter will run as is.
 * 
 * param M 
 * param N
 * param rate time constant
 */
Kalman::Kalman(unsigned int m, unsigned int n, float rate) /*: M(m), N(n)*/
{
	// this tracks the number of active and inactive frames
	numActiveFrames = 0;
	numInactiveFrames = 0;
	
	// setup kalman filter with a Model Matrix, a Measurement Matrix and no control vars
	filter = new cv::KalmanFilter(n, m, 0);
	
	// [1 0 dt 0]
	// [0 1 0 dt]
	// [0 0 1  0] = A
	// [0 0 0  1]
	
	// transitionMatrix is eye(n,n) by default
	filter->transitionMatrix.at<float>(0,2) = rate; // dt=0.04, stands for the time
	filter->transitionMatrix.at<float>(1,3) = rate; // betweeen two video frames in secs.
	filter->transitionMatrix.at<float>(0,0) = 1.0f;
	filter->transitionMatrix.at<float>(1,1) = 1.0f;
	filter->transitionMatrix.at<float>(2,2) = 1.0f;
	filter->transitionMatrix.at<float>(3,3) = 1.0f;
	
	// measurementMatrix is zeros(n,p) by default
	filter->measurementMatrix.at<float>(0,0) = 1.0f;
	filter->measurementMatrix.at<float>(1,1) = 1.0f;
	
	using cv::Scalar;
	
	// assign a small value to diagonal coeffs of processNoiseCov
	cv::setIdentity(filter->processNoiseCov, Scalar::all(1e-2)); // 1e-2
	
	// Measurement noise is important, it defines how much can we trust to the
	// measurement and has direct effect on the smoothness of tracking window
	// - increase this tracking gets smoother
	// - decrease this and tracking window becomes almost same with detection window
	cv::setIdentity(filter->measurementNoiseCov, Scalar::all(1e-1)); // 1e-1
	cv::setIdentity(filter->errorCovPost, Scalar::all(1));
	
	// Tracking center mass
	filter->statePost.at<float>(0,0) = 0.0f;
	filter->statePost.at<float>(1,0) = 0.0f;
	//filter->statePost.at<float>(2,0) = initRect.x2;
	//filter->statePost.at<float>(3,0) = initRect.y2;
}

/**
 * Sets the process and measurement noise for the KF instead of using the 
 * default values from the constructor.
 *
 * param Q process noise matrix
 * param R measurement noise matrix
 */
void Kalman::init(cv::Mat& Q, cv::Mat& R){
  filter->processNoiseCov = Q;
  filter->measurementNoiseCov = R;
}

#endif
