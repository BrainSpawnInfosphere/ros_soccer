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

#ifndef __KEVIN_H__
#define __KEVIN_H__

#include <sstream> // to concatonate c++ strings together
#include <iostream>
#include <string> // C++ for CerealPort


///////////////////////////////////////////////////////////////////////////////

namespace kevin {

    typedef unsigned char byte;
    typedef unsigned int uint;
    
    inline byte bit(byte a, byte b){ return (a & 1<<b); }
	
	template <typename T>
	inline T map(T x, T l1, T h1, T l2, T h2){
		T ans = (x-l1)*(h2-l2)/(h1-l1)+l2;
		return ans;
	}
	
	template <typename T>
    inline T limit(T x, T a, T b){
        return (x>b ? b : (x<a ? a : x));
    }
    
    template <typename T>
    inline T deadband(T x, T a, T b){
        return (x>b ? x : (x<a ? x : 0.0));
    }
    
    template <typename T>
    inline T sign(T a){
        return (a < 0.0 ? -1.0 : 1.0);
    }
    
    template <typename T>
    inline T in_to_mm(T in){
        return (in*25.4); // 25.4 mm per inch
    }
    
    template <typename T>
    inline T mm_to_in(T mm){
        return (mm/25.4); // 25.4 mm per inch
    }
    
    template <typename T>
    inline T degToRad(T angle){
        return (angle*M_PI/180.0);
    }
    
    template <typename T>
    inline T radToDeg( T rads ){ return rads*180.0/M_PI; }
    

}

#endif