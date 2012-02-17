/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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
* Author: Gonçalo Cabrita on 19/05/2010
*********************************************************************/
#ifndef __K_SERIAL_H__
#define __K_SERIAL_H__

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/ioctl.h>
//#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
//#include <unistd.h>
#include <fcntl.h>
//#include <stdio.h>
//#include <string.h>
#include <stdexcept>   // for exception, runtime_error, out_of_range

class kException : public std::runtime_error {
public: 
	kException(const char *msg) : std::runtime_error(msg) {}
};


#define __K_SERIAL_DEBUG__ 0

class kSerial {
public:
	kSerial(){
		fd = 0;
	}
	
	~kSerial(){
		if (fd > 0) close();
	}
	
	
	bool readByte(char& c){
		
		int n = ::read (fd, &c, 1);
		
		//if (n < 0) throw kException("Failed to read byte");
		if(n < 0) return false;
		
		return true;
	}		
	
	/**
	 * Move through serial buffer until findChar is found. If numMisses
	 * failed reads occur, then the function fails.
	 */
	bool readUntilChar(char findChar, int numMisses=10){
	    char c = -1;
	    int cnt = 0;
	    
	    while( c != findChar ){
	        if(cnt++ > numMisses) return false;
	        if(!readByte(c)) return false;
            //ROS_INFO("found 1: %c",c);
	    }
	    
	    return true;
	}
	
	/**
	 * Read a specific number of bytes and put them into a buffer
	 * \note readBytes() will attempt to read the serial port several
	 *       times before failing
	 * \return number of bytes read
	 */
	int readBytes(char *buf, const int numbytes, const int trys=3){
		int i, numread = 0, n = 0, numzeroes = 0;
		
		while (numread < numbytes){
		    //printf(".\n");
			n = ::read (fd, (buf + numread), (numbytes - numread));
			if (n < 0)
				return -1;
			if (0 == n){
				numzeroes++;
				if (numzeroes > trys)
					break;
			}
			numread += n;
		}
		
		if (__K_SERIAL_DEBUG__){
			printf ("Read:   ");
			for (i = 0; i < numread; i++)
				printf ("%d ", buf[i]);
			printf ("\nRead %d of %d bytes\n", numread, numbytes);
		}
		
		tcflush (fd, TCIFLUSH);			//discard data that was not read
		return numread;
	}
	
	/**
	 * Write a specific number of bytes from a buffer
	 * \note write() will attempt to read the serial port several
	 *       times before failing
	 * \return number of bytes written
	 */
	int write(const char* buf, const int numbytes, const int trys=3){
		int i, numwritten = 0, n = 0, numzeroes = 0;
		
		//write (fd, (buf + numwritten), (numbytes - numwritten));
    
        while (numwritten < numbytes){
            n = ::write (fd, (buf + numwritten), (numbytes - numwritten));
            
            if (n < 0) return -1;
            
            if (0 == n){
                numzeroes++;
                
                if (numzeroes > trys) break;
            }
            numwritten += n;
        }
		
		if (__K_SERIAL_DEBUG__){
			printf ("cwrite[%d]: ", numbytes);
			for (i = 0; i < numbytes; i++) printf ("%d ", buf[i]);
			printf ("\n");
		}
		
		return numwritten;
	}
	
	void close(){
		if( ::close(fd) < 0){
			throw kException("Failed to close port");
		}
		fd = 0;
	}
	
	bool open(const char *port, const int baud){
		//int speed;
		char msg[70];
		struct termios options;
		
		fd = ::open( port, O_RDWR | O_NOCTTY | O_NDELAY /*| O_NONBLOCK */ ); //may not need the nodelay
		if( fd <= 0 ){
			sprintf(msg,"ERROR: Unable to open port %s:%d - %s",port,baud,strerror(errno));
			//perror(msg);
			//return(-1);
			//throw kException(msg);
			return false;
		}
		/*
		fcntl (fd, F_SETFL, 0);
		tcflush (fd, TCIOFLUSH);
		
		//get config from fd and put into options
		tcgetattr (fd, &options); 
		//give raw data path
		cfmakeraw (&options);
		//set baud
		//cfsetispeed (&options, B57600); // create			
		//cfsetospeed (&options, B57600); // create
		//cfsetispeed (&options, B115200);			
		//cfsetospeed (&options, B115200);
		cfsetspeed(&options, baud);
		//send options back to fd
		tcsetattr (fd, TCSANOW, &options);
		*/
		
		//fcntl (fd, F_SETFL, 0);
		fcntl(fd, F_SETFL, FNDELAY); // return if nothing there to read
		//tcflush (fd, TCIOFLUSH);
		
		//get config from fd and put into options
		tcgetattr (fd, &options); 
		
		// Enable the receiver and set local mode
		options.c_cflag |= (CLOCAL | CREAD);
		//options.c_cflag = (CLOCAL | CREAD);
		//options.c_cflag &= ~CRTSCTS; // Disable hardware flow control 
		//options.c_cflag |= CRTSCTS; // Enable hardware flow control 
		
		// timeout
		//options.c_cc[VMIN]  = 0; // 0 char
		//options.c_cc[VTIME] = 1; // .1 sec
		
		// set 8N1
		//options.c_cflag &= ~PARENB;
		//options.c_cflag &= ~CSTOPB;
		//options.c_cflag &= ~CSIZE;
		//options.c_cflag |= CS8;
		
		
		// set SW Flow Control    
		//options.c_iflag |= (IXON | IXOFF | IXANY);
		
		
		//give raw data path
		cfmakeraw (&options);
		
		//set baud
		cfsetspeed(&options, baud);
		
		//send options back to fd
		tcsetattr (fd, TCSANOW, &options);
		return true;
	}
	
	/**
	 * Determine the number of bytes in the input buffer without the need to 
	 * read it.
	 */
	unsigned int available(void){
		int bytes = 0;
		ioctl(fd, FIONREAD, &bytes);
		return (unsigned int)bytes;
	}
	
	void flush(void){ //discard data that was not read
		tcflush (fd, TCIFLUSH);
	}
	
protected:
	int fd;
};


#endif