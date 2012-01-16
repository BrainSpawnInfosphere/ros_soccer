
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <Arduino.h> 
#include <inttypes.h>

//< a.x a.y a.z g.x g.y g.z ref m.x m.y m.z batt > = 22 bytes + 2 start/end = 24 
#define STATUS_LEN 24

/**
 * The buffer holds the message
 */
class cSensor {
public:

  cSensor() {
    ;
  }

  void packBytes(byte *m, int v){
    m[0] = ((byte)(v>>8));      // send hi
    m[1] = ((byte)(v & 0xff));  // send low
  }

  void packBytes(byte *m, unsigned long v){
    m[0] = ((byte)(v>>24 & 0xff));      // send hi
    m[1] = ((byte)(v>>16 & 0xff));      // send hi med
    m[2] = ((byte)(v>>8 & 0xff));       // send low med
    m[3] = ((byte)(v & 0xff));          // send low
  }

  /**
   * Sends the sensor data structure as a series of bytes. It also
   * formats it with the proper start/end message chars.
   * \param sensors
   * FIXME change to writeBuffer(buff,size) and use memcpy() to fill buffer 01092011 KJW
   */
  void send(void){
    byte *p = buffer;

    p[0] = '<'; p++;
    packBytes(p,accel_x); p+=sizeof(int);
    packBytes(p,accel_y); p+=sizeof(int);
    packBytes(p,accel_z); p+=sizeof(int);
    packBytes(p,gyro_x); p+=sizeof(int);
    packBytes(p,gyro_y); p+=sizeof(int);
    packBytes(p,gyro_z); p+=sizeof(int);
    packBytes(p,ref); p+=sizeof(int);
    packBytes(p,mag_x); p+=sizeof(int);
    packBytes(p,mag_y); p+=sizeof(int);
    packBytes(p,mag_z); p+=sizeof(int);
    packBytes(p,batt); p+=sizeof(int);
    *p = '>';

    Serial.write(buffer,len);

    /*
	Serial.write('<');
     	//Serial.write(p,sizeof(sensors_t));
     	Serial.write(s.ir);
     	writeBytes(s.compass);
     	writeBytes(s.x);
     	writeBytes(s.y);
     	writeBytes(s.z);
     	writeBytes(s.w);
     	writeBytes(s.v_ref);
     	writeBytes(s.batt);
     	writeBytes(s.en0); s.en0 = 0; // clear encoder
     	writeBytes(s.en1); s.en1 = 0; // clear encoder
     	writeBytes(s.time);
     	Serial.write('>');
     	 */
  }

private:
  byte buffer[STATUS_LEN];

  int accel_x, accel_y, accel_z;
  int mag_x, mag_y, mag_z;
  int gyro_x, gyro_y, gyro_z, ref;
  int batt;
  //int ir0,ir1,ir2;
  //byte bump;  // <xxxxx L C R>
  //byte cliff; // <xxxx LR LF RF RR>
  byte error;
};

#endif

