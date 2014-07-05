
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <Arduino.h> 
#include <inttypes.h>
#include <L3G.h> // these don't help!! need to put in main.cpp file before cSensors.h
#include <LSM303.h>

//a.x a.y a.z g.x g.y g.z m.x m.y m.z batt = 20 bytes 
#define STATUS_LEN 20

#define CURRENT_PIN 1
#define BATT_PIN    0

/**
 * The buffer holds the message
 */
class cSensor {
public:

  cSensor() {
    ;
  }

  void init(){
    gyro.init(L3G4200D_DEVICE,L3G_SA0_HIGH); // sa0 is pulled high by default
    gyro.enableDefault(); 
    compass.init(LSM303DLM_DEVICE,LSM303_SA0_A_LOW); // sa0 is pulled low by default
    compass.enableDefault();
  }

  // intel stores memory <low,hi>
  void packBytes(byte *m, int v){
    m[0] = ((byte)(v & 0xff));  // send low
    m[1] = ((byte)(v>>8));      // send hi
  }

  /* need to reverse order
  void packBytes(byte *m, unsigned long v){
    m[0] = ((byte)(v>>24 & 0xff));      // send hi
    m[1] = ((byte)(v>>16 & 0xff));      // send hi med
    m[2] = ((byte)(v>>8 & 0xff));       // send low med
    m[3] = ((byte)(v & 0xff));          // send low
  }
  */

  /**
   * Sends the sensor data structure as a series of bytes. It also
   * formats it with the proper start/end message chars.
   */
  char* getMessage(int& msg_size){
    byte *p = buffer;
    msg_size = STATUS_LEN;

    //*p = 10; // 10 bytes
    packBytes(p,(int)compass.a.x); 
    p+=sizeof(int);
    packBytes(p,(int)compass.a.y); 
    p+=sizeof(int);
    packBytes(p,(int)compass.a.z); 
    p+=sizeof(int);
    packBytes(p,(int)gyro.g.x); 
    p+=sizeof(int);
    packBytes(p,(int)gyro.g.y); 
    p+=sizeof(int);
    packBytes(p,(int)gyro.g.z); 
    p+=sizeof(int);
    packBytes(p,(int)compass.a.x); 
    p+=sizeof(int);
    packBytes(p,(int)compass.a.y); 
    p+=sizeof(int);
    packBytes(p,(int)compass.a.z); 
    p+=sizeof(int);
    packBytes(p,batt); 
    p+=sizeof(int);
    
    return (char*)buffer;
  }

  // debug output
  void ascii(String &msg){  
    read();

    msg = "----------------------\n";
    msg += "Accel: " + String(compass.a.x) + " " + String(compass.a.y) + " " + String(compass.a.z) + '\n';
    msg += "Mag: " + String(compass.m.x) + " " + String(compass.m.y) + " " + String(compass.m.z) + '\n';
    msg += "Gyro: " + String(gyro.g.x) + " " + String(gyro.g.y) + " " + String(gyro.g.z) + '\n';
    msg += "Temp: " + String(gyro.readTemperature()) + '\n';
    msg += "Batt: " + String(batt) + '\n';
  }

  bool read(){
    compass.read();
    gyro.read();
    batt = analogRead(BATT_PIN);

    return true;
  }

private:
  byte buffer[STATUS_LEN];
  //char tbuff[128];

  L3G gyro;
  LSM303 compass;
  
  int batt;
  //int ir0,ir1,ir2;
  //byte bump;  // <xxxxx L C R>
  //byte cliff; // <xxxx LR LF RF RR>
  byte error;
};

#endif



