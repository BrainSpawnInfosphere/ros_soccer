
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <Arduino.h> 
#include <inttypes.h>
#include <L3G4200D.h> // these don't help!! need to put in main.cpp file before cSensors.h
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
    gyro.enableDefault();
    compass.init();
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

  void printIMU(void){
    read();

    Serial.println("----------------------");

    Serial.print("Accel: ");
    Serial.print(compass.a.x);
    Serial.print(" ");
    Serial.print(compass.a.y);
    Serial.print(" ");
    Serial.println((int)compass.a.z);

    Serial.print("Mag: "); 
    Serial.print((int)compass.m.x);
    Serial.print(" ");
    Serial.print((int)compass.m.y);
    Serial.print(" ");
    Serial.println((int)compass.m.z);

    Serial.print("Gyro: ");
    Serial.print((int)gyro.g.x);
    Serial.print(" ");
    Serial.print((int)gyro.g.y);
    Serial.print(" ");
    Serial.println((int)gyro.g.z);
    
    Serial.print("Battery: ");
    Serial.println((int)batt);
  }

  bool read(){
    compass.read();
    gyro.read();
    batt = analogRead(BATT_PIN);

    return true;
  }

private:
  byte buffer[STATUS_LEN];

  L3G4200D gyro;
  LSM303 compass;

  //int accel_x, accel_y, accel_z;
  //int mag_x, mag_y, mag_z;
  //int gyro_x, gyro_y, gyro_z, ref;
  int batt;
  //int ir0,ir1,ir2;
  //byte bump;  // <xxxxx L C R>
  //byte cliff; // <xxxx LR LF RF RR>
  byte error;
};

#endif



