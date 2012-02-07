



#include <Wire.h>
#include <L3G4200D.h>
#include <LSM303.h>

#include "cTone.h"
#include "TB6612FNG.h"
#include "cSensor.h"

#define m1a    22
#define m1b    23
#define m1pwm  7
#define m2a    24
#define m2b    25
#define m2pwm  8

#define m3a    38
#define m3b    39
#define m3pwm  5
#define m4a    40
#define m4b    41
#define m4pwm  6

#define RESET   14
#define BEEP    15


cSensor sensors;

MotorDriver motorAB = MotorDriver(m1pwm,m1a,m1b,m2pwm,m2a,m2b);
MotorDriver motorCD = MotorDriver(m3pwm,m3a,m3b,m4pwm,m4a,m4b);

//


void allStop(){
  motorAB.coastBothMotors();
  motorCD.coastBothMotors();

  delay(100);

  motorAB.stopBothMotors();
  motorCD.stopBothMotors();
}

void setup()
{
  // Start USB
  Serial.begin(57600);
  Serial.println("<START>");
  
  // Start Bluetooth
  Serial1.begin(57600);
  
  // start-up motor drivers and set to stop
  motorAB.begin();
  motorCD.begin();
  allStop();

  // start-up I2C and IMU
  Wire.begin();
  
  sensors.init();
}

// dir <xxxx d c b a>  0 - reverse, 1 - forward
void setMotors(byte dir, byte a, byte b, byte c, byte d){
  //Serial.print(char(dir));
  //Serial.println(char(a));
  if(dir & 1) motorAB.motor0Forward(a);
  else motorAB.motor0Reverse(a);

  if(dir & 2) motorAB.motor1Forward(b);
  else motorAB.motor1Reverse(b);

  if(dir & 4) motorCD.motor0Forward(c);
  else motorCD.motor0Reverse(c);

  if(dir & 8) motorCD.motor1Forward(d);
  else motorCD.motor1Reverse(d);
}

void getData(int size, char *buffer, HardwareSerial *serial){
  memset(buffer,0,size); // clear what we need
  
  for(int i=0;i<size;i++){
    buffer[i++] = serial->read();
  }
  
}

boolean decodeMsg(HardwareSerial* serial){	
  byte a,b,c,d,dir;
  static byte buffer[16];
  char sz;
  
  char msg = serial->read(); // get msg
  
  //if(msg == 's') sensors.read();

  switch(msg) {
  case 'p':
    sensors.read();
    sensors.printIMU();
    break;
    
  case 'P':
    for(int i=0;i<20;i++){
      sensors.read(); 
      sensors.printIMU();
    }
    break;
  
  case 's': // status 
    sensors.read();
    sensors.send(serial);
    break;

  case 'm': // motor commands m,dir,pwmA,pwmB,pwmC,pwmD
    //byte a,b,c,d,dir;
    
    getData(5,(char*)buffer,serial);
    
    dir = buffer[0];
    a = buffer[1];
    b = buffer[2];
    c = buffer[3];
    d = buffer[4];
    setMotors(dir,a,b,c,d);
    break;

  case 'h': // halt/stop
    allStop();
    break;
    
  case 'v':
    serial->print("<--------------------\nSoccer Robot Ver. 0.1\nby Kevin Walchko\n--------------------\n>");
    break;

  default:
    //serial->print("dcm: ");
    //serial->println(msg);
    return false;
  } 
  
  return true;
}


/**
 * Grab the entire msg at once and store it into a buffer
 *
 * <(msg)[size](data)...(data)> Note: that size is optional depending on msg
 *
 *
 * msg size
 * m   5 (dir u1 u2 u3 u4)
 * s   0
 * p   0
 * P   0
 * v   0
 *
 * \todo set for specific msg sizes
 *       single_cmd size=3 <X>   X=msg
 *       double_cmd size=4 <XY>  X=msg Y=data
 *       eeprom_cmd size=? <XYZ> X=msg Y=size Z=variable_data
 *
 * \note The ascii char '>' for end message is value 62, make sure
 *       the data isn't set to 62 or there may be an issue
 */
bool fillBuffer(HardwareSerial *serial){
  bool ok = false;

  // get start char
  char ascii = serial->read();
  
  //serial->print("fb found: ");
  //serial->println(ascii);
  
  //while(ascii == '\n' || ascii == '\r') ascii = serial->read();
  if(ascii != '<'){ 
    Serial.print("fb: ");
    Serial.println(ascii);
    return false;
  }
  
  // could check for valid message: h,m,p,s .... 
  ok = decodeMsg(serial);
  if(!ok) return false;
  
  // get end char
  ascii = serial->read();
  if(ascii == '>') return true;
  
  return false;
}

/**
 * Handles serial commands received, sends and error message (<e>) if
 * error occurs.
 */
bool handleSerial(HardwareSerial* serial){
  char ascii;
  bool ok = fillBuffer(serial);
  if(!ok){
    serial->print("<e>"); // error
    
    while(serial->available()){
      ascii = serial->read();
      delay(1);
    }
  }
  
  return true;
}

void loop()
{
  //sensors.read();

  // if we get a valid byte, read it
  while (Serial.available() >= 3) { // USB
    handleSerial(&Serial);
  }
  
  while (Serial1.available()) { // Bluetooth
    handleSerial(&Serial1);
    //Serial2.flush();
    //while(Serial
    Serial.print("BT");
    Serial1.println("hi");
  }
  
}













