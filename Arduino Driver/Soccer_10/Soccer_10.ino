



#include <Wire.h>
#include <L3G.h>     // <L3G4200D.h>
#include <LSM303.h>  // <LSM303.h>
#include <CmdMessenger.h>

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

#define COMM_SWITCH 10

cSensor sensors;
CmdMessenger cmdmsg;

MotorDriver motorAB = MotorDriver(m1pwm,m1a,m1b,m2pwm,m2a,m2b);
MotorDriver motorCD = MotorDriver(m3pwm,m3a,m3b,m4pwm,m4a,m4b);


void allStop(){
  motorAB.coastBothMotors();
  motorCD.coastBothMotors();

  delay(100);

  motorAB.stopBothMotors();
  motorCD.stopBothMotors();
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

void print_cb(void* arg){
  sensors.read();
  char array[128];

  String msg;
  sensors.ascii(msg);
  msg.toCharArray(array,128);
  cmdmsg.print(array);
}


void printLoop_cb(void* arg){
  int i=30;

  while(--i){
    print_cb(NULL);
    delay(500);
  }
}

void status_cb(void* arg){
  int sz;
  char *str = sensors.getMessage(sz);
  cmdmsg.sendCmd('s',str,sz);
}

void motors_cb(void* arg){
  Msg_t *msg = (Msg_t*) arg;
  byte a,b,c,d,dir;
  
  // Is this a valid motor message?
  if(msg->msg_size != 5){ 
    cmdmsg.sendCmd('e',NULL,0); // no - send error
    return;
  }

  dir = (byte)msg->msg[0];
  a = (byte)msg->msg[1];
  b = (byte)msg->msg[2];
  c = (byte)msg->msg[3];
  d = (byte)msg->msg[4];
  setMotors(dir,a,b,c,d);
}


void test_cb(void* arg){
  byte dir;
  const byte forward = 15; // B1111
  const byte backward = 0; // B0000
  const byte speed = 127;

  for(int i=0;i<2;++i){
    dir = (i == 0 ? forward : backward);
    setMotors(dir,speed,speed,speed,speed);
    delay(1000);
  }

  allStop();
}


void halt_cb(void* arg){
  allStop();
}

void version_cb(void* arg){
  char *str = "Robot 0.01";
  cmdmsg.sendCmd('v',str,10);
}



/**
 *
 */
void setup()
{
  // Start USB
  //  9600, 14400, 19200, 28800, 38400, 57600, or 115200
  Serial.begin(9600);

  // Start Bluetooth
  Serial1.begin(115200);
  
  
  pinMode(COMM_SWITCH,  INPUT_PULLUP);

  // start-up motor drivers and set to stop
  motorAB.begin();
  motorCD.begin();
  allStop();

  // start-up I2C and IMU
  Wire.begin();

  sensors.init();
  
  int val = digitalRead(COMM_SWITCH);

  if(val == LOW) cmdmsg.init(Serial1); // cmds over BT
  else cmdmsg.init(Serial); // cmds over USB
  
  cmdmsg.attach('h',0,halt_cb);
  cmdmsg.attach('s',0,status_cb);
  cmdmsg.attach('p',0,print_cb);  // debug
  cmdmsg.attach('P',0,printLoop_cb);  // debug
  cmdmsg.attach('m',5,motors_cb);
  cmdmsg.attach('v',0,version_cb);
  cmdmsg.attach('t',0,test_cb);

  cmdmsg.sendCmd('g',NULL,0);
  
   analogReadResolution(12); // enable 12b on Due
}


void loop()
{
  boolean ok;

  //sensors.read();

  if(cmdmsg.available() >=3){
    ok = cmdmsg.feedinSerialData(); // could we read the message?
    if( !ok ) cmdmsg.sendCmd('e',NULL,0); // no there was an error, tell the host
  }

}
