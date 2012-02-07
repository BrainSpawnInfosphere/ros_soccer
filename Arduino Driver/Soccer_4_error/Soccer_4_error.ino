



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

/*
class cSchedule {
public:
  cSchedule(const int a) : 
  size(a), dt(125) { 
    patterns = new unsigned int[a];
  }

  unsigned int get(unsigned int i){
    if(i < size) return patterns[i];

    return 0;
  }

  bool set(const unsigned int i, const unsigned int p){
    if(i < size) patterns[i] = p;
    else return false;

    return true;
  }

  void play(const int a){
    unsigned int p = get(a);
    for(int i=0;i<16;i++){
      if(p & (1<<i)) run();
      else delay(dt);
    }
  }

  virtual void run(void)=0;

protected:
  unsigned int *patterns;
  const unsigned int size;
  const int dt;
};

class cSong : 
public cSchedule {
public:
  cSong(byte pin, int a) : 
  cSchedule(a) {
    tone = new cTone();
    tone->init(pin);
  }

  virtual void run(void){
    tone->beep(dt);
  }

  cTone *tone;
};

class cLED : 
public cSchedule {
public:
  cLED(int p, int a) : 
  cSchedule(a) {
    pin = p;
    digitalWrite(pin,LOW);
  }

  virtual void run(void){
    digitalWrite(pin,HIGH);
    delay(dt);
    digitalWrite(pin,LOW);
  }

protected:
  int pin;
};


cTone sound;
*/

/**
 * Read a specific number of bytes from the computer-> port
 *//*
boolean readBytes(HardwareSerial* s, byte* b, const unsigned int size){
 unsigned int i = 0;
 int inByte = 0;
 byte error = 0;
 
 while(i<size){
 inByte = s->read();
 
 if(inByte == -1){
 delay(1);
 ++error;
 if(error > 5){
 errorCode(2);
 return false;
 }
 }
 else buffer[i++] = (byte)inByte;
 
 }
 
 return true;
 }
 */

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
  // Start XBee
  Serial.begin(57600);
  Serial.print("hello");
/*
  // play start-up sounds
  sound.init(37);
  sound.beep(100);
  delay(100);
  sound.beep(100);
*/
  // start-up motor drivers and set to stop
  motorAB.begin();
  motorCD.begin();
  allStop();

  /*
  cLED led(9,4);
   led.set(0,23);
   led.set(1,255);
   led.set(2, 136);
   led.set(3,345);
   */

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


boolean decodeMsg(byte* buff){	
  byte* p = buff;

  switch(p[0]) {
  case 'p':
    sensors.printIMU();
    break;
    
  case 'P':
    for(int i=0;i<20;i++) sensors.printIMU();
    break;

  case 's': // status 
    sensors.send();
    break;

  case 'm': // motor commands m,dir,pwmA,pwmB,pwmC,pwmD
    byte a,b,c,d,dir;
    dir = *(++p);
    a = *(++p);
    b = *(++p);
    c = *(++p);
    d = *(++p);
    setMotors(dir,a,b,c,d);
    break;

  case 'h': // halt/stop
    allStop();
    break;
    
  case 'v':
    Serial.print("--------------------\nSoccer Robot Ver. 0.1\nby Kevin Walchko\n--------------------\n");
    break;

  default:
    return false;
  }
  return true;
}

/**
 * Grab the entire msg at once and store it into a buffer
 *
 * <(msg)(data)...(data)>
 *
 * \todo set for specific msg sizes
 *       single_cmd size=3 <X>   X=msg
 *       double_cmd size=4 <XY>  X=msg Y=data
 *       eeprom_cmd size=? <XYZ> X=msg Y=size Z=variable_data
 *
 * \note The ascii char '>' for end message is value 62, make sure
 *       the data isn't set to 62 or there may be an issue
 */
bool fillBuffer(void){
  //bool ok = false;
  static const int max_msg_len = 8;

  int ascii = Serial.read();
  while(ascii == '\n' || ascii == '\r') ascii = Serial.read();
  if(ascii != '<') return false;
 
  static byte buffer[max_msg_len];
  memset(buffer,0,max_msg_len);
  ascii = Serial.read();
  int i = 0;
 
 // could check for valid message: h,m,p,s ....
 
  while( ascii != '>'){
    if(i == max_msg_len){
      return false;
    }

    buffer[i++] = ascii;
    ascii = Serial.read();
  }
    
  return decodeMsg(buffer);
}

/**
 * Handles serial commands received, sends and error message (<e>) if
 * error occurs.
 */
bool handleSerial(){
 
  bool ok = fillBuffer();
  if(!ok) Serial.print("<e>"); // error
  
  return true;
}

void loop()
{
  //sensors.read();

  // if we get a valid byte, read it
  //Serial.print(Serial.available());
  while (Serial.available()) {
    handleSerial();
  }
}













