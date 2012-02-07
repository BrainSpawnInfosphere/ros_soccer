

#ifndef __TB6612FNG_H__
#define __TB6612FNG_H__

#include <Arduino.h> 


class MotorDriver {
public:

  MotorDriver(int pwmA, int a0, int a1, int pwmB, int b0, int b1, int reset=-1) :
  pwmA_pin(pwmA), pwmB_pin(pwmB), A0_pin(a0), A1_pin(a1), B0_pin(b0), 
  B1_pin(b1), reset_pin(reset) {

    pinMode(pwmA_pin, OUTPUT);
    pinMode(pwmB_pin, OUTPUT);
    pinMode(A0_pin, OUTPUT);
    pinMode(A1_pin, OUTPUT);
    pinMode(B0_pin, OUTPUT);
    pinMode(B1_pin, OUTPUT);

    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
  }

  void begin(){
    if(reset_pin){
      digitalWrite(reset_pin, LOW);
      delay(100);
      digitalWrite(reset_pin, HIGH);
    }
  }

  void motor0Forward(uint8_t speed){
    digitalWrite(A0_pin,LOW);
    digitalWrite(A1_pin,HIGH);
    analogWrite(pwmA_pin, speed);
  }

  void motor1Forward(uint8_t speed){
    digitalWrite(B0_pin,LOW);
    digitalWrite(B1_pin,HIGH);
    analogWrite(pwmB_pin, speed);
  }

  void motor0Reverse(uint8_t speed){
    digitalWrite(A0_pin,HIGH);
    digitalWrite(A1_pin,LOW);
    analogWrite(pwmA_pin, speed);
  }

  void motor1Reverse(uint8_t speed){
    digitalWrite(B0_pin,HIGH);
    digitalWrite(B1_pin,LOW);
    analogWrite(pwmB_pin, speed);
  }

  /**
   * Stop is defined as break. It locks both motors up.
   */
  void stopBothMotors(){
    stopMotor0();
    stopMotor1();
  }

  void stopMotor0(){
    digitalWrite(A0_pin,HIGH);
    digitalWrite(A1_pin,HIGH);
  }

  void stopMotor1(){
    digitalWrite(B0_pin,HIGH);
    digitalWrite(B1_pin,HIGH);
  }



  /**
   * Coast is defined as a high impedance state. No current runs through motors
   * and they just spin.
   */
  void coastBothMotors(){
    motor0Coast();
    motor1Coast();
  }

  void motor0Coast(){
    digitalWrite(A0_pin,LOW);
    digitalWrite(A1_pin,LOW);
    analogWrite(pwmA_pin, 255);
  }

  void motor1Coast(){
    digitalWrite(B0_pin,LOW);
    digitalWrite(B1_pin,LOW);
    analogWrite(pwmB_pin, 255);
  }

protected:

  const int pwmA_pin, pwmB_pin, A0_pin, A1_pin, B0_pin, B1_pin, reset_pin;
};

#endif
