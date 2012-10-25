/*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 2 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*    Copyright © 2012 Shridhar Shah
*/
#include <WProgram.h>

#include <Servo.h> 
#include <ros.h>
//#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

#define STBY 6
#define B_PWM 3
#define A_PWM 9
#define A_IN2 8
#define A_IN1 7
#define B_IN1 5
#define B_IN2 4
//#define motor_A 0
//#define motor_B 1
//#define motor_AB 2
ros::NodeHandle  nh;

float servoL, servoR; 

Servo ServoL;
Servo ServoR;

int commandL = 0;
int commandR = 0;

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


void messageCb( const geometry_msgs::Point& msg){
  servoL = msg.x;
  servoR = msg.y;
  commandL = servoL;
  commandR = servoR;
  if(commandL == 0.0)
  {
    // ServoL.detach();
    digitalWrite(B_IN1, LOW);   // 
    digitalWrite(B_IN2, LOW);   // 
    digitalWrite(B_PWM, HIGH);   // 
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
  }
  else if (commandL > 0)
  { 
    // ServoL.attach(B_PWM);
    digitalWrite(B_IN2, HIGH); 
    digitalWrite(B_IN1, LOW);
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
    analogWrite(B_PWM, commandL);
    // ServoL.write(commandL);  
    //analogWrite(9, command);    
    //servoL.write(cmd_msgL.data); //set servo angle, should be from 0-180  
    //servoR.write(cmd_msgL.data); //set servo angle, should be from 0-180  
    //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
  }
  else
  {
    // ServoL.attach(B_PWM);
    digitalWrite(B_IN2, LOW); 
    digitalWrite(B_IN1, HIGH);
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
    analogWrite(B_PWM, -commandL);
    //ServoL.write(-commandL); 

  }

  //flush(cmd_msgL);



  if(commandR == 0.0)
  {
    // ServoR.detach();
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
    digitalWrite(A_IN1, LOW);
    digitalWrite(A_IN2, LOW);
    digitalWrite(A_PWM, HIGH);

  }
  else if(commandR > 0)
  { 

    //  ServoR.attach(A_PWM);
    digitalWrite(A_IN2, LOW); 
    digitalWrite(A_IN1, HIGH);
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
    //analogWrite(6, command);   
    analogWrite(A_PWM, commandR);
    // ServoR.write(commandR);  
    //servoL.write(cmd_msgL.data); //set servo angle, should be from 0-180  
    //servoR.write(cmd_msgL.data); //set servo angle, should be from 0-180  
    //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
  }
  else
  {
    //ServoR.attach(A_PWM);
    digitalWrite(A_IN2, HIGH); 
    digitalWrite(A_IN1, LOW);
    digitalWrite(STBY, HIGH);   // STYBY - HIGH
    //analogWrite(6, command);   
    analogWrite(A_PWM, -commandR);
    // ServoR.write(-commandR); 



  }

}


//std_msgs::Float64 test;
ros::Subscriber<geometry_msgs::Point> servocmd("servocommand", &messageCb);
//ros::Subscriber<std_msgs::Float64> sr("servoright", &messageCbr);
//ros::Publisher p("chatter", &test);

void setup(){
  pinMode(13, OUTPUT); // LED Indicator
  pinMode(A_IN1, OUTPUT);  // AIN2
  pinMode(A_IN2, OUTPUT);  // AIN1
  pinMode(STBY, OUTPUT);  // STNBY
  pinMode(B_IN1, OUTPUT);  // BIN1
  pinMode(B_IN2, OUTPUT);  // BIN2
  pinMode(A_PWM, OUTPUT);  // 
  pinMode(B_PWM, OUTPUT);  // 

  nh.initNode();
  //  nh.advertise(p);
  nh.subscribe(servocmd);
  //  nh.subscribe(sr);  
  //  servoL.attach(6); //attach it to pin 9
  //  servoR.attach(9); //attach it to pin 9

  digitalWrite(B_IN2, LOW);   // // AIN1 - LOW AIN2 - HIGH (CC rotation)
  digitalWrite(B_IN1, LOW);   // // AIN2
  digitalWrite(B_PWM, LOW);   // 
  digitalWrite(STBY, LOW);   // STYBY - HIGH 
  digitalWrite(A_IN1, LOW); // BIN2 - HIGH BIN1 - LOW (CCW rotation)
  digitalWrite(A_IN2, LOW);  // BIN1
  digitalWrite(A_PWM, LOW);
  
  setPwmFrequency(3, 256);
  setPwmFrequency(9, 256);
}

void loop(){
  // test.data = x;
  // p.publish( &test );
  
  nh.spinOnce();
  delay(1);

}

