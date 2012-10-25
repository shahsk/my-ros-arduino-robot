#include <WProgram.h>

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64.h>

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

void messageCbl( const std_msgs::Float64& msg){
  servoL = msg.data;
 // digitalWrite(13, HIGH-digitalRead(13));   // blink the led
 // command = cmd_msgL.data;
 commandL = servoL;
  if(commandL == 0.0)
  {
  // ServoL.detach();
  digitalWrite(B_IN1, LOW);   // 
  digitalWrite(B_IN2, LOW);   // 
  digitalWrite(B_PWM, HIGH);   // 
  digitalWrite(STBY, HIGH);   // STYBY - HIGH
 // digitalWrite(12, LOW);
 // digitalWrite(10, LOW);
 // digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
 // digitalWrite(9, HIGH);
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
 //delay(300);
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
  
}



void messageCbr( const std_msgs::Float64& msg){
  servoR = msg.data;
 // digitalWrite(13, HIGH-digitalRead(13));   // blink the led
 // command = cmd_msgL.data;
 commandR = servoR;
  if(commandR == 0.0)
  {
   // ServoR.detach();
 // digitalWrite(7, LOW);   // 
 // digitalWrite(8, LOW);   // 
 // digitalWrite(6, HIGH);   // 
  digitalWrite(STBY, HIGH);   // STYBY - HIGH
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
  digitalWrite(A_PWM, HIGH);
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
 //delay(300);
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
ros::Subscriber<std_msgs::Float64> sl("servoleft", &messageCbl);
ros::Subscriber<std_msgs::Float64> sr("servoright", &messageCbr);
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
  nh.subscribe(sl);
  nh.subscribe(sr);  
//  servoL.attach(6); //attach it to pin 9
//  servoR.attach(9); //attach it to pin 9
  
  digitalWrite(B_IN2, LOW);   // // AIN1 - LOW AIN2 - HIGH (CC rotation)
  digitalWrite(B_IN1, LOW);   // // AIN2
  digitalWrite(B_PWM, LOW);   // 
  digitalWrite(STBY, LOW);   // STYBY - HIGH 
  digitalWrite(A_IN1, LOW); // BIN2 - HIGH BIN1 - LOW (CCW rotation)
  digitalWrite(A_IN2, LOW);  // BIN1
  digitalWrite(A_PWM, LOW);
  
}

void loop(){
 // test.data = x;
 // p.publish( &test );
  nh.spinOnce();
  delay(20);
  
}
