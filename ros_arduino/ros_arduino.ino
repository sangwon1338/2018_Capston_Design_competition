
#include <ros.h>
#include <std_msgs/Int32.h>  
#include <std_msgs/Float32.h>
#include "Servo.h"

Servo myservo;
Servo esc;

const int pwPin = 7;
float duration = 0;

int n=0; 
int m=0;
int dt=100;
int rps_front_left;
int rps_front_right;
int rps_rear_left;
int rps_rear_right;

int arraysize = 9;
double rangevalue[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
double pulse;
double modE;


// pin번호 모음
const int hall=8; //flw
const int hall2=9; //frw
const int hall3=10; //rlw
const int hall4=11; //rrw
unsigned long prevmillis; // To store time
//unsigned long duration; // To store time difference
unsigned long refresh; // To store time for refresh of reading
const int D = 8; // Diameter in cm (include at the top)
int rpm=0,rpm2=0,rpm3=0,rpm4=0; // RPM value

boolean currentstate,currentstate2,currentstate3,currentstate4; // Current state of IR input scan
boolean prevstate,prevstate2,prevstate3,prevstate4;; // State of IR sensor in previous scan

ros::NodeHandle  nh;
std_msgs::Int32 int_msg;
std_msgs::Float32 float_msg;
std_msgs::Float32 float_hallfront_left_msg;
std_msgs::Float32 float_hallfront_right_msg;
std_msgs::Float32 float_hallrear_left_msg;
std_msgs::Float32 float_hallrear_right_msg;
ros::Publisher dist("dist", &float_msg);
ros::Publisher Rps_front_left("Rps_front_left", &float_hallfront_left_msg);
ros::Publisher Rps_front_right("Rps_front_right", &float_hallfront_right_msg);
ros::Publisher Rps_rear_left("Rps_rear_left", &float_hallrear_left_msg);
ros::Publisher Rps_rear_right("Rps_rear_right", &float_hallrear_right_msg);

// servo_motor steering
void servo_control(const std_msgs::Int32& angle_msg){
  myservo.write(angle_msg.data);
}

// DC_motor duty
void motor_duty(const std_msgs::Int32& motor_speed){
  esc.writeMicroseconds(motor_speed.data);
}

ros::Subscriber<std_msgs::Int32> servo_steer("angle_msg", servo_control );
ros::Subscriber<std_msgs::Int32> DC_duty("motor_speed", motor_duty );

void calculateDistance(){
        pulse = pulseIn(pwPin, HIGH);

  float_msg.data=pulse / 58;
  dist.publish( &float_msg);

}

void getRpm(){
   currentstate = digitalRead(hall); // Read IR sensor state
 if( prevstate != currentstate) // If there is change in input
   {
     if( currentstate == HIGH ) // If input only changes from LOW to HIGH
       {
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for nect revolution calculation
       }
   }
  
  prevstate = currentstate; // store this scan (prev scan) data for next scan
  currentstate2= digitalRead(hall2); // Read IR sensor state
 if( prevstate2 != currentstate2) // If there is change in input
   {
     if( currentstate2 == HIGH ) // If input only changes from LOW to HIGH
       {
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm2 = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for nect revolution calculation
       }
   }
  prevstate2 = currentstate2; // store this scan (prev scan) data for next scan
  currentstate3 = digitalRead(hall3); // Read IR sensor state
 if( prevstate3 != currentstate3) // If there is change in input
   {
     if( currentstate3 == HIGH ) // If input only changes from LOW to HIGH
       {
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm3 = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for nect revolution calculation
       }
   }
  prevstate3 = currentstate3; // store this scan (prev scan) data for next scan

 currentstate4 = digitalRead(hall4); // Read IR sensor state
 if( prevstate4 != currentstate4) // If there is change in input
   {
     if( currentstate4 == HIGH ) // If input only changes from LOW to HIGH
       {
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm4 = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for nect revolution calculation
       }
   }
  prevstate4 = currentstate4; // store this scan (prev scan) data for next scan
 
 float_hallfront_left_msg.data=rpm;
 Rps_front_left.publish( &float_hallfront_left_msg);
 float_hallfront_right_msg.data=rpm2;
 Rps_front_right.publish( &float_hallfront_right_msg);
 float_hallrear_left_msg.data=rpm3;
 Rps_rear_left.publish( &float_hallrear_left_msg);
 float_hallrear_right_msg.data=rpm4;
 Rps_rear_right.publish( &float_hallrear_right_msg);
}


void setup() {
  pinMode(pwPin, INPUT);

  myservo.attach(3);
  esc.attach(5);    
  esc.writeMicroseconds(1000);

 pinMode(hall,INPUT);   
 pinMode(hall2,INPUT); 
 pinMode(hall3,INPUT);   
 pinMode(hall4,INPUT);      
  prevmillis = 0;
  prevstate = LOW; 
  prevstate2= LOW;
  prevstate3= LOW; 
  prevstate4= LOW;

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(dist);  
  nh.advertise(Rps_front_left);
  nh.advertise(Rps_front_right);
  nh.advertise(Rps_rear_left);
  nh.advertise(Rps_rear_right);
  nh.subscribe(servo_steer);
  nh.subscribe(DC_duty);
  delay(1000);
}

void loop() {
  calculateDistance(); 
  getRpm();
  nh.spinOnce();
  //delay(50);
}
