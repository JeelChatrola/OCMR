/* 
 *  The main function of this code is to subscribe to pwm values of left 
 *  and right motor coming from ros and execute the same using a motor shield
*/

// Adding all the libraries and dependencies
#include "math.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>


// defining the pins to control the motor shield
#define BRAKE 0
#define CW 1  // CW = Clockwise
#define CCW 2 // CCW = Counterclockwise

#define MOTOR_L 0
#define MOTOR_R 1

#define MOTOR_A1_PIN 7   // A1 Pin for motor1
#define MOTOR_B1_PIN 8   // B1 Pin for motor1

#define MOTOR_A2_PIN 4   // A2 Pin for motor2
#define MOTOR_B2_PIN 9   // B2 Pin for motor2

#define PWM_MOTOR_L 5    // PWM for motor1
#define PWM_MOTOR_R 6    // PWM for motor2

#define EN_PIN_1 A0   // enable pin for motor1
#define EN_PIN_2 A1   // enable pin for motor2

ros::NodeHandle  nh;

void motorGo(int motor, int direct, int pwm)         //Function that controls the variables: motor(1 or 2), direction (cw ou ccw) or pwm (range 0 to 255);
{
  if(motor == MOTOR_L)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_L, pwm); 
  }


  else if(motor == MOTOR_R)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW); 
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_R, pwm); 
  }
  
}

// Function to get the data from the rostopic and execute it
void pwm_input_L( const std_msgs::Int16& pwm_value)
{
  int pwm =0;
  pwm = pwm_value.data;
  
  if ( pwm > 0 )
  {
  motorGo(MOTOR_L,CCW,pwm);
  }
  else
  {
  motorGo(MOTOR_L,CW,abs(pwm));
  }
} 
ros::Subscriber<std_msgs::Int16> pwm_L("/pwm_L", &pwm_input_L);

// Function to get the data from the rostopic and execute it
void pwm_input_R( const std_msgs::Int16& pwm_value)
{
  int pwm_R =0;
  pwm_R = pwm_value.data;
  
  if ( pwm_R > 0 )
  {
  motorGo(MOTOR_R,CW,pwm_R);
  }
  else
  {
  motorGo(MOTOR_R,CCW,abs(pwm_R));
  }
}
ros::Subscriber<std_msgs::Int16> pwm_R("/pwm_R", &pwm_input_R);

// Initialize the node 
void setup() {
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(pwm_R);
  nh.subscribe(pwm_L);

  TCCR1B = TCCR1B & 0b11111000 | 1;
}

// infinite loop to keep the motor enable pins high while motor is moving.
void loop() {
  
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
   
  nh.spinOnce();

}
