/* 
 *  The main function of this code is to publish the encoder values of left and right motor
 *  to the ros. Other function is getting the range values with the help of VL53L0X ToF sensor
 *  mounted on a servo motor and publishing it to ros for further processing.
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
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

#include <Servo.h>
#include "Seeed_vl53l0x.h"

Servo Rservo;
Seeed_vl53l0x VL53L0X;

//=====================================================================================================
// Defining all the hardware pins used

const int encoderPinA_L = 2; // encoder pins for motor 1 ( external interrupt capable )
const int encoderPinB_L = 3;

const int encoderPinA_R = 18; // encoder pins for motor 2 ( external interrupt capable )
const int encoderPinB_R = 19;

volatile long currentPosition_L = 0; // variable to store current position of motor1
volatile long currentPosition_R = 0; // variable to store current position of motor2


//=====================================================================================================
// Defining all the publisher and their data type

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
std_msgs::Int64 encoder_L; 
std_msgs::Int64 encoder_R; 
ros::Publisher pub_range( "range_data", &range_msg);
ros::Publisher encoderValue_L("encoder_L", &encoder_L);
ros::Publisher encoderValue_R("encoder_R", &encoder_R);

unsigned long range_timer;
char frameid[] = "/ir_ranger";

//=====================================================================================================
// Intializing the node handle ,interrupt pins and the VL53L0X sensor

void setup()
{
  Serial.begin(115200);

  // Node handle and publishers
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(encoderValue_L);
  nh.advertise(encoderValue_R);

  //Interrupt pins setup and definition
  pinMode(encoderPinA_L,INPUT_PULLUP);
  pinMode(encoderPinB_L,INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA_L), readEncoderA_L, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB_L), readEncoderB_L, CHANGE);
  
  pinMode(encoderPinA_R,INPUT_PULLUP);
  pinMode(encoderPinB_R,INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA_R), readEncoderA_R, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB_R), readEncoderB_R, CHANGE);

  //set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & 0b11111000 | 1;

  //Intializing the VL53L0X sensor
  
  VL53L0X_Error Status = VL53L0X_ERROR_NONE; 
  Status = VL53L0X.VL53L0X_common_init();
  //VL53L0X.VL53L0X_high_accuracy_ranging_init();
  //VL53L0X.VL53L0X_long_distance_ranging_init();
  VL53L0X.VL53L0X_high_speed_ranging_init();

  // servo pin definition
  Rservo.attach(13);
  
  // range message values
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.02;
  range_msg.min_range = 0.03;
  range_msg.max_range = 1.0;
  
}

//=====================================================================================================
// Interrupt Service Routine for reading the encoder values.

void readEncoderA_L()
{
    
  if (digitalRead(encoderPinA_L) != digitalRead(encoderPinB_L))
  {
    
    currentPosition_L++;
    
  }
  else
  {
    
    currentPosition_L--;
  }
  
}

void readEncoderB_L()
{
  
  if (digitalRead(encoderPinA_L) == digitalRead(encoderPinB_L))
  {
    
    currentPosition_L++;
  }
  else
  {
    
    currentPosition_L--;
  }
  
}

void readEncoderA_R()
{
      
  if (digitalRead(encoderPinA_R) != digitalRead(encoderPinB_R))
  {
    
    currentPosition_R++;
    
  }
  else
  {
    currentPosition_R--;
  }
  
}

void readEncoderB_R()
{
    
  if (digitalRead(encoderPinA_R) == digitalRead(encoderPinB_R))
  {
    
    
    currentPosition_R++;
  }
  else
  {
    
    currentPosition_R--;
  }
  
}

//=====================================================================================================
// Function to get the range values from the sensor

float getRange(){
    
    int sample;
    
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    
    //memset(&RangingMeasurementData, 0, sizeof(VL53L0X_RangingMeasurementData_t));
    Status = VL53L0X.PerformSingleRangingMeasurement(&RangingMeasurementData);

    sample = RangingMeasurementData.RangeMilliMeter;
    delay(5);
    
    return (sample);
}

//=====================================================================================================

void loop()
{
  // publish the range value every 50 milliseconds
  // since it takes that long for the sensor to stabilize
    
  if ( (millis()-range_timer) > 50){

    // moving motor in steps to emulate the lidar using a for loop 
    for ( int angle_setpoints= 0; angle_setpoints<= 180 ; angle_setpoints+= 10){

    
    encoder_L.data = currentPosition_L;
    encoderValue_L.publish( &encoder_L );

    encoder_R.data = currentPosition_R;
    encoderValue_R.publish( &encoder_R );
    
    
    Rservo.write(angle_setpoints);

    // giving the servo motor some time to return to the 0 position 
    if (angle_setpoints == 0){
      delay(500);
    }
    else { 
    delay(10);
    }

    // combining the motor position and distance data into range message
    range_msg.range = getRange();
    range_msg.header.stamp = nh.now();
    range_msg.field_of_view = angle_setpoints;
    
    pub_range.publish(&range_msg);
    
    range_timer =  millis() + 50;
       
    }
    
    delay(10);
  }
  nh.spinOnce();
}
//=====================================================================================================
