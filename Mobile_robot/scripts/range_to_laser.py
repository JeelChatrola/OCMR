#!/usr/bin/env python

# defining all the libraries
import rospy
import tf
import math
import time
import numpy as np

#defining data type used for a rostopic
from sensor_msgs.msg import LaserScan, Range

# variables to be used
last_time = 0.0
range_values = 0.0

rospy.init_node('range_to_laser')

scan = LaserScan()
ranges = Range()

#function to convert the degrees to radians
def deg2rad(deg):
    return deg * math.pi / 180

# Array defining the waypoints the servo goes through to get the range values
range_array = [0.0]*19
range_angle_array = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180]

'''
For 5 deg incremental changes
[0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,
125,130,135,140,145,150,155,160,165,170,175,180] - 37

For 3 degrees incremental changes
[   0,    3,    6,    9,   12,   15,   18,   21,   24,   27,   30,   33,
   36,   39,   42,   45,   48,   51,   54,  57,   60,   63,   66,   69,
   72,   75,   78,   81,   84,   87,   90,   93,   96,   99,  102,  105,
  108,  111,  114,  117,  120,  123,  126,  129,  132,  135,  138,  141,
  144,  147,  150,  153,  156,  159,  162,  165,  168,  171,  174,  177,
  180] - 61
'''

# Clearing the Scan message for new incoming message
def clear_scan():
    range_array = np.empty(19,dtype=float)


# callback function when we recieve the new values on the range_data topic
def ranges_callback(ranges):

    global last_time

    # converting the mm to Meter range for ROS to interpret
    answer = ranges.range/1000
    range_values = round(answer,2)
    
    # if the motor is at zero position keep that value at firt position in array also define the header stamp time
    if ranges.field_of_view == 0:
        last_time = time.time()
        scan.header.stamp = ranges.header.stamp

        if range_values > 1.20:
            range_values = 1.20
            
        elif range_values < 0.04:
            range_values = 0.04

        range_array[0] = range_values
    
    # adding the values of ranges in the Scan array matching their position in the array
    elif ranges.field_of_view <= 180 and ranges.field_of_view != 0:
        index_of_value = range_angle_array.index(ranges.field_of_view)
        
        if range_values > 1.2:
            range_values = 1.20
            
        elif range_values < 0.03:
            range_values = 0.03

        range_array[index_of_value] = range_values

        # once a laser scan is over clear scan and find the scan time taken.
        if ranges.field_of_view == 180:
            
            clear_scan()

            scan.scan_time = time.time() - last_time
    
# callback function converting the range_data to Laser_scan message
def laser_callback():

    global range_array
    global current_time,last_time,scantime

    scan.header.frame_id ='scanner'
    #scan.header.stamp = rospy.Time.now()
    scan.angle_min = 0
    scan.angle_max = deg2rad(180)
    scan.angle_increment = deg2rad(10)
    scan.range_min = 0.04 
    scan.range_max = 1.2
    
    scan.time_increment = 0
    
    scan.ranges = range_array
    
    return scan

# publisher and subcriber definition

pub = rospy.Publisher('/scan',LaserScan,queue_size=20)
sub = rospy.Subscriber('/range_data',Range,ranges_callback)

# infinite loop converting the range_data to laser_scan untill data is stopped sending
while not rospy.is_shutdown():
    
    scan = laser_callback()
    pub.publish(scan)
    time.sleep(0.001)

rospy.spin()
