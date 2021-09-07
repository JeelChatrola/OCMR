#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import Int64


class DiffTf:


    
    def __init__(self):
   
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### all the motor parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 40690))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.24)) # The wheel base width in meters
        
        self.encoder_min = rospy.get_param('encoder_min', -21474836472768)
        self.encoder_max = rospy.get_param('encoder_max', -21474836472768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("encoder_L", Int64, self.lwheelCallback)
        rospy.Subscriber("encoder_R", Int64, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        
        
    '''
    def static_laser_transformStamped(self):

        self.laserBroadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.static_laser_transformStamped = geometry_msgs.msg.TransformStamped
        self.static_laser_transformStamped.header.stamp = rospy.Time.now()
        self.static_laser_transformStamped.header.frame_id = "laser_link"
        self.static_laser_transformStamped.header.child_frame_id = self.base_frame_id

        self.static_laser_transformStamped.transform.translation.x = 0.28
        self.static_laser_transformStamped.transform.translation.y = 0.0
        self.static_laser_transformStamped.transform.translation.z = 0.1

        #quat = tf.transformations.quaternion_from_euler()

        self.static_laser_transformStamped.transform.rotation.x = 0
        self.static_laser_transformStamped.transform.rotation.y = 0
        self.static_laser_transformStamped.transform.rotation.z = 0
        self.static_laser_transformStamped.transform.rotation.w = 1
 
        self.laserBroadcaster.sendTransform(static_laser_transformStamped)
    '''
    # infinite loop to keep the node running till the roscore is shutdown
    def spin(self):
    
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    # function to calculate the transforms based on the encoder data received
    def update(self):
        
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           
             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # publish the odom information

            self.odomBroadcaster = tf2_ros.TransformBroadcaster()
            self.t = geometry_msgs.msg.TransformStamped()
            
            self.t.header.stamp = rospy.Time.now()
            self.t.header.frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
            self.t.child_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )

            
            self.t.transform.translation.x = self.x
            self.t.transform.translation.y = self.y
            self.t.transform.translation.z = 0
            

            #quat = tf.transformations.quaternion_from_euler()

            self.t.transform.rotation.x = quaternion.x
            self.t.transform.rotation.y = quaternion.y
            self.t.transform.rotation.z = quaternion.z
            self.t.transform.rotation.w = quaternion.w

            

            self.odomBroadcaster.sendTransform(self.t)
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.t.header.frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.t.child_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)


    
    def lwheelCallback(self, msg):
    
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
   
    def rwheelCallback(self, msg):
    
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass