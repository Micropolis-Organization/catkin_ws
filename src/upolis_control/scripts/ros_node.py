#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# linear_x = angular_z= float()
maxVelocity = 0.5 # m/s
minVelocity = -1.0 # m/s

maxSteering = 0.35 # rad
minSteering = -0.35 # rad

def map_value(x, a, b, c, d):
    """
    Maps a value x from range [a, b] to range [c, d].

    Parameters:
    - x: The value to map.
    - a: The lower bound of the original range.
    - b: The upper bound of the original range.
    - c: The lower bound of the new range.
    - d: The upper bound of the new range.

    Returns:
    - The value of x mapped to the new range [c, d].
    """
    return float(c + (x - a) * (d - c) / (b - a))

def cmd_vel_callback(cmdVelMsg=Twist()):
    linear_x = Float32()
    angular_z =Float32()
    
    #! limit the velocity and steering to the max and min values
    limit_linear_x = min(max(minVelocity,cmdVelMsg.linear.x),maxVelocity)
    limit_angular_z =  min(max(minSteering,cmdVelMsg.angular.z),maxSteering) 

    # print("limit_linear_x: ", limit_linear_x)
    # print("limit_angular_z: ", limit_angular_z)
    
    #! convert to percentage
    if limit_linear_x >0 :
        linear_x.data = map_value(limit_linear_x, 0, maxVelocity, 55, 100)
    
    elif limit_linear_x < 0:
        linear_x.data = map_value(limit_linear_x, minVelocity, 0, 0, 45)
    
    elif limit_linear_x == 0:
        linear_x.data = 50
        
    
  
    # linear_x.data = min(max(0, linear_x.data), 75)
    angular_z.data = map_value(limit_angular_z, minSteering, maxSteering, 100, 0)
    
    
    #! publish the limited msg
    
    # rospy.loginfo("Linear: " + str(linear_x.data) + " Angular: " + str(angular_z.data))  # Log the received data
    velocity_pub.publish(linear_x)
    steering_pub.publish(angular_z)
    

if __name__ == "__main__":
     rospy.init_node("ros_node")
     cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
     velocity_pub = rospy.Publisher("/velocity", Float32, queue_size=1)
     steering_pub = rospy.Publisher("/steering_rad", Float32, queue_size=1)
     
     rospy.spin()
