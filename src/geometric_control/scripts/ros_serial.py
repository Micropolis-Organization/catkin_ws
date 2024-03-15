#!/bin/python3

import serial
import rospy, time
from std_msgs.msg import Float32



serial_port = "/dev/evcu"
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate)
data_to_send = ""
output_throttle = 50
output_steering = 50



def velocity_cb(msg:Float32):
     global output_throttle
     output_throttle = msg.data

def steering_cb(msg:Float32):
     global output_steering
     output_steering = msg.data
     

def main():

    rospy.init_node('ros_serial_node', anonymous=True)
    rospy.Subscriber("/velocity", Float32,velocity_cb)
    rospy.Subscriber("/steering_rad", Float32,steering_cb)
    rate = rospy.Rate(10) # 10hFlopublishat32

    while not rospy.is_shutdown():

      data_to_send = "d:%3.d" % (int(output_throttle))
      data_to_send += ",s:%3.d" % (int(output_steering))
      rospy.loginfo(data_to_send)
      str_data = str(data_to_send) + "\n"
      ser.write(str_data.encode(encoding="ascii"))  # Send the integer data as a string

      rate.sleep()
      
     





if __name__ == '__main__':
    
    try:
        main()

    except rospy.ROSInterruptException:
        ser.close()

