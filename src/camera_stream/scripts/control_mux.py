#!/usr/bin/env python

from glob import glob
import random
import rospy
from std_msgs.msg import Int16, Float32
import numpy as np

throttle_publisher = rospy.Publisher('velocity', Float32, queue_size=30)
steering_publisher = rospy.Publisher('steering_rad', Float32, queue_size=30)
# fake_distance_publisher = rospy.Publisher('person_depth', Float32, queue_size=10)

distances = list()
number_of_objects_list = list()
distance = 0.0
num_of_objects = 0
throttle_percentage = 0
steering_percentage = 0
theta = 0
app_throttle = 100
app_steering = 0
control_switch = 0
stop_sign = -1
max_throttle = 100

def distance_callback(msg):
    global throttle_percentage, stop_sign
    # num_of_objects = random.randrange(0,3)
    distance = round(msg.data, 2)
    # global distances, distance, number_of_objects_list, num_of_objects
    # distances.append(_distance)
    # number_of_objects_list.append(num_of_objects)
    # if len(distances) == 10:
    #     distances_counts = np.bincount(distances)
    #     distance = np.argmax(distances_counts)
    #     number_of_objects_count = np.bincount(number_of_objects_list)
    #     num_of_objects = np.argmax(number_of_objects_count)
    #     # distance = np.average(distances)
    #     distances = list()
    #     number_of_objects_list = list()

    if num_of_objects > 0: # check distance to nearest object
        if distance == 0 :
            # stop
            throttle_percentage = 0
        # object is 2~4m away
        elif distance == 100:
            # drive at throttle_percentage(dist) = 50 * dist - 100
            throttle_percentage = max_throttle
        
        else:
            throttle_percentage = max_throttle
        # object is too far away (further than 4m and closer by 5.5m )
        # elif distance <= 5.5:
        #     # drive at max throttle_percentage
        #     throttle_percentage = 100
        # # if distance is unknown
        # elif distance > 5.5:
        #     throttle_percentage = 100
    elif num_of_objects == 0:
        throttle_percentage = max_throttle
    
    if stop_sign == 0:
        throttle_percentage = 0
        # rospy.loginfo("Stop sign,\tthrottle% = " + str(throttle_percentage))
    else:
        pass
        # rospy.loginfo("Distance = " + str(distance) +
        #           " m,\tThrottle% = " + str(throttle_percentage) + 
        #          "%,\t#objects = " + str(num_of_objects))

def numobjects_callback(msg):
    global num_of_objects
    num_of_objects = msg.data

def stop_sign_callback(msg):
    global stop_sign
    global throttle_percentage
    stop_sign = msg.data

def app_throttle_callback(msg):
    global app_throttle
    app_throttle = msg.data

def siren_light_callback(msg):
    global control_switch
    control_switch = msg.data

def app_steering_callback(msg):
    global app_steering
    app_steering = msg.data

def theta_callback(msg):
    global theta
    theta = msg.data
    
def control_mux():
    global throttle_percentage
    global steering_percentage
    global theta
    rospy.init_node('control_mux', anonymous=True)
    rospy.Subscriber('person_depth', Float32, distance_callback, queue_size=10)
    rospy.Subscriber('persons_number', Float32, numobjects_callback)
    rospy.Subscriber('sign_depth', Float32, stop_sign_callback)
    rospy.Subscriber('ros_throttle_app', Int16, app_throttle_callback)
    rospy.Subscriber('ros_siren_light', Int16, siren_light_callback)
    rospy.Subscriber('ros_steering_app', Int16, app_steering_callback)
    rospy.Subscriber('theta', Float32, theta_callback)
    # rospy.spin()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # Mux logic (depends on siren light switch)
        if control_switch:
            throttle_percentage = app_throttle
            steering_percentage = app_steering
        else:
            steering_percentage = theta
        throttle_percentage = throttle_percentage/200
        # throttle_percentage = 0
        throttle_publisher.publish(throttle_percentage)
        steering_publisher.publish(steering_percentage)
        # fake_distance = random.random() * 6.0
        # fake_distance_publisher.publish(fake_distance)
        rate.sleep()

if __name__ == '__main__':
    control_mux()
