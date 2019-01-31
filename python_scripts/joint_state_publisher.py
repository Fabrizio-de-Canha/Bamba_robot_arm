#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
import os

def joint_state_publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher', anonymous = True)
    rate = rospy.Rate(50)
    target = np.array([0, 0, 0, 0, 0, 0.35])
    state = target
    np.savetxt('Desktop/cv_files/target.txt', target)
    k = 75
    while not rospy.is_shutdown():
        try:
            target = np.loadtxt('Desktop/cv_files/target.txt')
        except ValueError:
            continue
        sensor_msg = state
        try:
            if (target.round(2) != state.round(2)).any():
                vector = (target - state)/k
                for i in range(k):
                    sensor_msg += vector
                    rospy.loginfo(list(sensor_msg))
                    pub.publish(position = list(sensor_msg))
                    rate.sleep()
                rospy.sleep(0.2)
        except AttributeError:
            continue
        state = sensor_msg
        rospy.loginfo(list(sensor_msg))
        pub.publish(position = list(sensor_msg))
        rate.sleep()

joint_state_publisher()