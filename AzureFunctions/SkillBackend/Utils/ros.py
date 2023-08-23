#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32

# Open ROS Skill Server
rospy.init_node('skill_server', disable_signals=True)

# ROS Publishers
intent_publisher = rospy.Publisher('intent', Int32, queue_size=1)
intent_info_publisher = rospy.Publisher('intent_info', String, queue_size=1)
