#!/usr/bin/env python3
import rospy, time

# Import Messages
from alexa_conversation.msg import VoiceCommand
from Utils.command_list import command_info, available_areas

# Open ROS Skill Server
rospy.init_node('skill_server', disable_signals=True)
time.sleep(1)

# ROS Publishers
command_pub = rospy.Publisher('/multimodal_fusion/voice_command', VoiceCommand, queue_size=1)

def send_command(command, area=None, wait_time=None):

    # Voice Command Message
    msg = VoiceCommand()
    msg.command = command
    msg.info = command_info[command]

    # Area Command if Defined
    msg.area = area if area in available_areas else ''

    # Wait Time if Defined
    if wait_time is not None: msg.area = wait_time

    command_pub.publish(msg)
