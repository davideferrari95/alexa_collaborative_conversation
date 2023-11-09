#!/usr/bin/env python3
import rospy, rospkg, time, sys

# Import Messages
from alexa_conversation.msg import VoiceCommand

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/script')

# Import Command List
from utils.command_list import *
from utils.object_list  import *

# Open ROS Skill Server
rospy.init_node('skill_server', disable_signals=True)
time.sleep(1)

# ROS Publishers
command_pub = rospy.Publisher('/alexa_conversation/voice_command', VoiceCommand, queue_size=1)

def send_command(command, object=None):

    # Voice Command Message
    msg = VoiceCommand()
    msg.command = command
    msg.info = command_info[command]

    # Area Command if Defined
    msg.object = object if object in available_objects else ''

    command_pub.publish(msg)
