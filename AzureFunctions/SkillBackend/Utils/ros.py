#!/usr/bin/env python3

import rclpy, time, sys
from rclpy.node import Node

# Import Messages
from alexa_conversation.msg import VoiceCommand

# Import Parent Folders
from pathlib import Path
sys.path.append(f'{str(Path(__file__).resolve().parents[3])}/scripts/utils')


# Initialize ROS
rclpy.init()
node = Node('skill_server')

# ROS Publishers
command_pub = node.create_publisher(VoiceCommand, '/alexa_conversation/voice_command', 1)

# Wait for Initialization
time.sleep(1)

def send_command(command, object=None):

    # Voice Command Message
    msg = VoiceCommand()
    msg.command = command
    msg.object = object

    command_pub.publish(msg)
