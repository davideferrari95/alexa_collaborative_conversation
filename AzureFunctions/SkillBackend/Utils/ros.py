#!/usr/bin/env python3

import rclpy, time, sys
from rclpy.node import Node
from pathlib import Path

# Import Messages
from alexa_conversation.msg import VoiceCommand

# # Import Parent Folders
# sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/script')

# # Import Parent Folders
# sys.path.append(str(Path(__file__).resolve().parents[1] / "utils"))

# Import Command List
from command_list import *
from object_list import *

class SkillServer(Node):

    def __init__(self, node_name='skill_server'):

        # Initialize Node
        super().__init__(node_name)

        # ROS Publishers
        self.command_pub = self.create_publisher(VoiceCommand, '/alexa_conversation/voice_command', 1)

        time.sleep(1)

    def send_command(self, command, object=None):

        # Voice Command Message
        msg = VoiceCommand()
        msg.command = command
        msg.info = command_info[command]

        # Area Command if Defined
        msg.object = object if object in available_objects else ''

        self.command_pub.publish(msg)

def init_ros():

    # Initialize ROS
    rclpy.init()

    # Initialize Skill Server
    server = SkillServer()

    # Spin Server Node
    while rclpy.ok(): rclpy.spin_once(server)
