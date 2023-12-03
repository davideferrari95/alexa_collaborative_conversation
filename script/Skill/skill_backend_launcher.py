#!/usr/bin/env python3

import os, subprocess, signal, threading
import rclpy
from rclpy.node import Node

# Import Utilities
from utils import delete_pycache_folders

class SkillLauncher(Node):

    def __init__(self, node_name='skill_launcher'):

        # Initialize ROS Node
        super().__init__(node_name)

        # # Open ROS Skill Server in a Separate Thread
        # threading.Thread(target=lambda: rospy.init_node('skill_launcher', disable_signals=True)).start()

        # Declare Launch Parameters
        self.declare_parameter('launch_azure',    False)
        self.declare_parameter('launch_ngrok',    False)
        self.declare_parameter('launch_node_red', False)

        # Get Launch Parameters
        self.launch_azure    = self.get_parameter('launch_azure').get_parameter_value().bool_value
        self.launch_ngrok    = self.get_parameter('launch_ngrok').get_parameter_value().bool_value
        self.launch_node_red = self.get_parameter('launch_node_red').get_parameter_value().bool_value

        # Get Path to Package
        # rospack = rospkg.RosPack()
        # path = rospack.get_path('alexa_conversation')

        # Set Path to ngrok
        NGROK_PATH = '/home/davide/Documenti/Programmi/ngrok/'

        # Launch Node-RED, ngrok and Azure in Separate Terminals
        if self.launch_ngrok:    self.NGROK    = subprocess.Popen('gnome-terminal -e "./ngrok http 7071"', cwd=f'{NGROK_PATH}', shell=True)
        if self.launch_node_red: self.NODE_RED = subprocess.Popen('gnome-terminal -- "node-red"', shell=True)
        if self.launch_azure:    self.AZURE    = subprocess.Popen('func start', cwd=f'{path}/AzureFunctions/', shell=True)

        # Register Signal Handler
        signal.signal(signal.SIGINT, self.handle_signal)

    def handle_signal(self, sig, frame):

        # SIGINT (Ctrl+C)
        print("\nProgram Interrupted. Killing Opened Terminal...\n")

        # Kill Node-RED and ngrok
        if self.launch_ngrok:    os.system('killall node-red')
        if self.launch_node_red: os.system('killall ngrok')

        # Wait for Azure Functions
        if self.launch_azure: self.AZURE.wait()

        # Delete `__pycache__` Folders
        delete_pycache_folders(verbose=True)

        print("\nDone\n\n")
        exit(0)

if __name__ == '__main__':

    # Initialize ROS
    rclpy.init(None)

    # Initialize Skill Launcher
    launcher = SkillLauncher()
    
    # Spin Launcher Node
    while rclpy.ok(): rclpy.spin(launcher)
