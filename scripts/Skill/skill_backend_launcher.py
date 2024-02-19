#!/usr/bin/env python3

import os, subprocess, signal, threading
import rclpy
from rclpy.node import Node

# Import Utilities
from utils import get_ros2_workspace_path, delete_pycache_folders

class SkillLauncher(Node):

    def __init__(self, node_name='skill_launcher'):

        # Initialize ROS Node
        super().__init__(node_name)

        # Declare Launch Parameters
        self.declare_parameter('launch_azure',    False)
        self.declare_parameter('launch_ngrok',    False)
        self.declare_parameter('launch_node_red', False)
        self.declare_parameter('package_path',    '')

        # Get Launch Parameters
        self.launch_azure    = self.get_parameter('launch_azure').get_parameter_value().bool_value
        self.launch_ngrok    = self.get_parameter('launch_ngrok').get_parameter_value().bool_value
        self.launch_node_red = self.get_parameter('launch_node_red').get_parameter_value().bool_value

        # Package, ROS2 Workspace Path and Home Path
        PACKAGE_PATH = self.get_parameter('package_path').get_parameter_value().string_value
        WORKSPACE_PATH = get_ros2_workspace_path()
        HOME_PATH      = os.path.expanduser('~')

        # Assert Paths Exist
        assert os.path.exists(PACKAGE_PATH),   f'Package Path {PACKAGE_PATH} does not exist'
        assert os.path.exists(WORKSPACE_PATH), f'ROS2 Workspace Path {WORKSPACE_PATH} does not exist'

        # Launch Node-RED Command
        NODE_RED_COMMAND = (
            f"source {WORKSPACE_PATH}/install/setup.bash && "
            "source /opt/ros/foxy/setup.bash && "
            "export ROS_DOMAIN_ID=10 && "
            f"node-red -u {HOME_PATH}/.node-red-2"
        )

        # Launch Node-RED, ngrok and Azure in Separate Terminals
        if self.launch_ngrok:    self.NGROK    = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", 'ngrok http 7071'])
        if self.launch_node_red: self.NODE_RED = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", NODE_RED_COMMAND])
        if self.launch_azure:    self.AZURE    = subprocess.Popen('func start', cwd=f'{PACKAGE_PATH}/AzureFunctions/', shell=True)

        # Register Signal Handler
        signal.signal(signal.SIGINT, self.handle_signal)

    def handle_signal(self, sig, frame):

        # SIGINT (Ctrl+C)
        print("\nProgram Interrupted. Killing Opened Terminal...\n")

        # Kill Node-RED and ngrok
        if self.launch_ngrok:    os.system('killall ngrok')
        if self.launch_node_red: os.system('killall node-red')

        # Wait for Azure Functions
        if self.launch_azure: self.AZURE.wait()

        # Delete `__pycache__` Folders
        delete_pycache_folders(verbose=True)

        print("\nDone\n\n")
        exit(0)

if __name__ == '__main__':

    # Initialize ROS
    rclpy.init()

    # Initialize Skill Launcher
    launcher = SkillLauncher()
    
    # Spin Launcher Node
    while rclpy.ok(): rclpy.spin(launcher)
