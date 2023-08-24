#!/usr/bin/env python3
import subprocess, os, threading, signal
import rospy, rospkg

# Import Utilities
from utils import delete_pycache_folders

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_launcher', disable_signals=True)).start()

# Get Launch Parameters
launch_azure    = rospy.get_param('/skill_backend_launcher/launch_azure', False)
launch_ngrok    = rospy.get_param('/skill_backend_launcher/launch_ngrok', False)
launch_node_red = rospy.get_param('/skill_backend_launcher/launch_node_red', False)

# Get Path to Package
rospack = rospkg.RosPack()
path = rospack.get_path('alexa_conversation')

# Set Path to ngrok
NGROK_PATH = '/home/davide/Documenti/Programmi/ngrok/'

# Launch Node-RED, ngrok and Azure in Separate Terminals
if launch_ngrok:    NGROK    = subprocess.Popen('gnome-terminal -e "./ngrok http 7071"', cwd=f'{NGROK_PATH}', shell=True)
if launch_node_red: NODE_RED = subprocess.Popen('gnome-terminal -- "node-red"', shell=True)
if launch_azure:    AZURE    = subprocess.Popen('func start', cwd=f'{path}/AzureFunctions/', shell=True)

def handle_signal(sig, frame):

    # SIGINT (Ctrl+C)
    print("\nProgram Interrupted. Killing Opened Terminal...\n")

    # Kill Node-RED and ngrok
    if launch_ngrok:    os.system('killall node-red')
    if launch_node_red: os.system('killall ngrok')

    # Wait for Azure Functions
    if launch_azure:    AZURE.wait()

    # Delete `__pycache__` Folders
    delete_pycache_folders(verbose=True)

    print("\nDone\n\n")
    exit(0)

# Register Signal Handler
signal.signal(signal.SIGINT, handle_signal)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while not rospy.is_shutdown(): pass
