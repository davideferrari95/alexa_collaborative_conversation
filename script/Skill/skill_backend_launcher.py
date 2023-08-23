#!/usr/bin/env python3
import subprocess, os, threading, signal
import rospy, rospkg

# Get Path to Package
rospack = rospkg.RosPack()
path = rospack.get_path('alexa_conversation')

# Set Path to ngrok
NGROK_PATH = '/home/davide/Documenti/Programmi/ngrok/'

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_launcher', disable_signals=)).start()

# Launch Node-RED and ngrok in a Separate Thread Terminal
NODE_RED = subprocess.Popen('gnome-terminal -- "node-red"', shell=True)
NGROK    = subprocess.Popen('gnome-terminal -e "./ngrok http 5000"', cwd=f'{NGROK_PATH}', shell=True)
NODE_RED_ID, NGROK_ID = NODE_RED.pid, NGROK.pid
print(f'Node-RED PID: {NODE_RED_ID}\nngrok PID: {NGROK_ID}')

# Launch Azure Functions
AZURE = subprocess.Popen('func start -p 5050', cwd=f'{path}/AzureFunctions/', shell=True)

def handle_signal(sig, frame):

    # SIGINT (Ctrl+C)
    print("\nProgram Interrupted. Killing Opened Terminal...\n")

    # Kill Node-RED and ngrok
    # NODE_RED.wait()
    # NGROK.wait()

    # Kill Azure Functions
    # AZURE.wait()

    print("Done\n")
    exit(0)

# Register Signal Handler
signal.signal(signal.SIGINT, handle_signal)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while not rospy.is_shutdown(): pass
