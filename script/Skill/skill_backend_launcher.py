#!/usr/bin/env python3
import subprocess, os, threading, signal
import rospy, rospkg

# Get Path to Package
rospack = rospkg.RosPack()
path = rospack.get_path('alexa_conversation')

# Set Path to ngrok
NGROK_PATH = '/home/davide/Documenti/Programmi/ngrok/'

# Launch Commands
commands = [
    f'gnome-terminal -- "node-red"',
    f'gnome-terminal -e "/.{NGROK_PATH}/ngrok http 5000"'
]

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_launcher', disable_signals=True)).start()

# Launch Node-RED and ngrok in a Separate Thread Terminal
processes = [subprocess.Popen(cmd, shell=True) for cmd in commands]
NODE_RED_ID, NGROK_ID = processes[0].pid, processes[1].pid
print(f'Node-RED PID: {NODE_RED_ID}')
print(f'ngrok PID: {NGROK_ID}')

# Launch Azure Functions
AZURE = subprocess.Popen('func start -p 5050', cwd=f'{path}/AzureFunctions/', shell=True)

def handle_signal(sig, frame):

    # SIGINT (Ctrl+C)
    print("\nProgram Interrupted. Killing Opened Terminal...\n")

    # Kill Node-RED and ngrok
    processes[0].wait()
    processes[1].wait()
    # os.kill(NODE_RED_ID, signal.SIGINT)
    # os.kill(NGROK_ID,    signal.SIGINT)

    # Kill Azure Functions
    AZURE.wait()

    print("Done\n")
    exit(0)

# Register Signal Handler
signal.signal(signal.SIGINT, handle_signal)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while not rospy.is_shutdown(): pass
