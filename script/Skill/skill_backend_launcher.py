#!/usr/bin/env python3
import subprocess, os, threading, signal
import rospy, rospkg

# Get Path to Package
rospack = rospkg.RosPack()
path = rospack.get_path('alexa_conversation')

# Set Path to ngrok
NGROK_PATH = '/home/davide/Documenti/Programmi/ngrok'

# Launch Commands
commands = [
    f'gnome-terminal -- "node-red"',
    f'gnome-terminal -e "/.{NGROK_PATH}/ngrok http 5000"'
]

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_launcher', disable_signals=True)).start()

# Launch Node-RED and ngrok in a Separate Thread Terminal
processes = [subprocess.Popen(cmd, shell=True) for cmd in commands]
# processes = [subprocess.Popen(cmd, shell=True, cwd=f'{NGROK_PATH}') for cmd in commands]

# threading.Thread(target=lambda: subprocess.Popen(['gnome-terminal -- "node-red"'], shell=True)).start()
NODE_RED_ID = processes[0].pid
NGROK_ID = processes[1].pid
print(f"Node-RED PID: {NODE_RED_ID}")

# Launch ngrok in a Separate Thread Terminal
# NGROK = threading.Thread(target=lambda: subprocess.Popen(['gnome-terminal -e "./ngrok http 5000"'], preexec_fn=os.setsid, cwd=f'{NGROK_PATH}', shell=True)).start()
# NGROK_ID = os.getpid()
print(f"ngrok PID: {NGROK_ID}")

# AZURE = subprocess.Popen('func start -p 7075', shell=True, cwd=f'{path}/AzureFunctions/')
# AZURE = subprocess.Popen('func start', shell=True, cwd=f'{path}/AzureFunctions/')

def handle_signal(signal, frame):

    # SIGINT (Ctrl+C)
    print("\nProgram Interrupted. Killing Opened Terminal...\n")

    # Kill Node-RED
    # os.kill(NODE_RED.pid, signal.SIGKILL)

    # Kill ngrok
    
    # Kill Azure Functions
    # AZURE.wait()

    print("Done\n")
    exit(0)

# Register Signal Handler
signal.signal(signal.SIGINT, handle_signal)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while not rospy.is_shutdown(): pass
