#!/usr/bin/env python3
#!/home/filippo/miniconda3/envs/collaborative_robot_env/bin python3
import subprocess

import time
azure_launch='func start'
ngrok_launch='ngrok http 7071'
node_red="node-red"

NODE_RED=subprocess.Popen(node_red, shell=True)
time.sleep(2)
AZURE = subprocess.Popen(azure_launch, shell=True, cwd="../AzureFunctions/")
time.sleep(5)
NGROK = subprocess.Popen(ngrok_launch, shell=True)
time.sleep(5)



NODE_RED.wait()
AZURE.wait()
NGROK.wait()

