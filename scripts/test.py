#!/usr/bin/env python3
#!/home/filippo/miniconda3/envs/collaborative_robot_env/bin python3
import rospy
from std_msgs.msg import String,Bool
#from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
import numpy as np
import time

rospy.init_node('test')
finalPos=JointTrajectory
destinationPos=JointTrajectoryPoint()
destinationPos.time_from_start = rospy.Duration(5)
ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
time.sleep(5)



defaultPos=[-5.045, -1.571, 1.981, -1.993, -1.641, -0.138]
destinationPos.positions=defaultPos
ur10Pub.publish(destinationPos)


pos1=[-5.653, -1.270, 2.514, -2.834, -1.552, -0.13755304018129522]
pos2=[-4.727609459553854, -0.6143368047526856, 1.1586445013629358, -2.227729459802145, -1.549403492604391, -0.13768464723695928]



destinationPos.positions=defaultPos
ur10Pub.publish(destinationPos)
flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
if flag.data is not True:
    raise Exception("Sorry, An exception occurred") 
    #pubgripper

destinationPos.positions=pos1
ur10Pub.publish(destinationPos)
flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
if flag.data is not True:
    raise Exception("Sorry, An exception occurred") 
    #pubgripper

destinationPos.positions=pos2
ur10Pub.publish(destinationPos)