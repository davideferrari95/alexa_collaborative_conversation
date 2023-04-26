#!/usr/bin/env python3
#!/home/filippo/miniconda3/envs/collaborative_robot_env/bin python3
from operator import truediv
import rospy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Pose
#from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest
import numpy as np
import subprocess
import time
import os
mountPos=[1.4624409675598145, -1.614187856713766, 1.8302066961871546, -1.795131345788473, -1.5152104536639612, -0.09420425096620733]
defaultPos=[-0.01490956941713506, -2.0275737247862757, 2.4103458563434046, -1.953010698358053, -1.5686352888690394, -0.05916053453554326]
#{'object':[posizione, gia preso,prendere con la calamita(da rimuovere),area,posizione di rlascio]}
tools={
        'allenkey':[[1.209991455078125, -1.1305053991130372, 1.8814151922809046, -2.251266141931051, -1.6082032362567347, -0.2987459341632288],False,True,"green", [1.531172752380371, -1.7695633373656214, 2.637843910847799, -2.508017202416891, -1.4946301619159144, 0.0023927688598632812]],
        #'allenkey 6':[[-2.5996604601489466, -1.3714750570109864, 2.477001969014303, -2.6082192860045375, -1.5663569609271448, 0.5769932270050049],False,True,"yellow",[-4.400205437337057, -1.4546173375895997, 2.468898598347799, -2.5122953854002894, -1.5861404577838343, 0.494495153427124]],
        'bottle':[[],False,True,"yellow"],
        # 'flat screwdrivers':[[0.964266300201416, -1.131260709171631, 1.9902237097369593, -2.3955232105650843, -1.503171745930807, 2.557365894317627],False,True,"green",[1.4655537605285645, -1.6076885662474574, 2.6418989340411585, -2.5798241100706996, -1.5709250609027308, -0.07017022768129522]],
        'flat screwdriver':[[0.964266300201416, -1.131260709171631, 1.9902237097369593, -2.3955232105650843, -1.503171745930807, 2.557365894317627],False,True,"green",[1.4655537605285645, -1.6076885662474574, 2.6418989340411585, -2.5798241100706996, -1.5709250609027308, -0.07017022768129522]],
        #'cross screwdrivers':[[-5.287935558949606, -1.1349046987346192, 2.005863968526022, -2.4497410259642542, -1.5572331587420862, -0.577982250844137],False,True,"green",[-4.470887962971823, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        # 'multifunction screwdrivers':[[-2.6812194029437464, -1.2024553579143067, 2.2298858801471155, -2.6013132534422816, -1.602584187184469, 1.9855148792266846],False,True,"yellow",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'multifunction screwdriver':[[-2.6812194029437464, -1.2024553579143067, 2.2298858801471155, -2.6013132534422816, -1.602584187184469, 1.9855148792266846],False,True,"yellow",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'wrench':[[-2.403022352849142, -1.2639001172832032, 2.2682769934283655, -2.5386020145811976, -1.617248837147848, -0.9075310865985315],False,True,"yellow",[1.40895414352417, -1.4546173375895997, 2.468898598347799, -2.5122953854002894, -1.5861404577838343, 0.494495153427124]],
        # 'allen screws':[ [-2.711510960255758, -1.3636163038066407, 2.438629929219381, -2.6094161472716273, -1.5834782759295862, -1.1168530623065394],False,True,"green",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'allen screw':[ [-2.711510960255758, -1.3636163038066407, 2.438629929219381, -2.6094161472716273, -1.5834782759295862, -1.1168530623065394],False,True,"green",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        # 'cross screws':[[0.8717985153198242, -1.406376675968506, 2.409961525593893, -2.5774790249266566, -1.5449555555926722, -0.5736220518695276],False,True,"green",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'cross screw':[[0.8717985153198242, -1.406376675968506, 2.409961525593893, -2.5774790249266566, -1.5449555555926722, -0.5736220518695276],False,True,"green",[1.40895414352417, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'main part':[[-1.2570365110980433, -1.2826450628093262, 2.300680939351217, -2.5817023716368617, -1.5601103941546839, 0.33130860328674316],False,True,"red",[1.40895414352417, -1.5081356328776856, 2.4545114676104944, -2.3878962002196253, -1.6063340345965784, 0.30598998069763184]],
        'part one':[[1.1107916831970215, -1.3539403241923829, 2.3259361425982874, -2.5088564358153285, -1.5601943174945276, -0.4494693914996546],False,True,"red",[1.40895414352417, -1.5960480175414027, 2.566508118306295, -2.5372234783568324, -1.567256275807516, -0.013668362294332326]],
        'part two':[[-1.5153682867633265, -1.259366826420166, 2.259684387837545, -2.5406629047789515, -1.6742079893695276, 1.7244203090667725],False,True,"red",[1.40895414352417, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
        'part three':[[-1.3056681791888636, -1.150644139652588, 2.0502451101886194, -2.4424320660033167, -1.6117499510394495, 1.768970251083374],False,True,"red",[1.40895414352417, -1.7439519367613734, 2.6499956289874476, -2.4990889034666957, -1.5576403776751917, -0.20911199251283819]],
        'part four':[[-2.51639968553652, -1.4876607221416016, 2.597243134175436, -2.6463076076903285, -1.5805171171771448, -0.891555134450094],False,True,"red",[1.40895414352417, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
        'support':[[-1.4374983946429651, -1.3897693914226075, 2.4993980566607874, -2.647731443444723, -1.6097844282733362, 0.1549820899963379],False,True,"red",[1.40895414352417, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
        'long screw':[[1.160428524017334, -1.2264259618571778, 1.9600589911090296, -2.2868124447264613, -1.5689590612994593, -0.3421538511859339],False,True,"red",[1.40895414352417, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
}

'''tools={
        'allenkey':[[-5.085838619862692, -1.1781070989421387, 1.9547651449786585, -2.4125381908812464, -1.569042984639303, -0.3517831007586878],False,True,"green", [-4.425851647053854, -1.7334186039366664, 2.5746267477618616, -2.3899313412108363, -1.5663211981402796, -0.15248233476747686]],
        #'allenkey 6':[[-2.5996604601489466, -1.3714750570109864, 2.477001969014303, -2.6082192860045375, -1.5663569609271448, 0.5769932270050049],False,True,"yellow",[-4.400205437337057, -1.4546173375895997, 2.468898598347799, -2.5122953854002894, -1.5861404577838343, 0.494495153427124]],
        'bottle':[[],False,True,"yellow"],
        'cross screwdrivers':[[-5.287935558949606, -1.1349046987346192, 2.005863968526022, -2.4497410259642542, -1.5572331587420862, -0.577982250844137],False,True,"green",[-4.470887962971823, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        #'flat screwdriver':[[],False,True],
        'multifunction screwdrivers':[[-2.6812194029437464, -1.2024553579143067, 2.2298858801471155, -2.6013132534422816, -1.602584187184469, 1.9855148792266846],False,True,"yellow",[-4.470887962971823, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'wrench':[[-2.490372959767477, -1.2064684194377442, 2.2157657782184046, -2.5080534420409144, -1.5445721785174769, 0.637559175491333],False,True,"yellow",[-4.400205437337057, -1.4546173375895997, 2.468898598347799, -2.5122953854002894, -1.5861404577838343, 0.494495153427124]],
        'allen screws':[[-5.521478478108541, -1.3992985051921387, 2.3638275305377405, -2.5150400600829066, -1.585672680531637, -0.5784733931170862],False,True,"green",[-4.470887962971823, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'cross screws':[[-5.344341103230612, -1.4137299817851563, 2.4048622290240687, -2.5191508732237757, -1.5055554548846644, -0.5796950499164026],False,True,"green",[-4.470887962971823, -1.5451415342143555, 2.568056885396139, -2.5590411625304164, -1.5783827940570276, -0.007795159016744435]],
        'main part':[[-1.2570365110980433, -1.2826450628093262, 2.300680939351217, -2.5817023716368617, -1.5601103941546839, 0.33130860328674316],False,True,"red",[-4.408040825520651, -1.5081356328776856, 2.4545114676104944, -2.3878962002196253, -1.6063340345965784, 0.30598998069763184]],
        'part one':[[-1.4806769529925745, -1.38058693826709, 2.453553024922506, -2.6321827373900355, -1.561237637196676, 0.09875798225402832],False,True,"red",[-4.473523441945211, -1.5960480175414027, 2.566508118306295, -2.5372234783568324, -1.567256275807516, -0.013668362294332326]],
        'part two':[[-1.5153682867633265, -1.259366826420166, 2.259684387837545, -2.5406629047789515, -1.6742079893695276, 1.7244203090667725],False,True,"red",[-4.525054756795065, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
        'part three':[[-1.3056681791888636, -1.150644139652588, 2.0502451101886194, -2.4424320660033167, -1.6117499510394495, 1.768970251083374],False,True,"red",[-4.576003853474752, -1.7439519367613734, 2.6499956289874476, -2.4990889034666957, -1.5576403776751917, -0.20911199251283819]],
        'part four':[[-1.3056681791888636, -1.150644139652588, 2.0502451101886194, -2.4424320660033167, -1.6117499510394495, 1.768970251083374],False,True,"red",[-4.525054756795065, -1.5805145702757777, 2.5785301367389124, -2.584334989587301, -1.6155598799334925, 0.019095420837402344]],
        
}'''

'''parts={
    'main part':[[-5.201291386281149, -1.0971382421306153, 1.7500017325030726, -2.2769295177855433, -1.531431023274557, -0.5164049307452601],False,True,"red",[-4.559031788502828, -1.674770017663473, 2.4509084860431116, -2.3027922115721644, -1.5977194944964808, 0.07538151741027832]],
    'part one':[[0,0,0,0,0,0],False,True,"red"],
    'part two':[[0,0,0,0,0,0],False,True,"red"],
    'part three':[[0,0,0,0,0,0],False,True,"red"],
    'part four':[[0,0,0,0,0,0],False,True,"red"],
    'screw_m4':[[0,0,0,0,0,0],False,True,"red"]
}'''

tips={
    'cross':[ [-1.4764750639544886, -1.1297262471965333, 2.0701282660113733, -2.4896685085692347, -1.5524371306048792, -1.4404662291156214],False,True,"red",[1.3014202117919922, -1.7242261372008265, 2.7010887304889124, -2.570982118646139, -1.549427334462301, -1.6277974287616175]],
    'flat':[[-1.5072897116290491, -1.1079743665507813, 2.013167206441061, -2.472867628137106, -1.625420872365133, 1.7687547206878662],False,True,"red",[1.3014202117919922, -1.7242261372008265, 2.7010887304889124, -2.570982118646139, -1.549427334462301, -1.6277974287616175]],
    'torox':[[-1.5594146887408655, -1.0772250455668946, 1.9655197302447718, -2.4736105404295863, -1.625768009816305, 1.768730878829956],False,True,"red",[1.3014202117919922, -1.7242261372008265, 2.7010887304889124, -2.570982118646139, -1.549427334462301, -1.6277974287616175]],
}

side = {'left':[1.5353798866271973, -1.064389244919159, 1.1746123472796839, -1.8566700420775355, -1.6348355452166956, -0.3233879248248499],
        'right':[1.5028204917907715, -1.7718893490233363, 1.9573467413531702, -1.765754838983053, -1.555542294179098, -0.24763137498964483]
        }


flagOrientation=False
orientations = { 'vertical':[1.2197179794311523, -1.0376113218120118, 2.405584160481588, -3.100814481774801, -0.09345561662782842, -1.4305699507342737],
                'horizontal': [1.4624528884887695, -1.614175935784811, 1.830242935811178, -1.795131345788473, -1.5152104536639612, -0.09416801134218389],
                'frontal':[1.4624409675598145, -1.6141401729979457, 1.8302906195269983, -1.794664045373434, -1.5151150862323206, -1.586318318043844]
                }

selectedSide='right'
currentStep=""
mounting=False

steps={
    "first":[False,["part one","part two,cross screwdrivers","cross screws"]],
    "second":[False,["part three","part four","allenkey","allen screws"]],
    "third":[False,['long screw','wrench']],
    "fourth":[False,[""]]
    }
###
tablePos=[]
tts=""
required=[]
requiredString=""
needTool=""
needScrew=""
lastTool=""
mountTool=""
def randInt():
    x = np.random.randint(100)
    return x

rospy.init_node('expManager')

# destinationPos=JointState
destinationPos=JointTrajectoryPoint()

destinationPos.time_from_start = rospy.Duration(0)
destinationPos.velocities = [0.4]
launchPub=rospy.Publisher('launch',Bool,queue_size=1)
toolPub = rospy.Publisher('pubtool',String,queue_size=10)
errorPub = rospy.Publisher('Error',String,queue_size=100)
orientationPub = rospy.Publisher('pubOrientation',String,queue_size=10)
responsePub= rospy.Publisher('pubResponse',String,queue_size=10)
screwPub= rospy.Publisher('screw',String,queue_size=10)
bottlePub= rospy.Publisher('pubbottle',String,queue_size=10)
eventPub= rospy.Publisher('events',String,queue_size=10)

ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
ur10PubCartesian=rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command',CartesianPoint,queue_size=10)
ttsPub=rospy.Publisher('tts',String,queue_size=100)

gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)
gripper_req = RobotiQGripperControlRequest()
cartesian_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)
rate = rospy.Rate(500)

def gripping(position):
    gripper_req.position=position
    rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
    gripper_response = gripper_srv(gripper_req)
    '''if gripper_response.status != "2":
        raise Exception("Sorry, An exception occurred")'''

def multihandover(vector):
        for i in vector:
            if i not in tools.keys():
                
                destinationPos.positions=tips[i][0]
                
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                time.sleep(1)
                gripping(0)

                cartesian_req= GetForwardKinematicRequest()
                cartesian_req.joint_position=tips[i][0]
                rospy.wait_for_service('ur_rtde/getFK')
                cartesian_response = cartesian_srv(cartesian_req)
                print(cartesian_response)

                cartesian_pose = CartesianPoint()
                cartesian_response.tcp_position.position.z+=0.20
                cartesian_pose.cartesian_pose = cartesian_response.tcp_position
                cartesian_pose.velocity = 0.08
                ur10PubCartesian.publish(cartesian_pose)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                destinationPos.positions=[-0.32539683977235967, -1.8089076481261195, 2.42702562013735, -2.137054582635397, -1.4966190497027796, -3.4405556360827845]
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                destinationPos.positions=tips[i][4]
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                gripping(100)
                destinationPos.positions=defaultPos
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                time.sleep(1)
            else:
                destinationPos.positions=tools[i][0]
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                time.sleep(1)
                gripping(0)


                cartesian_req= GetForwardKinematicRequest()
                cartesian_req.joint_position=tools[i][0]
                rospy.wait_for_service('ur_rtde/getFK')
                cartesian_response = cartesian_srv(cartesian_req)
                print(cartesian_response)

                cartesian_pose = CartesianPoint()
                cartesian_response.tcp_position.position.z+=0.20
                cartesian_pose.cartesian_pose = cartesian_response.tcp_position
                cartesian_pose.velocity = 0.08
                ur10PubCartesian.publish(cartesian_pose)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 

                destinationPos.positions=[-0.32539683977235967, -1.8089076481261195, 2.42702562013735, -2.137054582635397, -1.4966190497027796, -3.4405556360827845]
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                destinationPos.positions=tools[i][4]
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                gripping(100)
                destinationPos.positions=defaultPos
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                time.sleep(1)
        
def handover2(position,side,grip):

    destinationPos.positions=[-1.2727144400226038, -1.5570243161967774, 2.150663201008932, -2.124763151208395, -1.5115569273578089, 0.2871372699737549]
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)
    destinationPos.positions=position
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)
    gripping(grip)

    cartesian_req= GetForwardKinematicRequest()
    cartesian_req.joint_position=position
    rospy.wait_for_service('ur_rtde/getFK')
    cartesian_response = cartesian_srv(cartesian_req)
    print(cartesian_response)

    cartesian_pose = CartesianPoint()
    cartesian_response.tcp_position.position.z+=0.20
    cartesian_pose.cartesian_pose = cartesian_response.tcp_position
    cartesian_pose.velocity = 0.08
    ur10PubCartesian.publish(cartesian_pose)

    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)

    
    destinationPos.positions=side
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)

def handover3(position,side,grip):
    destinationPos.positions=position
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)
    gripping(grip)


    destinationPos.positions=side
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)

def handover(position,side,grip):
    destinationPos.positions=position
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)
    gripping(grip)

    cartesian_req= GetForwardKinematicRequest()
    cartesian_req.joint_position=position
    rospy.wait_for_service('ur_rtde/getFK')
    cartesian_response = cartesian_srv(cartesian_req)
    print(cartesian_response)

    cartesian_pose = CartesianPoint()
    cartesian_response.tcp_position.position.z+=0.20
    cartesian_pose.cartesian_pose = cartesian_response.tcp_position
    cartesian_pose.velocity = 0.08
    ur10PubCartesian.publish(cartesian_pose)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)

    destinationPos.positions=side
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)
 
def returningPosition():
    destinationPos.positions=defaultPos
    ur10Pub.publish(destinationPos)

def reverseHandover(position,side):
    destinationPos.positions=side
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    #pubgripper

    destinationPos.positions=position
    ur10Pub.publish(destinationPos)
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    #pubgripper

    destinationPos.position=defaultPos
    ur10Pub.publish(destinationPos)

#callback per cambiare il lato
def changeSide():
    if selectedSide=="left" :
        selectedSide="right"
    else:
        selectedSide="left"   
    destinationPos.positions=side[selectedSide]
    ur10Pub.publish(destinationPos)     

#callback per prendere le punte
def tipCallBack(tip):
    if tip.data in tips.keys():
        i=tip.data
        time.sleep(1)
        handover(tips[i][0],side[selectedSide],0)
        global tablePos
        tablePos=tips[i][4]
        responsePub.publish('1')

#invia tool a robot
def toolCallBack(tool):
    print ('ho ricevuto: ', tool.data)
    #conta quanti oggetti ci sono con lo stesso nome
    multiple=0
    multiple_tool=""
    #controllo che il montaggio non stia avvenendo
    if mounting==False:
        for i in tools.keys():
            if tool.data in i:
                multiple_tool=multiple_tool+ "," +i
                multiple+=1
        #oggetto singolo        
        if multiple==1:
            for i in tools.keys():
                if tool.data==i:
                    msg_str= tools[i]
                    #controllo che l'oggetto non sia gia stato preso
                    if tools[i][1] == False:
                        print(tools[i][1])
                        #controllo che l'oggetto sia prendibile con il magnete
                        if tools[i][2] == True:
                            rospy.loginfo(msg_str)
                            tools[i][1] = True
                            toolPub.publish(msg_str)
                            global tablePos
                            tablePos=tools[i][4] 
                            global lastTool
                            lastTool=i
                            if tool.data == "main part":
                                handover2(tools[i][0],side[selectedSide],0)
                            else:    
                                handover(tools[i][0],side[selectedSide],0)
                            responsePub.publish('1')
                            
                        #errore oggetto non prendibile con la calamita
                        else:
                            errorPub.publish("object not taken") 
                    #errore oggetto gia preso
                    else: 
                        errorPub.publish("object already taken")
                        
        #errore oggetto multiplo                
        elif multiple >1:
            
            errorPub.publish("multiple object")  
        #errore oggetto mancante               
        elif multiple==0: 
            if "screwdriver" in tool.data:
                errorPub.publish('alexa ask collaborative tutorial where is the cross screwdriver') 
            else:
                errorPub.publish("object missing")
               
    #errore durante il montaggio e impossibilità del robot a muoversi
    else:
        errorPub.publish('mounting error')
        global mountTool
        mountTool=tool.data       

#lato 
def sideCallBack(side):
    print ('ho ricevuto: ', side.data) 
    if side.data in side.keys():
        selectedSide=side.data 

#orientamento pezzo
def orientationCallBack(orientation):
    print ('ho ricevuto: ', orientation.data)
    if 'default' in orientation.data:

        destinationPos.positions=orientations['horizontal']
        ur10Pub.publish(destinationPos)
        
    elif orientation.data in orientations.keys():
        destinationPos.positions=orientations[orientation.data]
        ur10Pub.publish(destinationPos)

#risposta che hai preso l'oggetto
def responseCallBack(msg):
    print ('ho ricevuto: ', msg.data)
    if "other hand" in msg.data:
        changeSide()
        selectedSide="right"
        responsePub.publish('1')
        time.sleep(5)
        responsePub.publish(msg.data)
    elif "put here" in msg.data or "table" in msg.data:
        handover3(tablePos,defaultPos,100)
        time.sleep(1)
        gripping(100)
    else:        
        gripping(100)
        time.sleep(1)
        returningPosition()
    flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
    if flag.data is not True:
        raise Exception("Sorry, An exception occurred") 
    time.sleep(1)

    #controllo se le viti non sono ancora state prese
    global lastTool
    if len(lastTool) > 0:
        if tools['allen screw'][1] == False and 'allenkey' in lastTool:
            x = randInt()
            if x>=50:
                time.sleep(5)
                eventPub.publish('take screws')
                global needScrew
                needScrew='allen screw'
        elif tools['cross screw'][1] == False and 'cross' in lastTool:
            x = randInt()
            if x>=50:
                time.sleep(5)
                eventPub.publish('take screw') 
                needScrew='cross screw'
        elif "multifunction" in lastTool:
                time.sleep(1)
                eventPub.publish("cross") 

        elif lastTool == "part three" and tools['part four'][1] == False:
            eventPub.publish('alexa ask collaborative tutorial where is the part four')  
            global needTool
            needTool="part four" 
            
            lastTool=""
            
            time.sleep(5) 
        elif lastTool == "part four" and tools['part three'][1] == False:
            eventPub.publish('alexa ask collaborative tutorial where is the part three')  
            needTool="part three" 
            
            lastTool=""
            
            time.sleep(5)                  
        print(lastTool)
    
#comnferma per le viti
def confirmCallBack(msg):
    print ('ho ricevuto: ', msg.data)
    if msg.data != "no":
        if "cross" in needScrew:
            tools['cross screws'][1]= True 
        elif "allen" in needScrew:
            tools['allen screws'][1]= True
        handover(tools[needScrew][0],side[selectedSide],0)
        responsePub.publish('1')    
       
# conferma
def bottleCallBack(msg):
    print ('ho ricevuto: ', msg.data)
    if msg.data != "no":
        destinationPos.positions=tools['bottle'][0]
        ur10Pub.pubblish(destinationPos)

#cambia stato si montataggio
def mountingCallBack(msg):
    global mounting
    print ('ho ricevuto: ', msg.data)
    if mounting is False and tools['main part'][1]==False:
        errorPub.publish('main part error')
    else:
        if msg.data != "no" and msg.data != "don't start":
            if mounting is False:
                handover(tools['support'][0],mountPos,0)
                '''destinationPos.positions=mountPos
                ur10Pub.publish(destinationPos)
                flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
                if flag.data is not True:
                    raise Exception("Sorry, An exception occurred") 
                time.sleep(3)
                gripping(0)'''
            else:
                gripping(100)
                returningPosition()
            mounting = not mounting
        else:
            time.sleep(1)
            eventPub.publish('alexa ask collaborative tutorial where is the '+mountTool)
#collback per put down
def putDownCallBack(tool):
    print(tool.data)
    if tools(tool.data)[1]== True:
        destinationPos.positions=tools[tool.data][0]
        ur10Pub.publish(destinationPos)
#cambio del flag per stopscrew
def stopScrewCallBack(flag):
    flagOrientation=True

#callback per gestire i consilgi in base agli step
def stepCallBack(step):
    '''if step=='next':
        array=steps.keys()
        preStep=currentStep
        currentStep=steps.keysarray.index(preStep)+1
    else:
        currentStep=step'''
    if step.data == "finish" and mounting is True:
        destinationPos.positions=[1.4621772766113281, -1.518237904911377, 2.4079530874835413, -2.4440018139281214, -1.5012553373919886, -0.09433633485902959]
        ur10Pub.publish(destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        gripping(100)
        returningPosition()

    if step.data == "first":
        if tools['part two'][1]==False:
            print("primo step")
            handover(tools['part two'][0],side[selectedSide],0)
            tools['part two'][1]=True
            time.sleep(1)
            responsePub.publish('1')
    elif step.data == " second":
        pass
    elif step.data == " third":
        pass
    
    if step.data == "fourth":
        time.sleep(7)
        eventPub.publish("orientation advise")
        
    ''' else:
        x = randInt()
        if x > 60:
            time.sleep(5)
            global needTool
            needTool=steps[step.data][1]
            speechText= " ".join(needTool)
            tts="to mount you need the " + speechText
            time.sleep(2)
            ttsPub.publish(tts)
            #rospy.wait_for_message('/wait', Bool)
            time.sleep(1)
            eventPub.publish("random advise")'''

#random consiglio
def randomAdviseCallBack(confirm):
    if "no" not in confirm.data:
        if isinstance(needTool, str):
            handover(tools[needTool][0],side[selectedSide],0)
            responsePub.publish('1')
            tools[needTool][1]=True
            global lastTool
            lastTool=needTool
        else:
            for i in needTool:
                if i in tools.keys():
                    tools[i][1]=True
                elif i in tips.keys():
                    tips[i][1]=True    
            multihandover(needTool)    
            
       
            tts="I've finished bringing you the tools, they're on the table"
            ttsPub.publish(tts)


def locateCallBack(tool):
    tool=tool.data
    global needTool
    if "." in tool:
        needTool = tool.split('.')
    else:
        needTool=tool    
    if mounting is False:
        x=randInt()
        if x > 0:
            time.sleep(3)
            eventPub.publish("random advise")



def main():
    current_directory = os.getcwd()
    print(current_directory)
    
    #subprocess.Popen("python launch.py", shell=True, cwd="../../filippo/ros_ws/src/collaborative_robot/scripts")
   
    
    tipSub = rospy.Subscriber('/tip',String,tipCallBack)
    toolSub = rospy.Subscriber('/tool',String,toolCallBack)
    sideSub = rospy.Subscriber('/side',String,sideCallBack)
    orientationSub = rospy.Subscriber('/orientation',String,orientationCallBack)
    responseSub= rospy.Subscriber('/response',String,responseCallBack)
    confirmSub= rospy.Subscriber('/confirm',String,confirmCallBack)
    bottleSub= rospy.Subscriber('/bottle',String,bottleCallBack)
    mountigSub = rospy.Subscriber('/mounting',String,mountingCallBack)
    stopScrewSub=rospy.Subscriber('/stopscrew',Bool,stopScrewCallBack)
    stepSub=rospy.Subscriber('/step',String,stepCallBack)
    randomAdviseSub=rospy.Subscriber('/randomAdvise',String,randomAdviseCallBack)
    locatingSub=rospy.Subscriber('/locate',String,locateCallBack)

    time.sleep(5)
    print('--nodo inizializzato---asttesa messaggi--')

    returningPosition()    
    gripper_req.position, gripper_req.speed, gripper_req.force = 100, 100, 25
    rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
    gripper_response = gripper_srv(gripper_req)
    gripping(100)
    print(gripper_response.status)
    print(gripper_response.success)
    time.sleep(1)
    #launchPub.publish(True)
    rospy.spin()#aspetta all'infinito


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass