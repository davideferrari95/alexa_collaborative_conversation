#!/usr/bin/env python3
import rospy, time

# Import Messages
from std_msgs.msg import String, Bool
from alexa_conversation.msg import VoiceCommand
from Utils.command_list import AbstractCommand

class SkillServerNode():

    """ Skill Server ROS Node """

    # Initialize Keep Alive Flag
    KEEP_ALIVE = False
    alive_sended = False
    another_dialog = False

    def __init__(self):

        # Open ROS Skill Server
        rospy.init_node('skill_server', disable_signals=True)
        time.sleep(1)

        # ROS Publishers
        self.command_pub       = rospy.Publisher('/multimodal_fusion/voice_command', VoiceCommand, queue_size=1)
        self.alexa_tts_pub     = rospy.Publisher('/alexa/tts', String, queue_size=1)
        self.alexa_events_pub  = rospy.Publisher('/alexa/events', String, queue_size=1)
        self.alexa_routine_pub = rospy.Publisher('/alexa/routine_command', String, queue_size=1)
        self.alexa_alive_pub   = rospy.Publisher('/alexa/alive', Bool, queue_size=1)

        # ROS Subscribers
        self.keep_alive_sub    = rospy.Subscriber('/alexa/keep_alive', Bool, self.keep_alive_callback)

    def keep_alive_callback(self, msg:Bool):

        """ Keep Skill Alive Callback """

        # Set Keep Alive Flag
        self.KEEP_ALIVE = msg.data
        print('\nKeep Alive Callback:', msg.data, '\n')

    def send_command(self, command:AbstractCommand, wait_time=None):

        # Voice Command Message
        msg = VoiceCommand()
        msg.command = command.getID()
        msg.info = command.getInfo()

        # Area Command
        msg.area = command.getArea()

        self.command_pub.publish(msg)

    def alexa_tts(self, text:str):

        # Publish Alexa TTS
        self.alexa_tts_pub.publish(String(text))

    def alexa_event(self, event:str):

        # Publish Alexa Event
        self.alexa_events_pub.publish(String(event))

    def alexa_routine(self, routine:str):

        # Publish Alexa Routine
        self.alexa_routine_pub.publish(String(routine))

    def alexa_keep_alive(self):

        # Publish Alexa Routine
        self.alexa_alive_pub.publish(Bool(True))

# Start Skill Server ROS Node
SkillNode = SkillServerNode()
print('Skill Server Started')
