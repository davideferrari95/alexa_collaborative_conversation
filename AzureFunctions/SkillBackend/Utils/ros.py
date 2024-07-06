#!/usr/bin/env python3
import rospy, time
from typing import Union

# Import Messages
from std_msgs.msg import String, Bool
from alexa_conversation.msg import MultimessageCommand, PickCommand, MoveCommand, MoveObjectCommand, ExecuteTaskCommand
from Utils.command_list import *

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
        self.command_pub       = rospy.Publisher('/alexa/command', MultimessageCommand, queue_size=1)
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

    def send_command(self, command:Union[Command, PickCommand, MoveCommand, MoveObjectCommand]):

        """ Send Command to ROS """

        # Create Multimessage Command
        msg = MultimessageCommand()

        # Command ID and Info
        msg.name = command.getName()
        msg.type = command.getType()
        msg.id   = command.getID()
        msg.info = command.getInfo()

        # Null, ROS, Default, Stop Commands
        if command.getType() in [NULL, ROS, DEFAULT, STOP]: pass

        # Move Command
        elif command.getType() == MOVE:

            # Add Movement Information
            msg.move_command.direction = command.getDirection()
            msg.move_command.distance  = command.getDistance()
            msg.move_command.measure   = command.getMeasure()
            msg.move_command.location  = command.getLocation()

        # Pick Command
        elif command.getType() == PICK:

            # Add Pick Information
            msg.pick_command.object_name = command.getObjectName()
            msg.pick_command.location    = command.getLocation()

        # Move Object Command
        elif command.getType() == MOVE_OBJECT:

            # Add Object Information
            msg.move_object_command.object_name   = command.getObjectName()
            msg.move_object_command.from_location = command.getFromLocation()
            msg.move_object_command.to_location   = command.getToLocation()

        # Execute Task Command
        elif command.getType() == EXECUTE_TASK:

            # Add Task Information
            msg.execute_task_command.task_name = command.getTask()

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
