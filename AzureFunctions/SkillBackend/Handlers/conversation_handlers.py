import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import send_command
from Utils.command_list import *

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class BeginExperiment_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'BeginExperiment_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('BeginExperiment_API Handler')

        # Publish ROS Message
        send_command(EXPERIMENT_START)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class ObstacleDetected_ObjectMoved_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'ObstacleDetected_ObjectMoved_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('ObstacleDetected_ObjectMoved_API Handler')

        # Publish ROS Message
        send_command(MOVED_OBJECT)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class PutObjectHere_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'PutObjectHere_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('PutObjectHere_API Handler')

        # Get API Arguments
        args = get_api_arguments(handler_input)

        # Check for `Area` Argument
        if 'area' in args.keys(): area = args['area']
        else: area = None

        # Area Command if Defined
        if area in available_areas: send_command(PUT_OBJECT_IN_GIVEN_AREA, area)
        elif area in gesture_areas: send_command(PUT_OBJECT_IN_AREA_GESTURE)
        else: send_command(PUT_OBJECT_IN_AREA)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class ResumeMoving_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'ResumeMoving_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('ResumeMoving_API Handler')

        # Publish ROS Message
        send_command(CAN_GO)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class WaitForCommand_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'WaitForCommand_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('WaitForCommand_API Handler')

        # Publish ROS Message
        send_command(WAIT_FOR_COMMAND)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class MoveToUser_UserMovedBack_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'MoveToUser_UserMovedBack_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('MoveToUser_UserMovedBack_API Handler')

        # Publish ROS Message
        send_command(USER_MOVED)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class WaitForTime_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'WaitForTime_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('WaitForTime_API Handler')

        # Get API Arguments
        args = get_api_arguments(handler_input)

        # Check for `time` and `time_unit` Arguments
        time = 0 if not 'time' in args.keys() else args['time']
        time_unit = 'seconds' if not 'time_unit' in args.keys() else args['time_unit']

        # Publish ROS Message
        send_command(WAIT_TIME, f'{time} {time_unit}')

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Wait_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Wait_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Wait_API Handler')

        # Publish ROS Message
        send_command(WAIT_FOR_COMMAND)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }
