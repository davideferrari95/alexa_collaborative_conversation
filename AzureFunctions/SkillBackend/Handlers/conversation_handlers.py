import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import send_command, available_objects
from Utils.ros   import BEGIN_EXPERIMENT, PROVIDE_SCREW, PROVIDE_SCREWDRIVER, HOLD_OBJECT, TAKE_OBJECT, MOVE_MOUNTING

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class BeginExperiment_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'BeginExperiment_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('BeginExperiment_API Handler')

        # Publish ROS Message
        send_command(BEGIN_EXPERIMENT)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Provide_Screw_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Screw_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Provide_Screw_API Handler')
        print('Provide_Screw_API Handler')

        # Publish ROS Message
        send_command(PROVIDE_SCREW)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Provide_Screwdriver_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Screwdriver_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Provide_Screwdriver_API Handler')
        print('Provide_Screwdriver_API Handler')

        # Publish ROS Message
        send_command(PROVIDE_SCREWDRIVER)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Hold_Object_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Hold_Object_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Hold_Object_API Handler')
        print('Hold_Object_API Handler')

        # Publish ROS Message
        send_command(HOLD_OBJECT)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Take_Object_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Take_Object_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Take_Object_API Handler')
        print('Take_Object_API Handler')

        # Publish ROS Message
        send_command(TAKE_OBJECT)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Move_Mounting_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Move_Mounting_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Move_Mounting_API Handler')
        print('Move_Mounting_API Handler')

        # Publish ROS Message
        send_command(MOVE_MOUNTING)

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }
