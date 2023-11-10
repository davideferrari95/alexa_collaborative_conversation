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
        print('BeginExperiment_API Handler')

        # Publish ROS Message
        send_command(BEGIN_EXPERIMENT)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class BeginExperiment_TestDemo_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'BeginExperiment_TestDemo_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('BeginExperiment_TestDemo_API Handler')
        print('BeginExperiment_TestDemo_API Handler')

        # Publish ROS Message
        send_command(BEGIN_EXPERIMENT)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Provide_Screw_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Screw_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Provide_Screw_API Handler')
        print('Provide_Screw_API Handler')

        # Publish ROS Message
        send_command(PROVIDE_SCREW)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Provide_Screw_TestDemo_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Screw_TestDemo_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Provide_Screw_TestDemo_API Handler')
        print('Provide_Screw_TestDemo_API Handler')

        # Publish ROS Message
        send_command(PROVIDE_SCREW)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Provide_Screwdriver_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Screwdriver_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Provide_Screwdriver_API Handler')
        print('Provide_Screwdriver_API Handler')

        # Publish ROS Message
        send_command(PROVIDE_SCREWDRIVER)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Hold_Object_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Hold_Object_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Hold_Object_API Handler')
        print('Hold_Object_API Handler')

        # Publish ROS Message
        send_command(HOLD_OBJECT)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Take_Object_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Take_Object_API')

    def handle(self, handler_input: HandlerInput):

        logger.debug('Take_Object_API Handler')
        print('Take_Object_API Handler')

        # Publish ROS Message
        send_command(TAKE_OBJECT)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response

class Move_Mounting_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Move_Mounting_API')

    def handle(self, handler_input: HandlerInput):

        # Get API Arguments
        args = get_api_arguments(handler_input)

        # Check for `YesNo` Argument
        if 'YesNo' in args.keys(): yes_no = args['YesNo']
        else: yes_no = 'No'

        if yes_no.lower() in ['yes', 'sure']: yes_no = True
        elif yes_no.lower() in ['no', 'no thanks']: yes_no = False

        logger.debug('Move_Mounting_API Handler')
        print(f'Move_Mounting_API Handler: {yes_no}')

        # Publish ROS Message
        if yes_no: send_command(MOVE_MOUNTING)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response
