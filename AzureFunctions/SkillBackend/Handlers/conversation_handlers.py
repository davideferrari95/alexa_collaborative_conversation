import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments, custom_API_response
from Utils.ros   import SkillNode
from Utils.command_list import *

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class BeginExperiment_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'BeginExperiment_API')

    def handle(self, handler_input: HandlerInput):

        print('BeginExperiment_API Handler')

        # Publish ROS Message
        SkillNode.send_command(command_list.get_command_by_name('EXPERIMENT_START'))

        return {
            "apiResponse": {},
            "shouldEndSession": True
        }

class Alive_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Alive_API')

    def handle(self, handler_input: HandlerInput):

        print('Alive_API Handler')
        SkillNode.alive_sended = True

        return {
            "apiResponse": {},
            "reprompt": {},
            "shouldEndSession": False
        }

class AnotherDialog_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'AnotherDialog_API')

    def handle(self, handler_input: HandlerInput):

        print('AnotherDialog_API Handler')
        SkillNode.another_dialog = True

        return {
            "apiResponse": {},
            "shouldEndSession": False
        }

class MoveDirection_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'MoveDirection_API')

    def handle(self, handler_input: HandlerInput):

        print('MoveDirection_API Handler - Move API')

        # Get Arguments from API Request
        args = get_api_arguments(handler_input)
        measure, distance, direction = args['measure'], args['distance'], args['direction']
        print(f'Moving {direction} {distance} {measure}')

        # Get Command
        command:MoveCommand = command_list.get_command_by_name('MOVE_DIRECTION')

        # Update Command
        command.setDirection(direction)
        command.setDistance(distance)
        command.setMeasure(measure)

        # Return API Response
        custom_API_response(handler_input, command, 'Moving ' + direction + ' ' + distance + ' ' + measure)

class GoTo_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'GoTo_API')

    def handle(self, handler_input: HandlerInput):

        print('GoTo_API Handler - GoTo API')

        # Get Arguments from API Request
        args = get_api_arguments(handler_input)
        location = args['location']
        print(f'GoTo {location}')

        # Get Command
        command:MoveCommand = command_list.get_command_by_name('MOVE_GOTO')

        # Update Command
        command.setLocation(location)

        # Return API Response
        custom_API_response(handler_input, command, 'GoTo ' + location)

