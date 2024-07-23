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
        command.setDistance(float(distance))
        command.setMeasure(measure)

        # Return API Response
        success, response = custom_API_response(handler_input, command, f'Moving {direction} {distance} {measure}')
        print(success, response)

        # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response

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
        success, response = custom_API_response(handler_input, command, f"I'm going to {location}")

       # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response

class Stop_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Stop_API')

    def handle(self, handler_input: HandlerInput):

        print('Stop_API Handler - Stop API')

        # Get Command
        command:Command = command_list.get_command_by_name('STOP')

        # Return API Response
        success, response = custom_API_response(handler_input, command, 'Stopping...')

       # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response

class PickObject_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'PickObject_API')

    def handle(self, handler_input: HandlerInput):

        print('PickObject_API Handler - PickObject API')

        # Get Arguments from API Request
        args = get_api_arguments(handler_input)
        object, location = args['object'], args['location']
        print(f'Pick {object} from {location}')

        # Get Command
        command:PickCommand = command_list.get_command_by_name('PICK_OBJECT')

        # Update Command
        command.setObjectName(object)
        command.setLocation(location)

        # Return API Response
        success, response = custom_API_response(handler_input, command, f'I pick {object} from {location}')

       # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response

class MoveObject_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'MoveObject_API')

    def handle(self, handler_input: HandlerInput):

        print('MoveObject_API Handler - MoveObject API')

        # Get Arguments from API Request
        args = get_api_arguments(handler_input)
        object, from_location, to_location = args['object'], args['from_location'], args['to_location']
        print(f'Move {object} from {from_location} to {to_location}')

        # Get Command
        command:MoveObjectCommand = command_list.get_command_by_name('MOVE_OBJECT')

        # Update Command
        command.setObjectName(object)
        command.setFromLocation(from_location)
        command.setToLocation(to_location)

        # Return API Response
        success, response = custom_API_response(handler_input, command, f'I move {object} from {from_location} to {to_location}')

       # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response

class ExecuteTask_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'ExecuteTask_API')

    def handle(self, handler_input: HandlerInput):

        print('ExecuteTask_API Handler - ExecuteTask API')

        # Get Arguments from API Request
        args = get_api_arguments(handler_input)
        task_name = args['task_name']
        print(f'Execute Task {task_name}')

        # Get Command
        command:ExecuteTaskCommand = command_list.get_command_by_name('MOVE_OBJECT')

        # Update Command
        command.setTaskName(task_name)

        # Return API Response
        success, response = custom_API_response(handler_input, command, f'Execute Task {task_name}')

       # Publish ROS Message
        if success: SkillNode.send_command(command)

        # Return API Response
        return response
