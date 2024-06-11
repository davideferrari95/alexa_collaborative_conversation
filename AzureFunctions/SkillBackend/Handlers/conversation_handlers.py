import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import SkillNode
from Utils.command_list import command_list

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# API Response Status
SUCCESS, FAIL, DEFAULT = 'Success', 'Fail', 'Default'

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

class MovementDialogue_Direction_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'MovementDialogue_Direction_API')

    def handle(self, handler_input: HandlerInput):

        print('MovementDialogue_Direction_API Handler')

        # Publish ROS Message
        # SkillNode.send_command(command_list.get_command_by_name('MOVED_OBJECT'))
        # SkillNode.another_dialog = False

        args = get_api_arguments(handler_input)
        measure, distance, direction = args['measure'], args['distance'], args['direction']
        print(f'Moving {direction} {distance} {measure}')

        test = True
        # test = False

        if (test):

            # Return Success API Response
            return handler_input.response_builder.set_api_response({
                    'status': SUCCESS,
                    # 'status': DEFAULT,
                    'string': 'Test Success Message'
                }).set_should_end_session(False).response

        else:

            # Return Failed API Response
            return handler_input.response_builder.set_api_response({
                    'status': FAIL,
                    'string': 'Test Failed Message'
                }).set_should_end_session(True).response
