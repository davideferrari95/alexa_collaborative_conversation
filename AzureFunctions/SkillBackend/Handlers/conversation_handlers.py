import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import SkillNode
from Utils.command_list import command_list

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

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

class Test_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Test_API')

    def handle(self, handler_input: HandlerInput):

        print('Test_API Handler')

        # Publish ROS Message
        SkillNode.send_command(command_list.get_command_by_name('MOVED_OBJECT'))
        SkillNode.another_dialog = False

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
