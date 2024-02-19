import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import send_command, GET_OBJECT

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class Provide_Object_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        return is_api_request(handler_input, 'Provide_Object_API')

    def handle(self, handler_input: HandlerInput):

        # Get API Arguments
        args = get_api_arguments(handler_input)
        print(f'Provide_Object_API Arguments: {args}')

        # Check for `YesNo` Argument
        if 'object' in args.keys(): object = args['object']
        else: object = 'None'

        logger.debug('Provide_Object_API Handler')
        print(f'Provide_Object_API Handler: {object}')

        # Publish ROS Message
        send_command(GET_OBJECT, object)

        response = {
            "apiResponse": {
                'empty': 'empty'
            }
        }

        return response
