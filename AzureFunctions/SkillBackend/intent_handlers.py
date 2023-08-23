from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput

from utils import is_api_request, get_api_arguments

class BeginExperiment_IntentHandler(AbstractRequestHandler):

    """ Handler for BeginExperiment Intent """

    def can_handle(self, handler_input: HandlerInput):
        return is_api_request(handler_input, 'BeginExperiment')

    def handle(self, handler_input: HandlerInput):

        # Get API Arguments
        session_attributes = handler_input.attributes_manager.session_attributes
        
        # if 'my_attribute' in session_attributes.keys():
        #     my_attribute = session_attributes['my_attribute']
        # else: my_attribute = None
        
        response = {
            "apiResponse": {
                "response" : "test"
            },
            "shouldEndSession": False
        }

        return response


class Example_IntentHandler(AbstractRequestHandler):

    """ Handler for Example Intent """

    def can_handle(self, handler_input: HandlerInput):
        return is_api_request(handler_input, 'Example')

    def handle(self, handler_input: HandlerInput):

        # Get API Arguments
        args = get_api_arguments(handler_input)
        my_arg = args['my_argument']

        # Get Session Attributes
        session_attributes = handler_input.attributes_manager.session_attributes

        # Search for a Specific Attribute in the Session Attributes
        if 'my_attribute' in session_attributes.keys(): my_attribute = session_attributes['my_attribute']
        else: my_attribute = None

        # Build Response
        response = {
            "apiResponse": 
            {
                "my_variable" : my_attribute
            },
            "shouldEndSession": False
        }

        return response
