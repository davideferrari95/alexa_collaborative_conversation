import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.response import Response
from ask_sdk_core.utils import is_intent_name

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments

class TestIntentHandler(AbstractRequestHandler):

    """ Handler for testIntent Intent """

    def can_handle(self, handler_input: HandlerInput):

        return is_intent_name("TestIntent")(handler_input)

    def handle(self, handler_input: HandlerInput):

        logging.info('testIntent Intent Handler')

        # Speak Output Response
        speak_output = "This is a Test!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )

class HelloWorldIntentHandler(AbstractRequestHandler):

    """ Handler for Hello World Intent """

    def can_handle(self, handler_input):

        # type: (HandlerInput) -> bool
        return is_intent_name("HelloWorldIntent")(handler_input)

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        speak_output = "Hello World!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                # .ask("add a reprompt if you want to keep the session open for the user to respond")
                .response
        )

class BeginExperiment_IntentHandler(AbstractRequestHandler):

    """ Handler for BeginExperiment Intent """

    def can_handle(self, handler_input: HandlerInput):

        return is_intent_name('BeginExperiment')(handler_input)

    def handle(self, handler_input: HandlerInput):

        # Get API Arguments
        session_attributes = handler_input.attributes_manager.session_attributes
        
        # if 'my_attribute' in session_attributes.keys():
        #     my_attribute = session_attributes['my_attribute']
        # else: my_attribute = None
        
        logging.debug('BeginExperiment Intent Handler')
        
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
