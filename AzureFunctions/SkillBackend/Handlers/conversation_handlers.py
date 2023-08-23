import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.response import Response
from ask_sdk_core.utils import is_request_type, is_intent_name, get_intent_name

# Import Utilities
from Utils.utils import is_api_request, get_api_arguments
from Utils.ros   import intent_publisher, intent_info_publisher

# Import Messages
from std_msgs.msg import String, Int32

class ObstacleDetected_ObjectMoved_API_Handler(AbstractRequestHandler):

    def can_handle(self, handler_input: HandlerInput):

        logging.debug('ObstacleDetected_ObjectMoved_API Intent Handler')
        return is_api_request(handler_input, 'ObstacleDetected_ObjectMoved_API')

    def handle(self, handler_input: HandlerInput):

        # args=get_api_arguments(handler_input)

        logging.debug('ObstacleDetected_ObjectMoved_API Handler')

        msg = String()
        msg.data = 'ObstacleDetected_ObjectMoved'
        intent_info_publisher.publish(msg)
        
        msg = Int32()
        msg.data = 1
        intent_publisher.publish(msg)

        # type: (HandlerInput) -> Response
        speak_output = "Handler"
        

        # return (
        #     handler_input.response_builder
        #         .speak(speak_output)
        #         # .ask("add a reprompt if you want to keep the session open for the user to respond")
        #         .response
        # )

        # session_attributes = handler_input.attributes_manager.session_attributes
        # session_attributes['selectedOrientation'] = orientation
        response={
            "apiResponse": {},
            "shouldEndSession": True
        }
        return response
