#!/usr/bin/env python3
import os, json, logging
import rospy, rospkg
from std_msgs.msg import String, Int32

# Azure Functions
import azure.functions as func

# Import Ask SDK
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_webservice_support.webservice_handler import WebserviceSkillHandler
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.dispatch_components import AbstractRequestInterceptor, AbstractResponseInterceptor
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.response import Response
from ask_sdk_core.utils import request_util, is_request_type, is_intent_name, get_intent_name

# Open ROS Skill Server in a Separate Thread
rospy.init_node('skill_server', disable_signals=True)
intent_info_publisher = rospy.Publisher('intent_info', String, queue_size=1)
intent_publisher = rospy.Publisher('intent', Int32, queue_size=1)

def is_api_request(handler_input, api_name):

    """ Helper method to check if the incoming request is an API request """

    try:
        return request_util.get_request_type(handler_input) == 'Dialog.API.Invoked' and handler_input.request_envelope.request.api_request.name == api_name
    except Exception as ex:
        logging.error(ex)
        return False

def get_api_arguments(handler_input):

    """ Helper method to get API arguments from the request envelope """

    try:
        return handler_input.request_envelope.request.api_request.arguments
    except Exception as ex:
        logging.error(ex)
        return False

class LaunchRequestHandler(AbstractRequestHandler):

    """ Handler for Skill Launch """

    def can_handle(self, handler_input):

        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):

        logging.info('Launch Request Handler')
        speak_output = "Welcome, you can say Hello or Help. Which would you like to try?"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

class HelpIntentHandler(AbstractRequestHandler):

    """ Handler for Help Intent """

    def can_handle(self, handler_input):

        # type: (HandlerInput) -> bool
        return is_intent_name("AMAZON.HelpIntent")(handler_input)

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        speak_output = "You can say hello to me! How can I help?"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )

class CancelOrStopIntentHandler(AbstractRequestHandler):

    """ Single Handler for Cancel and Stop Intent """

    def can_handle(self, handler_input):

        # type: (HandlerInput) -> bool
        return (is_intent_name("AMAZON.CancelIntent")(handler_input) or
                is_intent_name("AMAZON.StopIntent")(handler_input))

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        speak_output = "Goodbye!"

        return (
            handler_input.response_builder
                .speak(speak_output)
                .response
        )

class FallbackIntentHandler(AbstractRequestHandler):

    """ Handler for Fallback Intent """

    def can_handle(self, handler_input):

        return is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        # logger.info("In FallbackIntentHandler")
        speech = "Hmm, I'm not sure. You can tell me give a tool or change orientation. What would you like to do?"
        reprompt = "I didn't catch that. What can I help you with?"

        return handler_input.response_builder.speak(speech).ask(reprompt).response

class SessionEndedRequestHandler(AbstractRequestHandler):

    """ Handler for Session End """

    def can_handle(self, handler_input):

        # type: (HandlerInput) -> bool
        return is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        # Any cleanup logic goes here.

        return handler_input.response_builder.response

class IntentReflectorHandler(AbstractRequestHandler):

    """ The intent reflector is used for interaction model testing and debugging.
    It will simply repeat the intent the user said. You can create custom handlers
    for your intents by defining them above, then also adding them to the request
    handler chain below.
    """

    def can_handle(self, handler_input):

        # type: (HandlerInput) -> bool
        return is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):

        # type: (HandlerInput) -> Response
        intent_name = get_intent_name(handler_input)
        speak_output = "You just triggered " + intent_name + "."

        return (
            handler_input.response_builder
            .speak(speak_output)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response
        )

class CatchAllExceptionHandler(AbstractExceptionHandler):

    """ Generic error handling to capture any syntax or routing errors. If you receive an error
    stating the request handler chain is not found, you have not implemented a handler for
    the intent being invoked or included it in the skill builder below.
    """

    def can_handle(self, handler_input, exception):

        return True

    def handle(self, handler_input, exception):

        logging.error(exception, exc_info=True)
        
        speak_output = "Sorry,Sorry, I had trouble doing what you asked. Please try again."

        return (
            handler_input.response_builder
            .speak(speak_output)
            .ask(speak_output)
            .response
        )

class LoggingRequestInterceptor(AbstractRequestInterceptor):

    """ Log the request envelope """

    def process(self, handler_input):

        # type: (HandlerInput) -> None
        logging.info("Request Envelope: {}".format(handler_input.request_envelope))

class LoggingResponseInterceptor(AbstractResponseInterceptor):

    """ Log the response envelope """

    def process(self, handler_input, response):

        # type: (HandlerInput, Response) -> None
        logging.info("Response: {}".format(response))


class testIntentHandler(AbstractRequestHandler):

    """ Handler for testIntent Intent """

    def can_handle(self, handler_input: HandlerInput):

        return is_intent_name("testIntent")(handler_input)

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

def main(req: func.HttpRequest) -> func.HttpResponse:

    """
    The SkillBuilder object acts as the entry point for your skill, routing all request and response
    payloads to the handlers above. Make sure any new handlers or interceptors you've
    defined are included below. The order matters - they're processed top to bottom.
    """

    # Skill Builder
    skill_builder = SkillBuilder()
    skill_builder.skill_id = os.environ["AlexaSkillID"]

    # Register Default Handlers
    skill_builder.add_request_handler(LaunchRequestHandler())
    # skill_builder.add_request_handler(HelpIntentHandler())
    # skill_builder.add_request_handler(CancelOrStopIntentHandler())
    # skill_builder.add_request_handler(FallbackIntentHandler())
    # skill_builder.add_request_handler(SessionEndedRequestHandler())
    # skill_builder.add_request_handler(IntentReflectorHandler())
    # skill_builder.add_exception_handler(CatchAllExceptionHandler())

    # Register Intents Handlers
    # skill_builder.add_request_handler(BeginExperiment_IntentHandler())
    skill_builder.add_request_handler(testIntentHandler())
    
    # Register Conversation Handlers
    skill_builder.add_request_handler(ObstacleDetected_ObjectMoved_API_Handler())
    # skill_builder.add_request_handler(ObstacleDetected_ObjectMoved_Handler())

    # Register Interceptors
    # skill_builder.add_global_request_interceptor(LoggingRequestInterceptor())
    # skill_builder.add_global_response_interceptor(LoggingResponseInterceptor())
    # lambda_handler = skill_builder.lambda_handler()

    # Create Skill and Deploy
    webservice_handler = WebserviceSkillHandler(skill=skill_builder.create())
    response = webservice_handler.verify_request_and_dispatch(req.headers, req.get_body().decode("utf-8"))

    return func.HttpResponse(json.dumps(response),mimetype="application/json")
