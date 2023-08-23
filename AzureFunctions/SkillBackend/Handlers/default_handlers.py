import logging

# Import Ask SDK
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.dispatch_components import AbstractRequestInterceptor, AbstractResponseInterceptor
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.response import Response
from ask_sdk_core.utils import is_request_type, is_intent_name, get_intent_name

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
        logging.info("In FallbackIntentHandler")
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
