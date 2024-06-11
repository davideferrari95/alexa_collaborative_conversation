#!/usr/bin/env python3
import os, sys, json, time
import rospy, rospkg
from threading import Thread

# Azure Functions
import azure.functions as func

# Ask SDK
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_webservice_support.webservice_handler import WebserviceSkillHandler

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions/SkillBackend')

# Import Intent and Conversation Handlers
from Handlers.intent_handlers import *
from Handlers.default_handlers import *
from Handlers.conversation_handlers import *

# Import ROS Utilities
from Utils.ros import SkillNode

# Keep Alive Function
def keep_alive():

    """ Keep Skill Alive Function """

    while not rospy.is_shutdown():

        # Continue Loop if Keep Alive is False or Another Dialog is Running
        if not SkillNode.KEEP_ALIVE or SkillNode.another_dialog: continue

        # Publish Alive Command
        SkillNode.alexa_keep_alive()
        print('Keep Alive Call')

        # Wait 12 Seconds
        rospy.sleep(12)

# Keep Alive Thread
keep_alive_thread = Thread(target=keep_alive)
keep_alive_thread.start()

def main(req: func.HttpRequest) -> func.HttpResponse:

    """
    The SkillBuilder object acts as the entry point for your skill, routing all request and response
    payloads to the handlers above. Make sure any new handlers or interceptors you've
    defined are included below. The order matters - they're processed top to bottom.
    """

    # Skill Builder
    skill_builder = SkillBuilder()
    skill_builder.skill_id = os.environ["AlexaSkillID"]

    # Register Default Handlers - Intents
    skill_builder.add_request_handler(LaunchRequestHandler())
    skill_builder.add_request_handler(HelpIntentHandler())
    skill_builder.add_request_handler(CancelOrStopIntentHandler())
    skill_builder.add_request_handler(FallbackIntentHandler())
    skill_builder.add_request_handler(SessionEndedRequestHandler())
    skill_builder.add_request_handler(IntentReflectorHandler())
    skill_builder.add_exception_handler(CatchAllExceptionHandler())

    # Register Intents Handlers
    # skill_builder.add_request_handler(BeginExperiment_IntentHandler())
    # skill_builder.add_request_handler(TestIntentHandler())
    
    # Register Conversation Handlers
    # skill_builder.add_request_handler(BeginExperiment_API_Handler())
    # skill_builder.add_request_handler(AnotherDialog_API_Handler())
    # skill_builder.add_request_handler(Alive_API_Handler())
    skill_builder.add_request_handler(MovementDialogue_Direction_API_Handler())

    # Register Interceptors
    skill_builder.add_global_request_interceptor(LoggingRequestInterceptor())
    skill_builder.add_global_response_interceptor(LoggingResponseInterceptor())

    # Create Skill and Deploy
    webservice_handler = WebserviceSkillHandler(skill=skill_builder.create())
    response = webservice_handler.verify_request_and_dispatch(req.headers, req.get_body().decode("utf-8"))

    return func.HttpResponse(json.dumps(response),mimetype="application/json")
