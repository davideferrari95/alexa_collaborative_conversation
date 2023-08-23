#!/usr/bin/env python3
import os, sys, json, logging, threading
import rospy, rospkg

# Azure Functions
import azure.functions as func

# Ask SDK
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_webservice_support.webservice_handler import WebserviceSkillHandler

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_conversation")}/AzureFunctions/SkillBackend')

# Import Utilities
from utils import is_api_request, get_api_arguments, setup_logging

# Import Intent and Conversation Handlers
from intent_handlers import *
from conversation_handlers import *

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_server', disable_signals=True)).start()

# Setup Logging
logger = setup_logging()
logger.debug('Skill Server Started')

def main(req: func.HttpRequest) -> func.HttpResponse:

    """
    The SkillBuilder object acts as the entry point for your skill, routing all request and response
    payloads to the handlers above. Make sure any new handlers or interceptors you've
    defined are included below. The order matters - they're processed top to bottom.
    """

    # Skill Builder
    sb = SkillBuilder()
    sb.skill_id = os.environ["AlexaSkillID"]

    # Register Intents Handlers
    sb.add_request_handler(BeginExperiment_IntentHandler())
    
    # Register Conversation Handlers
    # sb.add_request_handler(ObstacleDetected_ObjectMoved_Handler())


    # sb.add_request_handler(SessionEndedRequestHandler())
    # sb.add_request_handler(IntentReflectorHandler())
    # sb.add_request_handler(AdviceApiAndler())
    # sb.add_request_handler(RandomAdviseHandler())

    # register exception handlers
    # sb.add_exception_handler(CatchAllExceptionHandler())

    # # register interceptors
    # sb.add_global_request_interceptor(LoggingRequestInterceptor())
    # sb.add_global_response_interceptor(LoggingResponseInterceptor())

    # Create Skill and Deploy
    webservice_handler = WebserviceSkillHandler(skill=sb.create())
    lambda_handler = sb.lambda_handler()

    response = webservice_handler.verify_request_and_dispatch(req.headers, req.get_body().decode("utf-8"))

    return func.HttpResponse(json.dumps(response),mimetype="application/json")
