#!/usr/bin/env python3
from ast import arg
import logging
import os
import json
from urllib import response
import requests
import ask_sdk_core.utils as ask_utils
import time
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from ask_sdk_core.dispatch_components import AbstractRequestInterceptor
from ask_sdk_core.dispatch_components import AbstractResponseInterceptor
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_webservice_support.webservice_handler import WebserviceSkillHandler
from ask_sdk_model.dialog import (
    ElicitSlotDirective, DelegateDirective)
from ask_sdk_model import (
    Response, IntentRequest, DialogState, SlotConfirmationStatus, Slot)

from ask_sdk_model import Response
from ask_sdk_model.dialog import DelegateRequestDirective, DelegationPeriodUntil
from ask_sdk_core.utils import is_request_type, is_intent_name #from dependencies.function_helper import util #(absolute)

import azure.functions as func
#from utils import util
from OpenSSL import rand, crypto, SSL
import rospy
from std_msgs.msg import String,Bool
import ssl

rospy.init_node('message')

toolPub = rospy.Publisher('/tool',String,queue_size=10)
pub_orientation=rospy.Publisher('/orientation',String,queue_size=10)
pub_side=rospy.Publisher('/side',String,queue_size=10)
pub_response=rospy.Publisher('/response',String,queue_size=10)
pub_confirm= rospy.Publisher('/confirm',String,queue_size=10)
pub_mounting=rospy.Publisher('/mounting',String,queue_size=10)
putDownPub=rospy.Publisher('/putdown',String,queue_size=10)
stopScrewPub=rospy.Publisher('/stopscrew',Bool,queue_size=10)
stepPub=rospy.Publisher('/step',String,queue_size=10)
randomAdvisePub=rospy.Publisher('/randomAdvise',String,queue_size=10)
tipPub=rospy.Publisher('/tip',String,queue_size=10)
locationPub=rospy.Publisher('/locate',String,queue_size=10)
time.sleep(1)

stepDescription={
    "first":["in the first step you will have to assemble the part two that I have already brought you  on the part one in the green area with te cross screw using the cross screwdriver","mount the part two that I'm bringing you on the part one in the green area with the cross screw",False],
    "second":["in the second step you will have to mount the part three over the support of part four using the allen screw and allenkey","mount the part three on the the support of part four part using the  allen screw",False],
    "third":["in the third step you will have to mount the main part on the support part using the wrench e long screw. after the mounting i will not be able to take tool o item for you, but i can still give you advice","mount the main part on the support with the long screw and start mounting.",False],
    "fourth":["the fourth step in mount the two pre-assemblies on the main part with the remained screws.I advise you to change the orientation of the piece to vertical to be more comfortable for mounting the flange.","mount the two pre-assemplies on the main part with the remained screws. I advise you to change the orientation of the piece to vertical",False]
}

toolLocation={
    'main part':["red",False],
    'part one':["green",False],
    'part two':["red",False],
    'part three':["red",False],
    'part four':["yellow",False],
    'long screw':["green",False],
    'allenkey ':["green",False],
    'bottle':["yellow",False],
    # 'flat screwdrivers':["green",False],
    'flat screwdriver':["green",False],
    #'flat screwdriver':[[],False,True],
    # 'multifunction screwdrivers':["yellow",False],
    'multifunction screwdriver':["yellow",False],
    'wrench':["yellow",False],
    # 'allen screws':["yellow",False],
    'allen screw':["yellow",False],
    # 'cross screws':["green",False],
    'cross screw':["green",False],
    'cross tip':["red",False],
    'flat tip':["red",False],

    'torox tip':["red",False]

}

lastToolSelected=""

class util:
    def is_api_request(handler_input, api_name):
        try:
            return ask_utils.request_util.get_request_type(handler_input) == 'Dialog.API.Invoked' and handler_input.request_envelope.request.api_request.name == api_name
        except Exception as ex:
            logging.error(ex)
            return False


    def get_api_arguments(handler_input):
        """Helper method to get API arguments from the request envelope."""
        try:
            return handler_input.request_envelope.request.api_request.arguments
        except Exception as ex:
            logging.error(ex)
            return False

class SetOrientationApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input, 'SetOrientation')

    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        orientation = args['orientation']
        print(args)
        print(orientation)
        pub_orientation.publish(orientation)

        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['selectedOrientation'] = orientation
        response={
            "apiResponse": {
                "orientation": orientation
            },
            "shouldEndSession": True
        }
        return response

class GetOrientaionApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input, 'GetOrientation')
    def handle(self, handler_input: HandlerInput):
        session_attributes = handler_input.attributes_manager.session_attributes
        
        if 'selectedOrientation' in session_attributes.keys():
            orientation = session_attributes['selectedOrientation']
        else: orientation = None
        
        response = {
            "apiResponse": {
                "selectedOrientation" : orientation
            },
            "shouldEndSession": False
        }

        return response

class SetSideApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input, 'SetSide')

    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        side = args['side']
        print(args)
        print(side)
        pub_side.publish(side)

        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['selectedSide'] = side
        response={
            "apiResponse": {
                "side": side
            },
            "shouldEndSession": True
        }
        return response

class GetSideApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input, 'GetSide')
    def handle(self, handler_input: HandlerInput):
        session_attributes = handler_input.attributes_manager.session_attributes
        
        if 'selectedSide' in session_attributes.keys():
            orientation = session_attributes['selectedSide']
        else: side = None
        
        response = {
            "apiResponse": {
                "selectedSide" : side
            },
            "shouldEndSession": False
        }

        return response

class TakeToolHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'TakeTool')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        tool=args['tool']
        if "screwdrivers" in tool: tool = tool[:-1]
        if "screws" in tool: tool = tool[:-1]

        toolPub.publish(tool)

        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['selectedTool'] = tool
        global lastToolSelected
        lastToolSelected=tool
        print (lastToolSelected)
        response={
            "apiResponse": {
                "tool": tool
            },
            "shouldEndSession": True
        }
        return response   

class GetTipsHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'GetTipsApiDefinition')

    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        tip=args['tip']
        print(tip)
        tipPub.publish(tip)

        
        response={
            "apiResponse": {
                "tip": tip
            },
            "shouldEndSession": True
        }
        return response   

class FindToolHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'FindTool')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        taken=args['response']
        print(taken)
        print(args)
        pub_response.publish(taken)
        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['res'] = taken
        response={
            "apiResponse": {
                "response": taken
            },
            "shouldEndSession": False
        }
        return response           

class NotTakenHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'RequestConfirmNotTaken')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        print(args)
        pub_response.publish(confirm)
        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['conf'] = confirm

        response={
                "apiResponse": {
                    "confirm": confirm
                },
                "shouldEndSession": True
            }
        return response      

class TakeScrewsHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'TakeScrewsAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        print(args)
        pub_confirm.publish(confirm)
        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['conf'] = confirm

        response={
                "apiResponse": {
                    "confirm": confirm
                },
                "shouldEndSession": True
            }
        return response                

class StopMountingApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'StopMountingAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        res=args['userRespondeMount']
        print(res)
        print(args)
        pub_mounting.publish(res)
        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['resMount'] = res

        response={
                "apiResponse": {
                    "userRespondeMount": res
                },
                "shouldEndSession": False
            }
        return response                

class MoreSpecificApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return util.is_api_request(handler_input,'MoreSpecificAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        #lastToolSelected="screwdriver"
        global lastToolSelected
        print (lastToolSelected)

        desc="there is"
        for i in toolLocation.keys():
            if lastToolSelected in i:
                desc=desc + " " + i+","
        desc=desc + ". what do you want?"
        print(desc)
        
        response={
            "apiResponse": {
                "description": desc
            },
            "shouldEndSession": True
        }
        return response   
        
class MissingErrorApiHanlder(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return util.is_api_request(handler_input,'MissingErrorAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        tool=args['tool']
        print(tool)
        session_attributes = handler_input.attributes_manager.session_attributes
        session_attributes['selectedTool'] = tool
        response={
            "apiResponse": {
                "tool": tool
            },
            "shouldEndSession": True
        }
        return response   

class AlreadyTakenApiHanlder(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return util.is_api_request(handler_input,'AlreadyTakenAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        tool=args['tool']
        print(tool)
        toolPub.publish(tool)
        
        response={
            "apiResponse": {
                "tool": tool
            },
            "shouldEndSession": True
        }
        return response   

class StartMountigApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return util.is_api_request(handler_input,'StartMountigAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        res=args['userRespondeMount']
        print(res)
        print(args)
        pub_mounting.publish(res)
        

        response={
                "apiResponse": {
                    "userRespondeMount": res
                },
                "shouldEndSession": True
            }
        return response                        

class ErrorMountigApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return util.is_api_request(handler_input,'ErrorMountingAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        res=args['userRespondeMount']
        print(res)
        print(args)
        pub_mounting.publish(res)
        response={
                "apiResponse": {
                    "userRespondeMount": res
                },
                "shouldEndSession": False
                 
            }
        return response            

class FallbackIntentHandler(AbstractRequestHandler):
    """Handler for Fallback Intent.
    AMAZON.FallbackIntent is only available in en-US locale.
    This handler will not be triggered except in that locale,
    so it is safe to deploy on any locale.
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        logger.info("In FallbackIntentHandler")
        speech = "Hmm, I'm not sure. You can tell me give a tool or change orientation. What would you like to do?"
        reprompt = "I didn't catch that. What can I help you with?"

        return handler_input.response_builder.speak(speech).ask(reprompt).response

class SessionEndedRequestHandler(AbstractRequestHandler):
    """Handler for Session End"""

    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response

        # Any cleanup logic goes here.

        return handler_input.response_builder.response

class IntentReflectorHandler(AbstractRequestHandler):
    """The intent reflector is used for interaction model testing and debugging.
    It will simply repeat the intent the user said. You can create custom handlers
    for your intents by defining them above, then also adding them to the request
    handler chain below.
    """
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        intent_name = ask_utils.get_intent_name(handler_input)
        speak_output = "You just triggered " + intent_name + "."

        return (
            handler_input.response_builder
            .speak(speak_output)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response
        )

class CatchAllExceptionHandler(AbstractExceptionHandler):
    """Generic error handling to capture any syntax or routing errors. If you receive an error
    stating the request handler chain is not found, you have not implemented a handler for
    the intent being invoked or included it in the skill builder below.
    """

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        logger.error(exception, exc_info=True)

        speak_output = "Sorry,Sorry, I had trouble doing what you asked. Please try again."

        return (
            handler_input.response_builder
            .speak(speak_output)
            .ask(speak_output)
            .response
        )

class GetInfoStepApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return util.is_api_request(handler_input, 'GetInfoStepAPI')

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        args = util.get_api_arguments(handler_input)
        step = args['step']
        if step == "fifth":
            desc="congratulation you have already finish"
            stepPub.publish("finish") 
        elif step == "next":
            desc="congratulation you have already finish"
            for i in stepDescription.keys():
                if stepDescription[i][2] is False:
                    desc = stepDescription[i][1]
                    
                    stepDescription[i][2] = True
                    stepPub.publish(i)
                    break
            if desc=="congratulation you have already finish":
                stepPub.publish("finish")    
        else:
            if step=="first" and stepDescription["first"][2]==False:
                desc=stepDescription["first"][1]
                stepDescription["first"][2]=True 
                stepPub.publish(step)
             
            else:
                desc = stepDescription[step][0]
                stepPub.publish(step) 
        print(desc)
        response = {
            'apiResponse': {
                'step': step,
                'description': desc
                
            },
            "shouldEndSession": False
        }
        return response
    
class GetLocationToolApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return util.is_api_request(handler_input, 'LocateToolAPIDefinition')

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        args = util.get_api_arguments(handler_input)

        tool = args['tool']
        print(tool)
        needTool=""
        global lastToolSelected

        if "screwdrivers" in tool: tool = tool[:-1]
        if "screws" in tool: tool = tool[:-1]

        if tool not in toolLocation.keys() and "screwdriver" in tool:
            if 'flat' in tool or 'cross' in tool or 'torox' in tool:
                pit = tool.split()[0]
                location = "there isn't the "+ tool +", but there is the "+ pit +" pit for the multyfunction screwdriver"
                lastToolSelected='multifunction screwdriver'
                needTool='multifunction screwdriver.'+pit
                locationPub.publish(needTool)
            else: location = "there isn't the "+ tool 
        elif tool not in toolLocation.keys() and "screwdriver" not in tool:
            location = "there isn't the "+ tool 
        else:
            if toolLocation[tool][1]==True:
                location="the " + tool + " ,is already taken"
            
            else: 
                location = "the " + tool + " is in the "+ toolLocation[tool][0] + " area"
                
                lastToolSelected=tool
                needTool=tool
                locationPub.publish(needTool)

                print (lastToolSelected)
        print(location)
        response = {
            'apiResponse': {
                'tool':tool,
                'location':location 
                
            },
            "shouldEndSession": False
        }
        return response    

class LoggingRequestInterceptor(AbstractRequestInterceptor):
    """Log the request envelope."""
    def process(self, handler_input):
        # type: (HandlerInput) -> None
        logger.info("Request Envelope: {}".format(
            handler_input.request_envelope))

class LoggingResponseInterceptor(AbstractResponseInterceptor):
    """Log the response envelope."""
    def process(self, handler_input, response):
        # type: (HandlerInput, Response) -> None
        logger.info("Response: {}".format(response))

class putDownApiHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return util.is_api_request(handler_input, 'PutDownAPIDefinition')

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        args = util.get_api_arguments(handler_input)

        tool = args['tool']
        print(tool)
        putDownPub.publish(tool)
        
        response = {
            'apiResponse': {
                'tool':tool
            },
            "shouldEndSession": True
        }
        return response     

class AdviceApiAndler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return util.is_api_request(handler_input, 'AdviceAPIDefinition')

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        args = util.get_api_arguments(handler_input)

        tool = args['tool']
        print(tool)
        
        response = {
            'apiResponse': {
                'tool':tool
            },
            "shouldEndSession": False
        }
        return response     
    
class RandomAdviseHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'RandomAdviseAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        print(args)
        randomAdvisePub.publish(confirm)
        
        response={
                "apiResponse": {
                    "confirm": confirm
                },
                "shouldEndSession": False
            }
        return response      

class OrientationAdviseHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'OrientationAdviseAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        if confirm != "no":
            pub_orientation.publish('vertical')
        response={
                "apiResponse": {
                    "confirm": confirm
                },
                "shouldEndSession": False
            }
        return response      

class MainPartErrorHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'MainPartErrorAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        if confirm != "no":
            toolPub.publish('main part')
        response={
                "apiResponse": {
                    "confirm": confirm
                },
                "shouldEndSession": True
            }
        return response      


class StopScrewErrorHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput):
        return util.is_api_request(handler_input,'StopScrewErrorAPIDefinition')
    def handle(self, handler_input: HandlerInput):
        args=util.get_api_arguments(handler_input)
        confirm=args['confirm']
        print(confirm)
        if confirm == "no":
            desc="before move the robot you have to mount the stop screw"
        else:
            desc="ok"
            stopScrewPub.publish(True)

        print(desc)    
        response={
                "apiResponse": {
                    "conf": confirm,
                    "description":desc
                },
                "shouldEndSession": False
            }
        return response      

def main(req: func.HttpRequest) -> func.HttpResponse:
    # The SkillBuilder object acts as the entry point for your skill, routing all request and response
    # payloads to the handlers above. Make sure any new handlers or interceptors you've
    # defined are included below. The order matters - they're processed top to bottom.
    sb = SkillBuilder()
    sb.skill_id = os.environ["AlexaSkillID"]
   
    sb.add_request_handler(SetOrientationApiHandler())
    sb.add_request_handler(GetOrientaionApiHandler())
    
    sb.add_request_handler(SetSideApiHandler())
    sb.add_request_handler(GetSideApiHandler())
    sb.add_request_handler(TakeToolHandler())
    sb.add_request_handler(FindToolHandler())
    sb.add_request_handler(NotTakenHandler())
    sb.add_request_handler(GetTipsHandler())
    sb.add_request_handler(TakeScrewsHandler())
    sb.add_request_handler(SessionEndedRequestHandler())
    sb.add_request_handler(IntentReflectorHandler())
    sb.add_request_handler(MoreSpecificApiHandler())
    sb.add_request_handler(MissingErrorApiHanlder())
    sb.add_request_handler(AlreadyTakenApiHanlder())
    sb.add_request_handler(StartMountigApiHandler())
    sb.add_request_handler(StopMountingApiHandler())
    sb.add_request_handler(ErrorMountigApiHandler())
    sb.add_request_handler(GetInfoStepApiHandler())
    sb.add_request_handler(GetLocationToolApiHandler())
    sb.add_request_handler(putDownApiHandler())
    sb.add_request_handler(AdviceApiAndler())
    sb.add_request_handler(RandomAdviseHandler())
    sb.add_request_handler(StopScrewErrorHandler())
    sb.add_request_handler(OrientationAdviseHandler())
    sb.add_request_handler(MainPartErrorHandler())


    #sb.add_request_handler(StopMountingIntentHandler())

    # register exception handlers
    sb.add_exception_handler(CatchAllExceptionHandler())

    # register interceptors
    sb.add_global_request_interceptor(LoggingRequestInterceptor())
    sb.add_global_response_interceptor(LoggingResponseInterceptor())
    webservice_handler = WebserviceSkillHandler(skill=sb.create())

    lambda_handler = sb.lambda_handler()


    response = webservice_handler.verify_request_and_dispatch(req.headers, req.get_body().decode("utf-8"))

    return func.HttpResponse(json.dumps(response),mimetype="application/json")  