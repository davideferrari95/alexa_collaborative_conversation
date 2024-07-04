import logging, requests

# Import Ask SDK
import ask_sdk_core.utils as ask_utils
from ask_sdk_core.handler_input import HandlerInput

# Import Utilities
from Utils.command_list import *
from Utils.ros import SkillNode
from typing import Union, Tuple

# API Response Status
SUCCESS, FAIL, DEFAULT = 'Success', 'Fail', 'Default'

def is_api_request(handler_input, api_name):

    """ Helper method to check if the incoming request is an API request. """

    try: return ask_utils.request_util.get_request_type(handler_input) == 'Dialog.API.Invoked' and handler_input.request_envelope.request.api_request.name == api_name
    except Exception as ex:
        logging.error(ex)
        return False

def get_api_arguments(handler_input):

    """Helper method to get API arguments from the request envelope."""

    try: return handler_input.request_envelope.request.api_request.arguments
    except Exception as ex:
        logging.error('Error occurred: ', ex)
        return False

def get_slots(handler_input):

    """ Helper method to get slots from the request envelope. """

    try: return handler_input.request_envelope.request.api_request.slots
    except Exception as ex:
        logging.error('Error occurred: ', ex)
        return False

def setup_logging():

    """ Helper method to setup logging. """

    # Setup Logging
    logger = logging.getLogger('akshay')
    logger.setLevel(logging.DEBUG)
    sh = logging.StreamHandler()
    sh.setLevel(logging.DEBUG)
    logger.addHandler(sh)

    return logger

def custom_API_response(handler_input: HandlerInput, command:Command, success_string:str, fail_string:str='Action not feasible'):

    # Check Action Feasibility
    feasibility = check_action_feasibility(command)

    if isinstance(feasibility, bool) and feasibility:

        # Publish ROS Message
        SkillNode.send_command(command)

        # Return Success API Response
        return handler_input.response_builder.set_api_response({
                'status': SUCCESS,
                'string': success_string
            }).set_should_end_session(True).response

    elif isinstance(feasibility, bool) and not feasibility:

        # Return Failed API Response
        return handler_input.response_builder.set_api_response({
                'status': FAIL,
                'string': fail_string
            }).set_should_end_session(True).response

    else:

        # Get Solution
        try: _, solution = feasibility
        except: _, solution = False, 'Unknown Error'

        # Return Failed API Response + Solution
        return handler_input.response_builder.set_api_response({
                'status': FAIL,
                'string': fail_string + ': ' + solution,
            }).set_should_end_session(False).response


def check_action_feasibility(command:Union[Command, PickCommand, MoveCommand]) -> Union[bool, Tuple[bool, str]]:

    """ Helper method to check if the action is feasible. """

    # Server URL
    url = 'http://127.0.0.1:5000/check_action'

    # Default Payload
    payload = {
        'name': command.getName(),
        'ID'  : command.getID(),
        'type': command.getType(),
        'info': command.getInfo()
    }

    if command.getType() == MOVE:

        # GoTo Command
        if command.location != 'null':

            # Add Location Information to the Payload
            payload.update({
                'location': command.location
            })

        # Move Command
        else:

            # Add Movement Information to the Payload
            payload.update({
                'direction': command.direction,
                'distance' : command.distance,
                'measure'  : command.measure
            })

    elif command.getType() == PICK:

        # Add Pick Information to the Payload
        payload.update({
            'object_name': command.object_name,
            'location'   : command.location
        })

    print(f"Checking action feasibility: {payload}")

    try:

        # Send POST Request -> Get Response
        response = requests.post(url, json=payload)
        response_data = response.json()

        # Check Response
        if response.status_code == 200:

            # Check Feasibility
            if response_data['feasible']: return True

            else:

                # Print Suggested Solution
                print(f"Action not feasible. Suggested solution: {response_data['solution']}")
                return False, response_data['solution']

        elif response.status_code == 400:

            # Print Error Message
            print(f"Error: {response_data.get('message', 'Invalid input')}")
            return False

        elif response.status_code == 401:

            # Print Error Message
            print(f"Error: {response_data.get('message', 'Command Type')}")
            return False

        else:

            # Print Error Message
            print(f"Error: {response_data.get('message', 'Unknown Error')}")
            return False

    # Request Failed
    except requests.exceptions.RequestException as e:

        print(f"Request failed: {e}")
        return False
