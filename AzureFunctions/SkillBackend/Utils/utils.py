import ask_sdk_core.utils as ask_utils
import logging

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
