EXPERIMENT_START            = 0
MOVED_OBJECT                = 1
PUT_OBJECT_IN_AREA          = 2
PUT_OBJECT_IN_GIVEN_AREA    = 3
PUT_OBJECT_IN_AREA_GESTURE  = 4
USER_MOVED                  = 5
USER_CANT_MOVE              = 6
REPLAN_TRAJECTORY           = 7
WAIT                        = 8
CAN_GO                      = 9

command_info = [
    'EXPERIMENT_START',
    'MOVED_OBJECT',
    'PUT_OBJECT_IN_AREA',
    'PUT_OBJECT_IN_GIVEN_AREA',
    'PUT_OBJECT_IN_AREA_GESTURE',
    'USER_MOVED',
    'USER_CANT_MOVE',
    'REPLAN_TRAJECTORY',
    'WAIT',
    'CAN_GO'
]

available_areas = [
    'left',
    'right',
    'front',
    'back',
    'box'
]

gesture_areas = [
    'here',
    'there'
]
