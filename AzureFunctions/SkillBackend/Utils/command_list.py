from typing import List

# Command Types Macro
NULL    = 'NULL'
ROS     = 'ROS'
DEFAULT = 'DEFAULT'

MOVE = 'MOVE'
STOP = 'STOP'
PICK = 'PICK'

MOVE_OBJECT  = 'MOVE_OBJECT'
EXECUTE_TASK = 'EXECUTE_TASK'

class Command():

    """ Basic Command Class """

    # Command Types
    command_types = [
        NULL, ROS, DEFAULT,
        MOVE, STOP, PICK,
        MOVE_OBJECT, EXECUTE_TASK
    ]

    def __init__(self, name:str, ID:int, type:str, info:str):

        # Save Command Information
        self.name, self.ID, self.type, self.info = name, ID, type, info

        # Check Command Type
        if type not in self.command_types: raise ValueError(f'Invalid Command Type: {type}')

    def __str__(self):

        return f'{self.name} ({self.ID})'

    # Command Information Getters
    def getName(self): return self.name
    def getID(self):   return self.ID
    def getType(self): return self.type
    def getInfo(self): return self.info

class MoveCommand(Command):

    """ Move Command Class """

    def __init__(self, name:str, ID:int, info:str, distance:float=0.0, direction:str='null', measure:str='null', location:str='null'):

        # Initialize Basic Command
        super().__init__(name, ID, MOVE, info)

        # Save Move Information
        self.direction = direction
        self.distance  = distance
        self.measure   = measure
        self.location  = location

    def __str__(self):

        # Return Move Information
        if self.location != 'null': return f'{super().__str__()} - Location: {self.location}'
        else: return f'{super().__str__()} - Distance: {self.distance}, Direction: {self.direction}, Measure: {self.measure}'

    # Move Information Setters
    def setDirection(self, direction): self.direction = direction
    def setDistance(self, distance):   self.distance  = distance
    def setMeasure(self, measure):     self.measure   = measure
    def setLocation(self, location):   self.location  = location

    # Move Information Getters
    def getDirection(self): return self.direction
    def getDistance(self):  return self.distance
    def getMeasure(self):   return self.measure
    def getLocation(self):  return self.location

class PickCommand(Command):

    """ Pick Command Class """

    def __init__(self, name:str, ID:int, info:str, object_name:str='null', location:str='null'):

        # Initialize Basic Command
        super().__init__(name, ID, PICK, info)

        # Save Pick Information
        self.object_name = object_name
        self.location = location

    def __str__(self):

        # Return Pick Information
        return f'{super().__str__()} - Object: {self.object_name}, Location: {self.location}'

    # Pick Information Setters
    def setObjectName(self, object_name): self.object_name = object_name
    def setLocation(self, location):      self.location    = location

    # Pick Information Getters
    def getObjectName(self): return self.object_name
    def getLocation(self):   return self.location

class CommandList(List[Command]):

    def __init__(self):

        # Initialize the List
        super().__init__()

    # Get Command by ID or Name
    def get_command_by_id(self, command_id): return next((command for command in self if command.getID() == command_id), None)
    def get_command_by_name(self, command_name): return next((command for command in self if command.getName() == command_name), None)

    # Add New Command
    def add_command(self, name: str, ID: int, type: str, info: str, *args):

        # Check Command Type -> Null, Default, ROS Command
        if type in [NULL, ROS, DEFAULT, STOP]: command = Command(name, ID, type, info)

        # Move Command
        elif type == MOVE:

            # Check Arguments
            if len(args) != 3:

                # Create Empty Move Command
                command = MoveCommand(name, ID, info)

                # raise ValueError('MOVE command requires distance, direction, and measure')

            else:

                # Create Move Command
                distance, direction, measure = args
                command = MoveCommand(name, ID, info, distance, direction, measure)

        # Pick Command
        elif type == PICK:

            # Check Arguments
            if len(args) != 2:

                # Create Empty Pick Command
                command = PickCommand(name, ID, info)

                # raise ValueError('PICK command requires object_name and location')

            else:

                # Create Pick Command
                object_name, location = args
                command = PickCommand(name, ID, info, object_name, location)

        # Add Command to the List
        self.append(command)

    # Add New Command
    def add_command(self, name:str, ID:int, type:str, info:str):

        # Create New Command
        command = Command(name, ID, type, info)

        # Add Command to the List
        self.append(command)

    def delete_command_by_id(self, command_id):

        # Find Command by ID
        command = self.get_command_by_id(command_id)

        # Remove Command from the List
        if command: self.remove(command)

    def delete_command_by_name(self, command_name):

        # Find Command by Name
        command = self.get_command_by_name(command_name)

        # Remove Command from the List
        if command: self.remove(command)

    def getCommandTypes(self):

        # Return Command Types
        return Command.command_types

# Create Command List
command_list = CommandList()

# Add Commands
command_list.add_command('NULL',             0, NULL,     'No Command')
command_list.add_command('EXPERIMENT_START', 1, ROS,      'Start the Experiment')
command_list.add_command('MOVE_DIRECTION',   2, MOVE,     'Move to a Direction',)
command_list.add_command('MOVE_GOTO',        3, MOVE,     'GoTo a Location',)
command_list.add_command('STOP',             4, STOP,     'Stop the Robot')

command_list.add_command('PICK_OBJECT',      5, PICK,     'Pick Up Object')
