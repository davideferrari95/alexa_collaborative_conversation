from typing import List

class Command():

    # Command Types
    command_types = [
        'NULL',
        'TEST',
        'API',
        'ROS'
    ]

    def __init__(self, name:str, ID:int, type:str, info:str):

        # Save Command Information
        self.name, self.ID, self.type, self.info = name, ID, type, info

        # Check Command Type
        if type not in self.command_types: raise ValueError(f'Invalid Command Type: {type}')

        # Properties
        self.area = None
        self.object = None

    def __str__(self):

        return f'{self.name} ({self.ID})'

    # Command Information Getters
    def getName(self): return self.name
    def getID(self):   return self.ID
    def getType(self): return self.type
    def getInfo(self): return self.info

    # Properties Getters
    def getArea(self):   return self.area
    def getObject(self): return self.object

class CommandList(List[Command]):

    def __init__(self):

        # Initialize the List
        super().__init__()

    # Get Command by ID or Name
    def get_command_by_id(self, command_id): return next((command for command in self if command.getID() == command_id), None)
    def get_command_by_name(self, command_name): return next((command for command in self if command.getName() == command_name), None)

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

# Create Command List
command_list = CommandList()

# Add New Commands
command_list.add_command('NULL',                       0,  'NULL', 'No Command')
command_list.add_command('EXPERIMENT_START',           1,  'ROS',  'Start the Experiment')
command_list.add_command('MOVED_OBJECT',               2,  'ROS',  'Object Moved')
command_list.add_command('PUT_OBJECT_IN_AREA',         3,  'ROS',  'Put Object in Area')
command_list.add_command('PUT_OBJECT_IN_GIVEN_AREA',   4,  'ROS',  'Put Object in Given Area')
command_list.add_command('PUT_OBJECT_IN_AREA_GESTURE', 5,  'ROS',  'Put Object in Area with Gesture')
command_list.add_command('USER_MOVED',                 6,  'ROS',  'User Moved')
command_list.add_command('USER_CANT_MOVE',             7,  'ROS',  'User Cant Move')
command_list.add_command('REPLAN_TRAJECTORY',          8,  'ROS',  'Replan Trajectory')
command_list.add_command('WAIT_FOR_COMMAND',           9,  'ROS',  'Wait for Command')
command_list.add_command('CAN_GO',                     10, 'ROS',  'Can Go')
command_list.add_command('WAIT_TIME',                  11, 'ROS',  'Wait Time')
