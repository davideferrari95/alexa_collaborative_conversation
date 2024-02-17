from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path

def create_alexa_node():

    # Get Package Path
    package_path = str(Path(__file__).resolve().parents[1])

    # Python Node - Parameters
    node_parameters = {
        'launch_azure':    LaunchConfiguration('azure'),
        'launch_ngrok':    LaunchConfiguration('ngrok'),
        'launch_node_red': LaunchConfiguration('node_red'),
        'package_path':    package_path,
    }

    # Alexa Skill Back-End - Node
    node = Node(
        package='alexa_conversation', executable='skill_backend_launcher.py', name='skill_backend_launcher',
        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')],
        parameters=[node_parameters]
    )

    return node

def generate_launch_description():

    # Launch Description
    launch_description = LaunchDescription()

    # Arguments
    launch_azure_arg    = DeclareLaunchArgument('azure',      default_value='true')
    launch_ngrok_arg    = DeclareLaunchArgument('ngrok',      default_value='true')
    launch_node_red_arg = DeclareLaunchArgument('node_red',   default_value='true')

    # Launch Description - Add Arguments
    launch_description.add_action(launch_azure_arg)
    launch_description.add_action(launch_ngrok_arg)
    launch_description.add_action(launch_node_red_arg)

    # Launch Description - Add Nodes
    launch_description.add_action(create_alexa_node())

    # Return Launch Description
    return launch_description
