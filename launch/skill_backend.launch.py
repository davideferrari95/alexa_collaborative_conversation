from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def create_alexa_node():

    # Python Node - Parameters
    node_parameters = {
        'launch_azure':    LaunchConfiguration('azure'),
        'launch_ngrok':    LaunchConfiguration('ngrok'),
        'launch_node_red': LaunchConfiguration('node_red'),
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

    # Example - Arguments
    launch_azure_arg    = DeclareLaunchArgument('azure',    default_value='true')
    launch_ngrok_arg    = DeclareLaunchArgument('ngrok',    default_value='true')
    launch_node_red_arg = DeclareLaunchArgument('node_red', default_value='true')

    # Launch Description - Add Arguments
    launch_description.add_action(launch_azure_arg)
    launch_description.add_action(launch_ngrok_arg)
    launch_description.add_action(launch_node_red_arg)

    # Include Rosbridge Launch File
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(FrontendLaunchDescriptionSource(rosbridge_dir + '/launch/rosbridge_websocket_launch.xml'), launch_arguments={'port':'9091'}.items())
    launch_description.add_action(rosbridge_launch)

    # Launch Description - Add Nodes
    launch_description.add_action(create_alexa_node())

    # Return Launch Description
    return launch_description
