from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_list = []
    
    eap_command_decoder = Node(
        package="dtefm_middle",
        executable="identity",
        output="screen",
    )
    launch_list.append(eap_command_decoder)
          
    launch_description = LaunchDescription(
        launch_list
    )
    
    return launch_description