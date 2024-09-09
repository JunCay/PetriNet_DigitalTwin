from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_list = []
    
    sr_state_sync_p = Node(
        package="dtefm_physical",
        executable="ether_gate_p",
        output='log',
    )
    launch_list.append(sr_state_sync_p)
    
    launch_description = LaunchDescription(
        launch_list
    )
    
    return launch_description