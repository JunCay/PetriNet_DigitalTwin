from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    launch_list = []
    
    identity_node = Node(
        package="dtefm_middle",
        executable="identity",
        output="screen",
    )
    launch_list.append(identity_node)
    
    identity_initializer = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="dtefm_core",
                executable="identity_initializer",
                output="log",
            )
        ]
    )
    
    
    launch_list.append(identity_initializer)
          
    launch_description = LaunchDescription(
        launch_list
    )
    
    return launch_description