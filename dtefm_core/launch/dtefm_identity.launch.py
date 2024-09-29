from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    launch_list = []
    
    sr_robot_ik_server = Node(
        package="dtefm_middle",
        executable="sr_robot_ik_server",
        output="log",
    )
    launch_list.append(sr_robot_ik_server)
    
    identity_node = Node(
        package="dtefm_middle",
        executable="identity",
        output="screen",
    )
    launch_list.append(identity_node)
    
    identity_initializer = TimerAction(
        period=0.5,
        actions=[
            Node(
                package="dtefm_core",
                executable="identity_initializer",
                output="log",
            )
        ]
    )
    launch_list.append(identity_initializer)
    
    intension_gate = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="dtefm_middle",
                executable="intension_gate",
                output="screen",
            )
        ]
    )
    launch_list.append(intension_gate)
    
    
    
          
    launch_description = LaunchDescription(
        launch_list
    )
    
    return launch_description