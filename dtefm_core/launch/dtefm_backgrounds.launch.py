from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_list = []
    
    eap_command_decoder = Node(
        package="dtefm_core",
        executable="eap_command_decoder",
    )
    launch_list.append(eap_command_decoder)
    
    eap_command_server = Node(
        package="dtefm_core",
        executable="eap_command_server",
    )
    launch_list.append(eap_command_server)
    
    eap_command_gate = Node(
        package="dtefm_core",
        executable="eap_command_gate",
    )
    launch_list.append(eap_command_gate)
    
    ether_gate_p = Node(
        package="dtefm_physical",
        executable="ether_gate_p",
        output='log',
    )
    launch_list.append(ether_gate_p)
    
    # ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
    ether_gate_s = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        output='log',
        parameters=[{'ROS_IP': '0.0.0.0'}],
    )
    launch_list.append(ether_gate_s)
    
    
    
    launch_description = LaunchDescription(
        launch_list
    )
    
    return launch_description