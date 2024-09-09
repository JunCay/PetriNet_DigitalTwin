import rclpy
import re
import socket
from rclpy.node import Node
from std_msgs.msg import String
from dtefm_interfaces.msg import EAPCommand, EFMCommand
from dtefm_interfaces.srv import EAPCommandAnalysis, EAPCommandState

# dtefm_command_gate:
#   listen to eap command in EAPCommand type from the topic of '/command/eap/decoded'
#   analyse the eap command using the service /command/eap/analysis, the result will be saved at the service



class CommandGate(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        self.eap_command_subscriber_ = self.create_subscription(EAPCommand, '/command/eap/decoded', self.eap_command_analysis, 10)
        self.efm_command_p_publisher_ = self.create_publisher(EFMCommand, '/command/efm/physical', 10)
        self.efm_command_s_publisher_ = self.create_publisher(EFMCommand, '/command/efm/simulate', 10)
        self.eap_command_analysis_client_ = self.create_client(EAPCommandAnalysis, '/command/eap/analysis_srv')

    def eap_command_analysis(self, msg:EAPCommand):
        snfn = msg.snfn
        cmd = msg.item
        while rclpy.ok() and self.eap_command_analysis_client_.wait_for_service(1) == False:
            self.get_logger().info(f'waiting for eap analysis server')
        request = EAPCommandAnalysis.Request()
        request.snfn = snfn
        request.item = cmd
        self.eap_command_analysis_client_.call_async(request).add_done_callback(self.eap_command_analysis_callback)
        
    def eap_command_analysis_callback(self, analysis_result):
        response = analysis_result.result()
        self.get_logger().info(f'received analysis result: {response}')
        
        
def main(args=None):
    rclpy.init(args=args)
    
    node = CommandGate('EAP_Command_Gate')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    

   