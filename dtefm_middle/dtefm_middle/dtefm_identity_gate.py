import rclpy
import sys
import os
import ast
import time
import torch
import numpy as np
from rclpy.node import Node
from dtefm_interfaces.msg import SRStateRobot, PlaceMsg, TransitionMsg, ArcMsg, PetriNet, GCPNStateMsg, GCPNActionMsg
from dtefm_interfaces.srv import SRTcpCommunication, SRState, PNCommand, IntensionGateControlSrv, GCPNSrv, IdentityGateControlSrv
from ament_index_python.packages import get_package_share_directory

class IdentityGate(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Identity Gate node {name} initialized..")

        self.pn_client_ = self.create_client(PNCommand, '/identity/pn_srv/core')
        self.pn_distribute_server_s = self.create_service(PNCommand, '/identity/pn_srv/simulate', self.pn_distribute_server_callback_s)
        self.pn_distribute_server_i = self.create_service(PNCommand, '/identity/pn_srv/inner', self.pn_distribute_server_callback_i)
        self.pn_distribute_server_p = self.create_service(PNCommand, '/identity/pn_srv/physical', self.pn_distribute_server_callback_p)

        self.identity_gate_control_server_ = self.create_service(IdentityGateControlSrv, '/identity/gate', self.identity_gate_control_callback)
        self.identity_gate_state = 0

        # self.test_call()

    def identity_gate_control_callback(self, request, response):
        self.identity_gate_state = request.control_state
        response.current_control_state = self.identity_gate_state
        return response

    def pn_distribute_server_callback_i(self, request, response):
        self.get_logger().info(f"receive inner PNCommad")
        if not self.identity_gate_state == 0:   # only allow inner intension
            response = PNCommand.Response()
            response.gate_status = 'blocked'
            return response
        self.get_logger().info(f"inner PNCommad distributed")
        self.pn_client_.call_async(request)
        response = PNCommand.Response()
        response.gate_status = 'distributed'
        return response
    
    def pn_distribute_server_callback_s(self, request, response):
        self.get_logger().info(f"receive simute PNCommad")
        if not self.identity_gate_state == 1:   # only allow simulate pn update
            response = PNCommand.Response()
            response.gate_status = 'blocked'
            return response
        response = self.pn_client_.call_async(request)
        response = PNCommand.Response()
        response.gate_status = 'distributed'
        return response
    
    def pn_distribute_server_callback_p(self, request, response):
        self.get_logger().info(f"receive physical PNCommad")
        
        if not self.identity_gate_state == 2:   # only allow physical pn update
            response = PNCommand.Response()
            response.gate_status = 'blocked'
            return response
        
        response = self.pn_client_.call_async(request)
        response = PNCommand.Response()
        response.gate_status = 'distributed'
        return response
    
    def test_call(self):
        request = PNCommand.Request()
        request.command = 'RNET'
        self.get_logger().info(f"start test call")
        future = self.pn_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"inner PNCommad get response {response}")
        return response
        
        
            
def main(args=None):
    rclpy.init(args=args)
    
    node = IdentityGate('identity_gate')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()