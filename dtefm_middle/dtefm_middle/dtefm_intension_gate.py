import rclpy
import sys
import os
import ast
import csv
import time
import torch
import numpy as np
from rclpy.node import Node
from dtefm_interfaces.msg import SRStateRobot, PlaceMsg, TransitionMsg, ArcMsg, PetriNet, GCPNStateMsg, GCPNActionMsg
from dtefm_interfaces.srv import SRTcpCommunication, SRState, PNCommand, IntensionGateControlSrv, GCPNSrv, IdentityGateControlSrv
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('dtefm_middle')
inital_file_path = os.path.join(package_share_directory, 'resource/action_command')

class IntensionGate(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Intension Gate node {name} initialized..")
        self.pn_core_client_ = self.create_client(PNCommand, '/identity/pn_srv/core')
        self.pn_client_ = self.create_client(PNCommand, '/identity/pn_srv/inner')
        self.intension_subscriber_ = self.create_subscription(GCPNActionMsg, '/identity/agent/action', self.intension_distribute_callback, 10)
        
        self.identity_gate_control_server_ = self.create_service(IntensionGateControlSrv, '/identity/gate', self.intension_gate_control_callback)
        self.intension_gate_state = {'mute': 0, 'remain': 1, 'execute':0}
        self.transition_list = None
        self.action2command = dict()
        self.update_pn()
        
        self.a2c_initial_file = os.path.join(inital_file_path, f'neural_petri_net_lock_action_command.csv')
    
    def get_action_command_from_file(self, a2c_initial_file):
        with open(a2c_initial_file, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
    
    def intension_gate_control_callback(self, request, response):
        self.intension_gate_state['mute'] = request.mute_state
        self.intension_gate_state['remain'] = request.remain_state
        self.intension_gate_state['execute'] = request.execute_state
        
        response = IntensionGateControlSrv.Response()
        response.mute_state = self.intension_gate_state['mute']
        response.remain_state = self.intension_gate_state['remain']
        response.execute_state = self.intension_gate_state['execute']
        
        return response
    
    def find_node_index(self, node_id, node_list):
        for i in range(len(node_list)):
            if node_id == node_list[i].id:
                return i
     
    def update_pn(self):
        request = PNCommand.Request()
        request.command = 'RNET'
        self.pn_core_client_.call_async(request).add_done_callback(self.update_pn_)
        
    def update_pn_(self, result):
        response = result.result()
        places = response.places
        transitions = response.transitions
        arcs = response.arcs
        self.place_list = places
        self.transition_list = transitions
        self.pn_adj = np.zeros((len(places), len(transitions)+1))
        
        for arc in arcs:
            if arc.direction == 'PtoT':
                i = self.find_node_index(arc.node_in, places)
                j = self.find_node_index(arc.node_out, transitions)
                self.pn_adj[i][j] = -1
            elif arc.direction == 'TtoP':
                i = self.find_node_index(arc.node_out, places)
                j = self.find_node_index(arc.node_in, transitions)
                self.pn_adj[i][j] = 1
        self.get_logger().info(f"pn_adj updated {self.pn_adj}")
    
    def intension_distribute_callback(self, msg):
        if self.transition_list == None:
            self.update_pn()
            time.sleep(0.1)
        action_index = msg.action_index
        
        # Direct return when idle
        if action_index == len(self.transition_list):
            self.get_logger().info(f"Idling")
            return
        
        # Direct return if mute
        if self.intension_gate_state['mute'] == 1:
            self.get_logger().info(f"action {action_index} muted")
            return
        
        
        action_name = self.transition_list[action_index].name
        
        if self.intension_gate_state['remain'] == 1:
            pnc = PNCommand.Request()
            pnc.command = 'MFRT'
            pnc.args = [action_name]
            self.pn_client_.call_async(pnc)
            self.get_logger().info(f"action {action_index} sent to pn core")
            
            return 
        elif self.intension_gate_state['remain'] == 0:
            if self.intension_gate_state['execute'] == 0:
                self.get_logger().info(f"action {action_index} sent to simulate executor")
                pass
            elif self.intension_gate_state['execute'] == 1:
                self.get_logger().info(f"action {action_index} sent to physical executor")
                pass

def main(args=None):
    rclpy.init(args=args)
    
    node = IntensionGate('Intension_Gate')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
            
            
        