import rclpy
import sys
import os
import ast
import csv
import time
import torch
import random
import re
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
        self.simulate_command_client_ = self.create_client(SRTcpCommunication, '/sr/command/simulate')
        self.physical_command_client_ = self.create_client(SRTcpCommunication, '/ether_bridge/physical')
        self.intension_subscriber_ = self.create_subscription(GCPNActionMsg, '/identity/agent/action', self.intension_distribute_callback, 10)
        self.simulate_command_monitor_server_ = self.create_service(SRTcpCommunication, '/sr/command/simulate/callback', self.simulate_cmd_callback)
        self.physical_command_monitor_server_ = self.create_service(SRTcpCommunication, '/sr/command/physical/callback', self.physical_cmd_callback)
        
        
        self.identity_gate_control_server_ = self.create_service(IntensionGateControlSrv, '/identity/gate/control', self.intension_gate_control_callback)
        self.intension_gate_state = {'mute': 0, 'remain': 1, 'execute':0}
        self.transition_list = None
        self.action2command = dict()
        self.sequence_number = random.randint(1,99)
        
        self.sending_request = 0
        self.last_action_index = 0
        time.sleep(2)
        self.update_pn()
        self.get_logger().info(f"pn initial command sent")
        
        self.a2c_initial_file = os.path.join(inital_file_path, f'neural_petri_net_lock_action_command.csv')
        self.get_action_command_from_file(self.a2c_initial_file)
        
        
    def get_action_command_from_file(self, a2c_initial_file):
        with open(a2c_initial_file, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                row = {k.strip(): v.strip() for k, v in row.items()}
                command = row['command']
                action = row['action']
                self.action2command[action] = command
                
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
        self.get_logger().info(f"intension gate update_pn command sent")
        
    def update_pn_(self, result):
        self.get_logger().info(f"intension gate update_pn command received callback")

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
        
        if self.last_action_index == action_index:
            self.get_logger().info(f"Command Ocupied-Idling")
            return
        
        # if self.sending_request:
        #     self.get_logger().info(f"Command is sending. Blocked")
        #     return 
        
        self.last_action_index = action_index
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
                command = self.action2command[str(action_index)]
                command_with_sequence = command.replace('<seq>', str(self.sequence_number).zfill(2))
                checksum = self.calculate_checksum(command_with_sequence)
                command = f"{command_with_sequence}{checksum}\r"
                request = SRTcpCommunication.Request()
                request.data = command
                request.transition_name = action_name
                self.sending_request = 1
                
                pnc = PNCommand.Request()
                pnc.command = 'SFRT'
                pnc.args = [action_name]
                self.pn_core_client_.call_async(pnc)
                
                self.simulate_command_client_.call_async(request)
                self.sequence_number = (self.sequence_number + 1)%100
                # self.get_logger().info(f"action {action_index} sent to simulate executor")
                self.get_logger().info(f"s: {command}")
                
            elif self.intension_gate_state['execute'] == 1:
                command = self.action2command[str(action_index)]
                if command == '':
                    self.get_logger().info(f"place holder command")
                    return
                request = SRTcpCommunication.Request()
                request.data = command
                request.transition_name = action_name
                self.sending_request = 1
                
                pnc = PNCommand.Request()
                pnc.command = 'MFRT'
                pnc.args = [action_name]
                self.pn_core_client_.call_async(pnc)
                
                self.physical_command_client_.call_async(request)
                self.sequence_number = (self.sequence_number + 1)%100
                # self.get_logger().info(f"action {action_index} sent to physical executor")
                self.get_logger().info(f"sent p: {command}")

    def simulate_cmd_callback(self, request, response):
        self.sending_request = 0
        action_name = request.transition_name
        print(f"{action_name} finished")
        pnc = PNCommand.Request()
        pnc.command = 'SOFT'
        pnc.args = [action_name]
        self.pn_core_client_.call_async(pnc)
        return response
    
    def physical_cmd_callback(self, request, response):
        self.sending_request = 0
        action_name = request.transition_name
        self.get_logger().error(f"{action_name} finished")
        pnc = PNCommand.Request()
        pnc.command = 'MOFT'
        pnc.args = [action_name]
        self.pn_core_client_.call_async(pnc)
        return response
    
    def calculate_checksum(self, data):
        checksum = 0
        for i in range(1,len(data)):
            c = data[i]
            checksum += ord(c)
        checksum = str(hex(checksum)).upper()
        return checksum[-2:]

    def get_command_n(command, n):
        parsed = command.split(',')
        
        if len(parsed) < n+1:
            return None
        else:
            if not re.search(r'[$!>?]',parsed[0]):
                return None
            # if not re.search(r'[A-Z]{4}',parsed[3]):
            #     return None
            return parsed[n]
    
def main(args=None):
    rclpy.init(args=args)
    
    node = IntensionGate('Intension_Gate')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
            
            
        