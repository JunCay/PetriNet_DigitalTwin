import socket
import threading
import multiprocessing
import time
import re
import random
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import concurrent.futures
from dtefm_interfaces.msg import SRStateRobot, SRValues
from concurrent.futures import ThreadPoolExecutor, wait
from dtefm_interfaces.srv import SRTcpCommunication, EtherGateSrv

# Calculate checksum function
def calculate_checksum(data):
    checksum = 0
    for i in range(1,len(data)):
        c = data[i]
        checksum += ord(c)
    checksum = str(hex(checksum)).upper()
    return checksum[-2:]

def deal_with_normal_response(response):
    pass

def deal_with_error_response(response):
    pass

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


class SRTcpGate(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.get_logger().info(f'node {name} created.')
        self.sr_tcp_sending_server_p = self.create_service(SRTcpCommunication, '/ether_bridge/physical', self.sr_tcp_communication_callback_p)      # receive input physical command
        self.sr_tcp_sending_server_s = self.create_service(SRTcpCommunication, '/ether_bridge/simulate', self.sr_tcp_communication_callback_s)      # receive input simulate command
        self.sr_unity_command_client_s = self.create_client(SRTcpCommunication, '/sr/command/simulation')
        self.sr_robot_state_publisher_ = self.create_publisher(SRStateRobot, '/sr/robot/state/physical', 10)
        self.sr_pa_result_publisher_ = self.create_publisher(SRValues, '/sr/pa/result/physical', 10)
        self.host = '10.0.0.2'
        self.port = 10110
        self.port2 = 10111
        self.step = False
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_socket()
        self.debug = False
        self.sequence_number = [random.randint(20, 40)]
        self.command_interval = 0.02
        self.thread_pool_p = ThreadPoolExecutor(max_workers=10)
        self.thread_pool_s = ThreadPoolExecutor(max_workers=10)
        self.thread_lock = multiprocessing.Lock()
        self.futures = {}
        self.ackn_list = ['INIT', 'MTRS']
        
        self.gate_state = {'physical_gate_on': True, 'simulate_gate_on': True, 'physical2simulate':True, 'simulate2physical':False}
        self.gate_service_ = self.create_service(EtherGateSrv, '/ether_gate/state', self.ether_gate_state_callback)
        
        self.thread_pool_p.submit(self.deal_with_response)
        self.thread_pool_p.submit(self.deal_with_response2)
    
    def connect_socket(self):
        try:
            self.s.connect((self.host, self.port))
        except:
            self.get_logger().error(f'node {self.name} socket connect {self.host}:{self.port} failed...')
        
        try:
            self.s2.connect((self.host, self.port2))
        except:
            self.get_logger().error(f'node {self.name} socket connect {self.host}:{self.port2} failed...') 
    
    def deal_with_response2(self):
        while True:
            response = self.s2.recv(1024).decode()
            splitted_response = [resp for resp in response.split('\r') if resp]
            self.get_logger().info(f"received: {splitted_response}")
            
            for response in splitted_response:
                # self.get_logger().info(f"dealing with: {response} | seq: {self.sequence_number[0]}")
                if response.startswith('>'):
                    resp_type = get_command_n(response, 2)
                    resp_seq = get_command_n(response, 2)
                elif response.startswith('$'):
                    resp_type = get_command_n(response, 5)
                    resp_seq = get_command_n(response, 2)
                    
                    if resp_type == 'RALN':
                        msg = SRValues()
                        values = []
                        for i in range(6, 15):
                            values.append(float(get_command_n(response,i)))
                        msg.values = values
                        self.sr_pa_result_publisher_.publish(msg)
                    
                    
                elif response.startswith('!'):
                    resp_type = get_command_n(response, 5)
                    resp_seq = get_command_n(response, 2)
                    if get_command_n(response, 4) == '0000' and resp_type in self.ackn_list:
                        msg = SRStateRobot()
                        msg.th = float(get_command_n(response, 7))/1000
                        msg.ex = float(get_command_n(response, 8))/1000
                        msg.h1 = float(get_command_n(response, 9))/1000
                        msg.h2 = float(get_command_n(response, 10))/1000
                        msg.z = float(get_command_n(response, 11))/1000
                        self.sr_robot_state_publisher_.publish(msg)
          
    def deal_with_response(self):
        while True:
            response = self.s.recv(1024).decode()
            splitted_response = [resp for resp in response.split('\r') if resp]
            self.get_logger().info(f"received: {splitted_response}")
            if len(splitted_response) < 1:
                continue
            
            for response in splitted_response:
                # self.get_logger().info(f"dealing with: {response} | seq: {self.sequence_number[0]}")
                if response.startswith('>'):
                    resp_type = get_command_n(response, 2)
                    resp_seq = get_command_n(response, 2)
                elif response.startswith('$'):
                    resp_type = get_command_n(response, 5)
                    resp_seq = get_command_n(response, 2)
                    
                    if resp_type == 'RPOS':
                        msg = SRStateRobot()
                        msg.th = float(get_command_n(response, 7))/1000
                        msg.ex = float(get_command_n(response, 8))/1000
                        msg.h1 = float(get_command_n(response, 9))/1000
                        msg.h2 = float(get_command_n(response, 10))/1000
                        msg.z = float(get_command_n(response, 11))/1000
                        self.sr_robot_state_publisher_.publish(msg)
                        
                elif response.startswith('!'):
                    resp_type = get_command_n(response, 5)
                    resp_seq = get_command_n(response, 2)
                    if get_command_n(response, 4) == '0000' and resp_type in self.ackn_list:
                        msg = SRStateRobot()
                        msg.th = float(get_command_n(response, 7))/1000
                        msg.ex = float(get_command_n(response, 8))/1000
                        msg.h1 = float(get_command_n(response, 9))/1000
                        msg.h2 = float(get_command_n(response, 10))/1000
                        msg.z = float(get_command_n(response, 11))/1000
                        self.sr_robot_state_publisher_.publish(msg)
                        # print(self.sequence_number[0], ': ', msg)
                        
                        ack_command = f"$,1,<seq>,ACKN,"
                        with self.thread_lock:
                            self.sequence_number[0] = (self.sequence_number[0] + 1)%100
                            ack_command_with_sequence = ack_command.replace('<seq>', str(self.sequence_number[0]).zfill(2))
                            ack_checksum = calculate_checksum(ack_command_with_sequence)
                            full_ack_command = f"{ack_command_with_sequence}{ack_checksum}\r"
                            self.s.sendall(full_ack_command.encode())
                            self.get_logger().info(f"Sent ACK: {full_ack_command}")
                elif response.startswith('!'):  
                    err_type = get_command_n(response, 1)
                else:
                    self.get_logger().error("Invalid response from sr100 server")
                    
    
    # Send command
    def send_command(self, full_command, unit_type):
        responses = []
        # self.get_logger().info(f"sending command: {full_command}")
        if unit_type == '1':
            self.s.sendall(full_command.encode())
        elif unit_type == '2':
            self.s2.sendall(full_command.encode())
        self.get_logger().info(f"Sent: {full_command}")
                
        return responses

    def tcp_sending_p(self, command, responses):
        unit_type = get_command_n(command, 1)
        command_type = get_command_n(command,3)
        # self.get_logger().info(f"get command: {command}")
        if not command_type:
            self.get_logger().error('Command Invalid')
            responses.responses = ['!,COMMAND_INVALID']
            return responses
        
        with self.thread_lock:
            self.sequence_number[0] = (self.sequence_number[0] + 1)%100
            command_with_sequence = command.replace('<seq>', str(self.sequence_number[0]).zfill(2))
            checksum = calculate_checksum(command_with_sequence)
            command = f"{command_with_sequence}{checksum}\r"
        
        if self.debug:
            responses.responses = [f"$,<{command}>,DEBUG", f"!,<{command}>,DEBUGFINISH"]
            time.sleep(5)
            self.get_logger().info(f'{command} finished')
        else:
            # self.get_logger().info(f"to sending command: {command}")
            responses.responses = self.send_command(command, unit_type)
            
            # print(responses.responses)
            
        
        time.sleep(self.command_interval)

        if responses.responses == None:
            responses.responses = []
        # self.get_logger().info(f"received response: {responses}")
        return responses
        
    def sr_tcp_communication_callback_p(self, request, responses):
        if self.gate_state['physical_gate_on'] == True:
            command = request.data
            future = self.thread_pool_p.submit(self.tcp_sending_p, command, responses)
            if self.step:
                while not future.done():
                    time.sleep(0.01)
                self.get_logger().info(f"server response: {responses}")
                return responses
            else:
                responses.responses = [f'command sent: {command}', f'{self.sequence_number}']
                return responses
    
    def tcp_sending_s(self, command, responses):
        unit_type = get_command_n(command, 1)
        command_type = get_command_n(command,3)
        # self.get_logger().info(f"get command: {command}")
        if not command_type:
            self.get_logger().error('Command Invalid')
            responses.responses = ['!,COMMAND_INVALID']
            return responses
        
        with self.thread_lock:
            self.sequence_number[0] = (self.sequence_number[0] + 1)%100
            command_with_sequence = command.replace('<seq>', str(self.sequence_number[0]).zfill(2))
            checksum = calculate_checksum(command_with_sequence)
            command = f"{command_with_sequence}{checksum}\r"
        request = SRTcpCommunication.Request()
        request.data = command
        self.sr_unity_command_client_s.call_async(request).add_done_callback(self.sr_unity_command_callback)
        
    def sr_unity_command_callback(self, result):
        pass
    
    def sr_tcp_communication_callback_s(self, request, responses):
        if self.gate_state['simulate_gate_on'] == True:
            command = request.data
            future = self.thread_pool_s.submit(self.tcp_sending_s, command, responses)
            responses.responses = [f'command sent: {command}', f'{self.sequence_number}']
            return responses
    
    def ether_gate_state_callback(self, request, response):
        # current: string[4]
        if request.request_state[0] == '0':
            self.gate_state['physical_gate_state'] = False
        elif request.request_state[0] == '1':
            self.gate_state['physical_gate_state'] = True
            
        if request.request_state[1] == '0':
            self.gate_state['simulate_gate_state'] = False
        elif request.request_state[1] == '1':
            self.gate_state['simulate_gate_state'] = True
            
        if request.request_state[2] == '0':
            self.gate_state['physical2simulate'] = False
        elif request.request_state[2] == '1':
            self.gate_state['physical2simulate'] = True
            
        if request.request_state[3] == '0':
            self.gate_state['simulate2physical'] = False
        elif request.request_state[3] == '1':
            self.gate_state['simulate2physical'] = True

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = SRTcpGate('ether_gate_p')
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
