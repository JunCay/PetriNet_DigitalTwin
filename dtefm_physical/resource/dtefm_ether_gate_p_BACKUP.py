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
from dtefm_interfaces.msg import SRState
from concurrent.futures import ThreadPoolExecutor, wait
from dtefm_interfaces.srv import SRTcpCommunication

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
        self.sr_tcp_sending_server_ = self.create_service(SRTcpCommunication, '/ether_bridge/physical', self.sr_tcp_communication_callback)
        self.sr_state_publisher_ = self.create_publisher(SRState, '/sr/robot/state', 10)
        self.host = '10.0.0.2'
        self.port = 10110
        self.step = False
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_socket()
        self.debug = False
        self.sequence_number = [random.randint(20, 40)]
        self.command_interval = 0.02
        self.thread_pool = ThreadPoolExecutor(max_workers=10)
        self.thread_lock = multiprocessing.Lock()
        self.futures = {}
    
    def connect_socket(self):
        try:
            self.s.connect((self.host, self.port))
        except:
            self.get_logger().error(f'node {self.name} socket connection failed...')
                
    # Send command
    def send_command(self, full_command):
        command_type = get_command_n(full_command, 3)
        command_seq = get_command_n(full_command, 2)
        responses = []

        self.s.sendall(full_command.encode())
        self.get_logger().info(f"Sent: {full_command}")

        finished = False
        
        while not finished:
            response = self.s.recv(1024).decode()
            splitted_response = [resp for resp in response.split('\r') if resp]
            self.get_logger().info(f"splitted: {splitted_response}")
            if len(splitted_response) < 1:
                finished = True
            # response = splitted_response[-1]
            
            for response in splitted_response:
                self.get_logger().info(f"dealing with: {response} | seq: {self.sequence_number[0]}")
                resp_type = get_command_n(response, 5)
                resp_seq = get_command_n(response, 2)
                
                if command_type[0] == 'R' or command_type[0] == 'S':
                    finished = True
                if command_type == 'RPOS':
                    msg = SRState()
                    msg.th = float(get_command_n(response, 7))/1000
                    msg.ex = float(get_command_n(response, 8))/1000
                    msg.h1 = float(get_command_n(response, 9))/1000
                    msg.h2 = float(get_command_n(response, 10))/1000
                    msg.z = float(get_command_n(response, 11))/1000
                    print(self.sequence_number[0], ': ', msg)
                if response.startswith('!'):
                    if resp_type == command_type and resp_seq == command_seq:
                        responses.append(response)
                    if get_command_n(response, 4) == '0000':
                        ack_command = f"$,1,<seq>,ACKN,"
                        with self.thread_lock:
                            self.sequence_number[0] = (self.sequence_number[0] + 1)%100
                            ack_command_with_sequence = ack_command.replace('<seq>', str(self.sequence_number[0]).zfill(2))
                            ack_checksum = calculate_checksum(ack_command_with_sequence)
                            full_ack_command = f"{ack_command_with_sequence}{ack_checksum}\r"
                            self.s.sendall(full_ack_command.encode())
                            self.get_logger().info(f"Sent ACK: {full_ack_command}")
                        responses.append(full_ack_command)
                        finished = True
                elif response.startswith('>'):
                    deal_with_error_response(response)
                    responses.append(response)
                    finished = True
                elif response.startswith('?'):
                    responses.append(response)
                    finished = True
                elif response.startswith('$'):
                    deal_with_normal_response(response)
                    if resp_type == command_type and resp_seq == command_seq:
                        responses.append(response)
                else:
                    self.get_logger().error("Invalid response from sr100 server")
                    finished = True
            
            # print(f"splited: {splited_response}")
            # for resp in splitted_response:
                
        return responses

    def tcp_sending(self, command, responses):
        # replace <seq> and add check sum
        command_type = get_command_n(command,3)
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
            
            responses.responses = self.send_command(command, self.sequence_number)
            # print(responses.responses)
            
        
        time.sleep(self.command_interval)

        if responses.responses == None:
            responses.responses = []
        # self.get_logger().info(f"received response: {responses}")
        return responses
        
    def sr_tcp_communication_callback(self, request, responses):
        command = request.data
        future = self.thread_pool.submit(self.tcp_sending, command, responses)
        if self.step:
            while not future.done():
                time.sleep(0.01)
            self.get_logger().info(f"server response: {responses}")
            return responses
        else:
            responses.responses = [f'command sent: {command}', f'{self.sequence_number}']
            return responses
    

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = SRTcpGate('ether_gate_p')
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
