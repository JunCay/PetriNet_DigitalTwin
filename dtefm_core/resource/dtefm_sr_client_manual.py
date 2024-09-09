import socket
import threading
import time
import re
import random
from concurrent.futures import ThreadPoolExecutor
import os

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

def get_command_type(command):
    parsed = command.split(',')
    if len(parsed) < 4:
        return None
    else:
        if parsed[0] != '$':
            return None
        if not re.search(r'[A-Z]{4}',parsed[3]):
            return None
        return parsed[3]
    
# Send command
def send_command(s, command, sequence_number):
    command_type = get_command_type(command)
    if not command_type:
        print("Command invalid")
        return False
    command_with_sequence = command.replace('<seq>', str(sequence_number[0]))
    checksum = calculate_checksum(command_with_sequence)
    full_command = f"{command_with_sequence}{checksum}\r"
    
    # print(f'{command}: connected')
    s.sendall(full_command.encode())
    # s.settimeout(10)
    print(f"Sent: {full_command}")

    while True:
        print("waiting...")
        response = s.recv(1024).decode()
        # if not response:
        #     break
        print(f"Received: {response}")
        if command_type[0] == 'R' or command_type[0] == 'S' or command_type == 'INIT':
            break
        if response.startswith('!'):
            ack_command = f"$,1,<seq>,ACKN"
            sequence_number[0] += 1
            ack_command_with_sequence = ack_command.replace('<seq>', str(sequence_number[0]))
            ack_checksum = calculate_checksum(ack_command_with_sequence)
            full_ack_command = f"{ack_command_with_sequence},{ack_checksum}"
            s.sendall(full_ack_command.encode())
            print(f"Sent ACK: {full_ack_command}")
            break
        elif response.startswith('>'):
            deal_with_error_response(response)
            break
        elif response.startswith('?'):
            break
        elif response.startswith('$'):
            deal_with_normal_response(response)
            continue
    return True
    
# Main function
def main():
    host = '10.0.0.2'
    port = 10110
    sequence_number = [random.randint(20, 40)]
    command_interval = 0.02
    executor = ThreadPoolExecutor(max_workers=4)
    futures = []
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    
    while True:
        command = input("Enter command: ")
        if command == 'exit':
            break
        try:
            future = executor.submit(send_command, s, command, sequence_number)
            futures.append(future)
        except socket.timeout:
            print("Connection timed out. Please try again.")
        sequence_number[0] = (sequence_number[0] + 1)%100
        print(len(futures) - sum(f.done() for f in futures))
        time.sleep(command_interval) 


if __name__ == "__main__":
    main()