import rclpy
import os
import sys
import csv
import ast
import time
from rclpy.node import Node
from dtefm_interfaces.msg import PlaceMsg, TransitionMsg, ArcMsg
from dtefm_interfaces.srv import PNCommand
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('dtefm_core')
inital_file_path = os.path.join(package_share_directory, 'resource/initial_file')

class IdentityInitializer(Node):
    def __init__(self, name, initial_net='default'):
        super().__init__(name)
        self.identity_pn_initializer_client_ = self.create_client(PNCommand, '/identity/pn_srv')
        self.pn_initial_command_file = os.path.join(inital_file_path, f'dtefm_pn_initial_command_{initial_net}.csv')
        self.pn_initial_commands = self.get_identity_pn_initial_command_from_file(self.pn_initial_command_file)
        self.request_by_sequence(self.identity_pn_initializer_client_, self.pn_initial_commands)
        
    def get_identity_pn_initial_command_from_file(self, pn_initial_file):
        request_lists = [[],[]]
        with open(pn_initial_file, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                row = {k.strip(): v.strip() for k, v in row.items()}
                command = row['command']
                args = ast.literal_eval(row['args'])
                rqt = PNCommand.Request()
                rqt.command = command
                rqt.args = args
                if command != 'MADA':
                    request_lists[0].append(rqt)
                else:
                    request_lists[1].append(rqt)
        return request_lists
                
    def request_by_sequence(self, client, request_sequence):
        for request in request_sequence[0]:
            client.call_async(request)
            time.sleep(0.01)
        time.sleep(0.01)
        for request in request_sequence[1]:
            client.call_async(request)
            time.sleep(0.01)
            
        
        
                
                
def main(args=None):
    rclpy.init(args=args)
    
    node = IdentityInitializer('identity_initializer', initial_net='efm')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()