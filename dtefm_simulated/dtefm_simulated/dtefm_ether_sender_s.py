import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dtefm_interfaces.srv import SRTcpCommunication
from dtefm_interfaces.msg import EFMCommand
from concurrent.futures import ThreadPoolExecutor

#

class EtherSenderP(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sr_command_client_ = self.create_client(SRTcpCommunication, '/ether_bridge/simulate')
        self.physical_command_subscriber_ = self.create_subscription(EFMCommand, '/command/efm/simulate', self.simulate_command_callback, 10)
        self.thread_pool = ThreadPoolExecutor(max_workers=10)
        self.type = 'INPUT'
        self.command_input_sending()
        
    def command_input_sending(self):
        while True:
            command = input("Enter command: ")
            if command == 'exit':
                return
            request = SRTcpCommunication.Request()
            request.data = command
            self.thread_pool.submit(self.call_task, request)
            
    def simulate_command_callback(self, msg:EFMCommand):
        request = SRTcpCommunication.Request()
        request.data = msg.data
        self.thread_pool.submit(self.call_task, request)
                
    
    def call_task(self, request):
        self.sr_command_client_.call_async(request).add_done_callback(self.sr_command_callback)
        
    
    def sr_command_callback(self, result):
        responses = result.result()
        print(responses)
        # pt = responses.responses[0]
        # self.get_logger().info(f'received result: {pt}')
        
        
        
def main(args=None):
    rclpy.init(args=args)
    
    node = EtherSenderP('ether_sender_p')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()