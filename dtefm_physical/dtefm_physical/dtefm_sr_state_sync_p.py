import rclpy
from rclpy.node import Node
from dtefm_interfaces.srv import SRTcpCommunication

class SRSynchronizer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        # self.sr_state_publisher_ = self.create_publisher(SRState, '/sr/robot/state', 10)
        self.sr_state_request_client_ = self.create_client(SRTcpCommunication, '/ether_bridge/physical')
        self.timer = self.create_timer(0.1, self.sr_state_sync_timer_callback)
        
        while rclpy.ok() and self.sr_state_request_client_.wait_for_service(1) == False:
            self.get_logger().info(f'waiting for sr_tcp_communication server')
        
    def sr_state_sync_timer_callback(self):
        command = "$,1,<seq>,RPOS,R,"
        request = SRTcpCommunication.Request()
        request.data = command
        self.sr_state_request_client_.call_async(request).add_done_callback(self.sr_synchronize_callback)
    
    def sr_synchronize_callback(self, result):
        response = result.result()
        self.get_logger().info(f'receive result: {response.responses[0]}')   
        
        
def main(args=None):
    rclpy.init(args=args)
    
    node = SRSynchronizer('sr_synchronizer')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    

   