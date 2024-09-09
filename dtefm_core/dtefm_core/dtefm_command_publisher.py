import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String


class CommandPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        self.publisher_ = self.create_publisher(String, '/command/eap/encoded_string', 10)
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = String()
        command = input("Input command: \n")
        msg.data = command
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Published command: "{command}"')

def main():
    rclpy.init()
    node = CommandPublisher('command_publisher')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()