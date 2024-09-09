import rclpy
from rclpy.node import Node
from concurrent.futures import ThreadPoolExecutor
from example_interfaces.srv import AddTwoInts
import time

class MyServer(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.handle_request)
        self.pool = ThreadPoolExecutor(max_workers=5)

    async def handle_request(self, request, response):
        self.get_logger().info('Incoming request: a=%d, b=%d' % (request.a, request.b))
        # 使用线程池异步处理请求
        future = self.pool.submit(self.process_request, request, response)
        while not future.done():
            time.sleep(0.01)
        self.get_logger().info('Response: %d' % response.sum)
        return response

    def process_request(self, request, response):
        # 模拟长时间操作
        response.sum = self.task(request.a, request.b)
        # self.get_logger().info('Response: %d' % response.sum)
        return response
    
    def task(self, a, b):

        time.sleep(a)
        return a+b
    

def main(args=None):
    rclpy.init(args=args)
    server = MyServer('test_server')
    rclpy.spin(server)

    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()