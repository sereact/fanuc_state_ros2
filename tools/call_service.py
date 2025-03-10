from sereact_custom_messaging.srv import BoolSrv

import rclpy
from rclpy.node import Node

class CallService(Node):
    def __init__(self, service_name):
        super().__init__("call_service")
        self.service_name = service_name
        self.client = self.create_client(BoolSrv, self.service_name)
        
        while not self.client.wait_for_service(1.0):
            self.get_logger().info(f"Waiting for {self.service_name} service to be available...")
        
        self.request = BoolSrv.Request()
        
        self.call_service()
        
    def call_service(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info(f"Service {self.service_name} called successfully")
        else:
            self.get_logger().error(f"Failed to call service {self.service_name}")
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description="Call a service with a boolean request")
    parser.add_argument("--service_name", type=str, required=True, help="Name of the service to call")
    return parser.parse_args()
            
            

if __name__ == "__main__":
    args = parse_arguments()
    rclpy.init()
    node = CallService(args.service_name)
    node.destroy_node()
    rclpy.shutdown()
        
        
        
        
        
        
        