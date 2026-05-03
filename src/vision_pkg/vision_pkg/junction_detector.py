import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('junction_detector')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()