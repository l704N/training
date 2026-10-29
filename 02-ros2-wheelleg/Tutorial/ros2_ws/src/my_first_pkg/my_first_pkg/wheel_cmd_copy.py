import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd

class Listener(Node):
    def __init__(self):
        super().__init__('my_listener')
        self.listener = self.create_subscription(WheelLegCmd,'my_wheel_cmd',self.srvCallback,10)

    def srvCallback(self,msg):
        self.get_logger().info(f'收到：右轮"{msg.right_wheel}"，左轮"{msg.left_wheel}"')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
