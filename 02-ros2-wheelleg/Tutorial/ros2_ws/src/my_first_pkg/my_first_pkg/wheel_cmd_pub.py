import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd

class Talker(Node):
    def __init__(self):
        super().__init__('my_talker')
        self.publisher = self.create_publisher(WheelLegCmd,'my_wheel_cmd',10)
        self.timer = self.create_timer(0.5,self.timeFeedBack)
        self.timeCount = 0
        
    
    def timeFeedBack(self):
        msg = WheelLegCmd()
        msg.right_phi1 ,msg.right_phi4, msg.left_phi1,msg.left_phi4,msg.right_wheel,msg.left_wheel = (0.0,0.0,0.0,0.0,0.1,0.1)
        self.publisher.publish(msg)
        self.get_logger().info(f'发布：右轮"{msg.right_wheel}"，左轮"{msg.left_wheel}",count:{self.timeCount}')
        self.timeCount+=1

def main(args=None):
    rclpy.init(args=args)
    node=Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()






