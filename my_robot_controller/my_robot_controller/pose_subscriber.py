#!/usb/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber=self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)#ogni volta che riceviamo un messaggio chiamiamo la callback

    def pose_callback(self,msg: Pose):
        self.get_logger().info("(" + str(msg.x) + "," +str(msg.y) +")")



def main (args=None):
    rclpy.init(args=args)

    node = PoseSubscriberNode()

    rclpy.spin(node)
    rclpy.shutdown()