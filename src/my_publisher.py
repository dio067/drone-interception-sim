import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_publisher")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = "hello"
        self.pub.publish(msg)


rclpy.init()
node = MyPublisher()
rclpy.spin(node)
