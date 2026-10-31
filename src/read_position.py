import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class Reader(Node):
    def __init__(self):
        super().__init__("reader")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_msg, qos
        )

    def on_msg(self, msg):
        print(f"x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}")


rclpy.init()
rclpy.spin(Reader())
