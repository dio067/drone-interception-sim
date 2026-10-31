import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class Offboard(Node):
    def __init__(self):
        super().__init__("offboard")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", q
        )
        self.pub_set = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", q
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", q
        )
        self.n = 0
        self.create_timer(0.1, self.loop)

    def loop(self):
        m = OffboardControlMode()
        m.position = True
        self.pub_mode.publish(m)

        s = TrajectorySetpoint()
        s.position = [0.0, 0.0, -5.0]
        self.pub_set.publish(s)

        if self.n == 20:
            self.cmd(176, 1.0, 6.0)
        if self.n == 30:
            self.cmd(400, 1.0, 0.0)
        self.n += 1

    def cmd(self, command, p1, p2):
        c = VehicleCommand()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        self.pub_cmd.publish(c)


rclpy.init()
rclpy.spin(Offboard())
