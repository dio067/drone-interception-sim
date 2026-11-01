import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    VehicleLocalPosition,
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
)

TRAJECTORY = "straight"

ALTITUDE = 10.0
OMEGA = 0.15
ZIG_PERIOD = 4.0
TARGET_SCHEDULE = [
    (0, 10, 1.5),
    (10, 20, 2.0),
    (20, 30, 3.0),
    (30, 40, 1.0),
]
HUNTER_SCHEDULE = [
    (0, 10, 2.0),
    (10, 20, 3.0),
    (20, 30, 4.0),
    (30, 40, 5.0),
]
KP = 0.8
STOP_DISTANCE = 2.0
TOPIC = "fmu/out/vehicle_local_position_v1"
NAN = float("nan")


def get_speed(elapsed_time, schedule):
    for start, end, speed in schedule:
        if start <= elapsed_time < end:
            return speed
    return schedule[-1][2]


def direction(elapsed):
    if TRAJECTORY == "circle":
        a = OMEGA * elapsed
        return math.sin(a), math.cos(a)
    if TRAJECTORY == "zigzag":
        return (0.6 if int(elapsed // ZIG_PERIOD) % 2 else -0.6), 0.8
    return 0.0, 1.0


class Drone:
    def __init__(self, node, ns, sys_id):
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node = node
        self.sys_id = sys_id
        self.pos = None
        node.create_subscription(
            VehicleLocalPosition, f"{ns}/{TOPIC}", self.on_pos, qos
        )
        self.mode_pub = node.create_publisher(
            OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos
        )
        self.sp_pub = node.create_publisher(
            TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos
        )
        self.cmd_pub = node.create_publisher(
            VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos
        )

    def on_pos(self, msg):
        self.pos = msg

    def now(self):
        return self.node.get_clock().now().nanoseconds // 1000

    def altitude(self):
        return -self.pos.z

    def send(self, pos=(NAN, NAN, NAN), vel=(NAN, NAN, NAN)):
        mode = OffboardControlMode()
        mode.timestamp = self.now()
        mode.position = not all(math.isnan(p) for p in pos)
        mode.velocity = not all(math.isnan(v) for v in vel)
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.now()
        sp.position = [float(p) for p in pos]
        sp.velocity = [float(v) for v in vel]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = NAN
        self.sp_pub.publish(sp)

    def hold(self, x, y):
        self.send(pos=(x, y, -ALTITUDE))

    def fly(self, v_north, v_east):
        self.send(pos=(NAN, NAN, -ALTITUDE), vel=(v_north, v_east, NAN))

    def arm(self):
        self.command(176, 1.0, 6.0)
        self.command(400, 1.0)

    def command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now()
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = self.sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


class Chase(Node):
    def __init__(self):
        super().__init__("chase")
        self.target = Drone(self, "/uav_1", 2)
        self.hunter = Drone(self, "/uav_2", 3)
        self.chase_start_time = None
        self.phase = "takeoff"
        self.ticks = 0
        self.offset = None
        self.stop_at = None
        self.create_timer(0.1, self.loop)

    def loop(self):
        if self.target.pos is None or self.hunter.pos is None:
            self.get_logger().info(
                "waiting for both drones...", throttle_duration_sec=2.0
            )
            return
        if self.offset is None and not self.find_offset():
            return

        self.ticks += 1
        if self.phase == "takeoff":
            self.takeoff()
        elif self.phase == "chase":
            self.chase()
        else:
            self.target.hold(*self.stop_at[0])
            self.hunter.hold(*self.stop_at[1])

        if self.ticks % 10 == 0:
            self.get_logger().info(
                f"{self.phase} | alt {self.target.altitude():.1f} / "
                f"{self.hunter.altitude():.1f} | gap {self.gap():.1f} m"
            )

    def find_offset(self):
        t, h = self.target.pos, self.hunter.pos
        if not (t.xy_global and h.xy_global):
            self.get_logger().info(
                "waiting for GPS reference...", throttle_duration_sec=2.0
            )
            return False
        north = (h.ref_lat - t.ref_lat) * 111319.5
        east = (h.ref_lon - t.ref_lon) * 111319.5 * math.cos(math.radians(t.ref_lat))
        self.offset = (north, east)
        self.get_logger().info(f"hunter is {north:.1f} m N, {east:.1f} m E of target")
        return True

    def hunter_xy(self):
        h = self.hunter.pos
        return h.x + self.offset[0], h.y + self.offset[1]

    def gap(self):
        if self.offset is None:
            return 0.0
        hx, hy = self.hunter_xy()
        return math.hypot(self.target.pos.x - hx, self.target.pos.y - hy)

    def takeoff(self):
        self.target.hold(0.0, 0.0)
        self.hunter.hold(0.0, 0.0)

        if self.ticks >= 15 and self.ticks % 10 == 0:
            self.target.arm()
            self.hunter.arm()

        if (
            self.target.altitude() > ALTITUDE - 0.5
            and self.hunter.altitude() > ALTITUDE - 0.5
        ):
            self.phase = "chase"
            self.chase_start_time = self.get_clock().now()
            self.get_logger().info("both at altitude, chase starts")

    def chase(self):
        t = self.target.pos
        elapsed = (self.get_clock().now() - self.chase_start_time).nanoseconds / 1e9

        target_speed = get_speed(elapsed, TARGET_SCHEDULE)
        hunter_speed = get_speed(elapsed, HUNTER_SCHEDULE)
        dn, de = direction(elapsed)
        self.target.fly(dn * target_speed, de * target_speed)

        hx, hy = self.hunter_xy()
        gap = self.gap()

        time_to_reach = gap / hunter_speed
        aim_x = t.x + t.vx * time_to_reach
        aim_y = t.y + t.vy * time_to_reach

        dx = aim_x - hx
        dy = aim_y - hy
        dist = math.hypot(dx, dy) or 1e-6
        desired_speed = min(hunter_speed, KP * dist)

        self.hunter.fly(dx / dist * desired_speed, dy / dist * desired_speed)

        if gap <= STOP_DISTANCE:
            self.stop_at = ((t.x, t.y), (self.hunter.pos.x, self.hunter.pos.y))
            self.phase = "stopped"
            self.get_logger().info("within 2 m: both drones stopped")


def main():
    rclpy.init()
    rclpy.spin(Chase())


if __name__ == "__main__":
    main()
