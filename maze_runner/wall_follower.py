import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimpleOpenSpaceDriver(Node):
    """
    Very simple corridor / maze driver:

    - Default: go straight.
    - If something is CLOSE in front: turn toward the more open side (left-front vs right-front).
    - Otherwise: if one side wall is too close, steer away slightly.
    - If not close to any wall: go straight (no constant centering).

    This avoids "always centering" twitching.
    """

    def __init__(self):
        super().__init__("simple_open_space_driver")

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_forward_deg = float(self.declare_parameter("scan_forward_deg", 0.0).value)

        # Speeds
        self.v = float(self.declare_parameter("v", 1.5).value)
        self.v_turn = float(self.declare_parameter("v_turn", 0.4).value)   # keep moving while turning
        self.w = float(self.declare_parameter("w", 1.0).value)
        self.w_max = float(self.declare_parameter("w_max", 2.0).value)

        # Distances (simple)
        self.front_stop = float(self.declare_parameter("front_stop", 1.2).value)  # if front < this => turn
        self.side_close = float(self.declare_parameter("side_close", 0.7).value)  # if side < this => push away

        # Sectors
        self.front_half_deg = float(self.declare_parameter("front_half_deg", 18.0).value)
        self.diag_half_deg = float(self.declare_parameter("diag_half_deg", 12.0).value)
        self.side_half_deg = float(self.declare_parameter("side_half_deg", 15.0).value)

        self.lf_deg = float(self.declare_parameter("lf_deg", 45.0).value)
        self.rf_deg = float(self.declare_parameter("rf_deg", -45.0).value)

        self.scan = None
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def sector_percentile(self, center_deg_rel: float, half_width_deg: float, q: float) -> float:
        """q in [0,1]. Use small q (0.2-0.35) to represent 'nearest typical' in that sector."""
        if self.scan is None or self.scan.angle_increment == 0.0:
            return float("inf")

        scan = self.scan
        rmin = scan.range_min if scan.range_min > 0.0 else 0.0
        rmax = scan.range_max if scan.range_max > 0.0 else float("inf")

        center = math.radians(self.scan_forward_deg + center_deg_rel)
        half = math.radians(half_width_deg)

        vals = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            d = (ang - center + math.pi) % (2.0 * math.pi) - math.pi
            if abs(d) <= half:
                vals.append(r)

        if not vals:
            return float("inf")

        vals.sort()
        idx = int(self.clamp(q, 0.0, 1.0) * (len(vals) - 1))
        return vals[idx]

    def tick(self):
        cmd = Twist()
        if self.scan is None:
            self.pub.publish(cmd)
            return

        # Measurements
        front = self.sector_percentile(0.0, self.front_half_deg, 0.25)
        lf = self.sector_percentile(self.lf_deg, self.diag_half_deg, 0.25)
        rf = self.sector_percentile(self.rf_deg, self.diag_half_deg, 0.25)
        left = self.sector_percentile(+90.0, self.side_half_deg, 0.35)
        right = self.sector_percentile(-90.0, self.side_half_deg, 0.35)

        # 1) Default straight
        cmd.linear.x = self.v
        cmd.angular.z = 0.0

        # 2) If front is close, turn toward open side
        if front < self.front_stop:
            turn_left = lf > rf
            cmd.linear.x = self.v_turn
            cmd.angular.z = +self.w if turn_left else -self.w
            cmd.angular.z = self.clamp(cmd.angular.z, -self.w_max, self.w_max)
            self.pub.publish(cmd)
            return

        # 3) If a side wall is too close, gently push away (small correction only)
        #    (If neither side is close -> stay straight)
        if math.isfinite(left) and left < self.side_close and (not math.isfinite(right) or right >= self.side_close):
            # too close to left => steer right
            cmd.angular.z = -0.4 * self.w
        elif math.isfinite(right) and right < self.side_close and (not math.isfinite(left) or left >= self.side_close):
            # too close to right => steer left
            cmd.angular.z = +0.4 * self.w
        elif math.isfinite(left) and math.isfinite(right) and (left < self.side_close) and (right < self.side_close):
            # both close -> steer toward the more open side
            cmd.angular.z = +0.4 * self.w if left > right else -0.4 * self.w

        cmd.angular.z = self.clamp(cmd.angular.z, -self.w_max, self.w_max)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOpenSpaceDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
