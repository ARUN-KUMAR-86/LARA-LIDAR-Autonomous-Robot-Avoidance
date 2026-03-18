#!/usr/bin/env python3
"""
vfh_controller.py — VFH + LiDAR-based collision recovery
Recovery triggers when:
  - A LiDAR ray is closer than COLLISION_DIST (touching obstacle)
  AND
  - Robot hasn't moved meaningfully in STUCK_TIME_THRESHOLD seconds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class VFHController(Node):

    NUM_SECTORS      = 72
    SECTOR_DEG       = 360.0 / 72

    THRESHOLD        = 10.0
    OBSTACLE_DIST    = 2.5
    ROBOT_RADIUS     = 0.13
    CERTAINTY_WEIGHT = 1.0
    MIN_RANGE_CUTOFF = 0.11

    MAX_LINEAR       = 0.22
    MAX_ANGULAR      = 0.65
    VALLEY_MIN_WIDTH = 2

    # ── Collision / stuck detection ───────────────────────────
    COLLISION_DIST       = 0.25   # metres — LiDAR ray THIS close = touching
    COLLISION_SECTORS    = 5      # how many rays must be close to confirm hit
    STUCK_DIST_THRESHOLD = 0.03   # metres — displacement check
    STUCK_TIME_THRESHOLD = 1.5    # seconds — how long before recovery fires
    CHECK_INTERVAL       = 0.3    # seconds between position snapshots

    # ── Recovery motion ───────────────────────────────────────
    REVERSE_SPEED         = -0.18
    RECOVERY_ANGULAR      = 0.70
    RECOVERY_REVERSE_TIME = 1.8
    RECOVERY_TURN_TIME    = 2.0

    def __init__(self):
        super().__init__('vfh_controller')

        self.declare_parameter('threshold',        self.THRESHOLD)
        self.declare_parameter('obstacle_dist',    self.OBSTACLE_DIST)
        self.declare_parameter('max_linear',       self.MAX_LINEAR)
        self.declare_parameter('max_angular',      self.MAX_ANGULAR)
        self.declare_parameter('valley_min',       self.VALLEY_MIN_WIDTH)
        self.declare_parameter('min_range_cutoff', self.MIN_RANGE_CUTOFF)

        self._refresh_params()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.dbg_pub  = self.create_publisher(String, '/vfh_debug', 10)

        # ── State ─────────────────────────────────────────────
        self.latest_scan      = None
        self.pos_x            = 0.0
        self.pos_y            = 0.0
        self.odom_ready       = False

        # Snapshots for progress check
        self.snap_x           = 0.0
        self.snap_y           = 0.0
        self.snap_time        = self.get_clock().now()

        # Stuck timer
        self.stuck_since      = None

        # Recovery state machine
        self.state            = 'NORMAL'   # NORMAL | REVERSING | TURNING
        self.recovery_start   = None
        self.recovery_dir     = 1          # +1=left, -1=right

        self.last_scan_time   = self.get_clock().now()

        # Timers
        self.create_timer(0.1,  self._control_loop)   # 10 Hz
        self.create_timer(0.5,  self._watchdog)

        self.get_logger().info('VFH + Collision Recovery started ✔')
        self.get_logger().info(
            f'  collision_dist={self.COLLISION_DIST}m  '
            f'stuck_time={self.STUCK_TIME_THRESHOLD}s')

    # ──────────────────────────────────────────────────────────
    def _refresh_params(self):
        self.threshold        = self.get_parameter('threshold').value
        self.obstacle_dist    = self.get_parameter('obstacle_dist').value
        self.max_linear       = self.get_parameter('max_linear').value
        self.max_angular      = self.get_parameter('max_angular').value
        self.valley_min       = int(self.get_parameter('valley_min').value)
        self.min_range_cutoff = self.get_parameter('min_range_cutoff').value

    def _watchdog(self):
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                'No /scan — halted', throttle_duration_sec=3.0)

    def odom_callback(self, msg: Odometry):
        self.pos_x     = msg.pose.pose.position.x
        self.pos_y     = msg.pose.pose.position.y
        self.odom_ready = True

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        self.latest_scan    = msg

    # ══════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP — 10 Hz
    # ══════════════════════════════════════════════════════════
    def _control_loop(self):
        if self.latest_scan is None:
            return

        self._refresh_params()
        now = self.get_clock().now()
        msg = self.latest_scan

        # ══════════════════════════════════════════════════════
        #  RECOVERY STATE MACHINE
        # ══════════════════════════════════════════════════════
        if self.state == 'REVERSING':
            elapsed = (now - self.recovery_start).nanoseconds / 1e9
            if elapsed < self.RECOVERY_REVERSE_TIME:
                self._publish_cmd(self.REVERSE_SPEED, 0.0)
                self._pub_warn(
                    f'◀ REVERSING  {elapsed:.1f}/{self.RECOVERY_REVERSE_TIME}s')
                return
            else:
                self.state          = 'TURNING'
                self.recovery_start = now
                self.get_logger().warn('Recovery: REVERSING done → TURNING')
                return

        if self.state == 'TURNING':
            elapsed = (now - self.recovery_start).nanoseconds / 1e9
            if elapsed < self.RECOVERY_TURN_TIME:
                self._publish_cmd(
                    0.05, self.recovery_dir * self.RECOVERY_ANGULAR)
                self._pub_warn(
                    f'↩ TURNING {"LEFT" if self.recovery_dir>0 else "RIGHT"}'
                    f'  {elapsed:.1f}/{self.RECOVERY_TURN_TIME}s')
                return
            else:
                # Done — reset everything
                self.state       = 'NORMAL'
                self.stuck_since = None
                self._reset_snapshot()
                self.get_logger().info(
                    '✔ Recovery complete — resuming navigation')
                return

        # ══════════════════════════════════════════════════════
        #  NORMAL: compute VFH velocity
        # ══════════════════════════════════════════════════════
        histogram, n_used = self._build_histogram(msg)
        binary    = [1 if h > self.threshold else 0 for h in histogram]
        n_blocked = sum(binary)

        if n_used == 0 or n_blocked == 0:
            linear, angular = self.max_linear, 0.0
            vfh_label = 'CLEAR'
        else:
            valleys = self._find_valleys(binary)
            if not valleys:
                linear, angular = 0.0, self.max_angular * 0.5
                vfh_label = 'SPINNING'
            else:
                best        = self._pick_best_valley(valleys)
                linear, angular = self._compute_velocity(best, binary)
                deg         = best * self.SECTOR_DEG
                if deg > 180: deg -= 360
                vfh_label   = f'NAV {deg:+.0f}°'

        # ══════════════════════════════════════════════════════
        #  COLLISION CHECK — LiDAR based
        #  Count rays closer than COLLISION_DIST
        # ══════════════════════════════════════════════════════
        close_rays = self._count_close_rays(msg, self.COLLISION_DIST)
        touching   = close_rays >= self.COLLISION_SECTORS

        # ══════════════════════════════════════════════════════
        #  PROGRESS CHECK — odometry based
        #  Take a snapshot every CHECK_INTERVAL seconds
        # ══════════════════════════════════════════════════════
        snap_elapsed = (now - self.snap_time).nanoseconds / 1e9
        if snap_elapsed >= self.CHECK_INTERVAL and self.odom_ready:
            dist = math.hypot(
                self.pos_x - self.snap_x,
                self.pos_y - self.snap_y)
            making_progress = dist >= self.STUCK_DIST_THRESHOLD
            self._reset_snapshot()

            if touching and not making_progress:
                # Robot is touching something AND not moving → start stuck timer
                if self.stuck_since is None:
                    self.stuck_since = now
                    self.get_logger().warn(
                        f'⚠ Possible collision: {close_rays} rays < '
                        f'{self.COLLISION_DIST}m, dist={dist*100:.1f}cm')
            else:
                # Moving fine or not touching → reset stuck timer
                if self.stuck_since is not None:
                    self.stuck_since = None

        # ── Check if stuck timer expired ──────────────────────
        if self.stuck_since is not None:
            stuck_elapsed = (now - self.stuck_since).nanoseconds / 1e9
            if stuck_elapsed >= self.STUCK_TIME_THRESHOLD:
                self.get_logger().warn(
                    f'🚨 STUCK! {close_rays} rays < {self.COLLISION_DIST}m '
                    f'for {stuck_elapsed:.1f}s → RECOVERY')
                self._enter_recovery(binary)
                return

        # ══════════════════════════════════════════════════════
        #  PUBLISH VELOCITY
        # ══════════════════════════════════════════════════════
        # Slow down pre-emptively when very close to anything
        if touching:
            linear  = min(linear, 0.08)   # cap speed when touching
            self.get_logger().warn(
                f'⚡ Close contact: {close_rays} rays < {self.COLLISION_DIST}m',
                throttle_duration_sec=0.5)

        self._publish_cmd(linear, angular)
        self._pub_debug(
            f'{vfh_label}  close={close_rays}  '
            f'lin={linear:.2f} ang={angular:.2f}',
            histogram, binary)

    # ══════════════════════════════════════════════════════════
    #  COUNT CLOSE RAYS (collision detection)
    # ══════════════════════════════════════════════════════════
    def _count_close_rays(self, msg: LaserScan, dist: float) -> int:
        count = 0
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                continue
            if r < self.min_range_cutoff:   # self-hit, skip
                continue
            if r < dist:
                count += 1
        return count

    # ══════════════════════════════════════════════════════════
    #  ENTER RECOVERY
    # ══════════════════════════════════════════════════════════
    def _enter_recovery(self, binary: list):
        n          = self.NUM_SECTORS
        left_free  = sum(1 for s in range(1,      n // 2) if binary[s] == 0)
        right_free = sum(1 for s in range(n // 2, n)      if binary[s] == 0)

        self.recovery_dir = 1 if left_free >= right_free else -1
        side = 'LEFT' if self.recovery_dir > 0 else 'RIGHT'

        self.get_logger().warn(
            f'Recovery → turning {side} '
            f'(left_free={left_free} right_free={right_free})')

        self.state          = 'REVERSING'
        self.recovery_start = self.get_clock().now()
        self.stuck_since    = None
        self._reset_snapshot()

    def _reset_snapshot(self):
        self.snap_x    = self.pos_x
        self.snap_y    = self.pos_y
        self.snap_time = self.get_clock().now()

    # ══════════════════════════════════════════════════════════
    #  VFH STEP 1 — BUILD HISTOGRAM
    # ══════════════════════════════════════════════════════════
    def _build_histogram(self, msg: LaserScan):
        histogram = [0.0] * self.NUM_SECTORS
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n_used    = 0

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < self.min_range_cutoff:
                continue
            if r > self.obstacle_dist:
                continue
            n_used   += 1
            certainty = self.CERTAINTY_WEIGHT * (1.0 - r / self.obstacle_dist)
            angle_rad = angle_min + i * angle_inc
            angle_deg = math.degrees(angle_rad) % 360.0

            inflate_deg = math.degrees(
                math.asin(min(self.ROBOT_RADIUS / max(r, 0.05), 1.0)))

            s_min = int((angle_deg - inflate_deg) / self.SECTOR_DEG)
            s_max = int((angle_deg + inflate_deg) / self.SECTOR_DEG) + 1

            for s in range(s_min, s_max + 1):
                histogram[s % self.NUM_SECTORS] += certainty ** 2

        return histogram, n_used

    # ══════════════════════════════════════════════════════════
    #  VFH STEP 2 — FIND VALLEYS
    # ══════════════════════════════════════════════════════════
    def _find_valleys(self, binary: list) -> list:
        n       = len(binary)
        doubled = binary * 2
        valleys = []
        i = 0
        while i < n:
            if doubled[i] == 0:
                j = i
                while j < i + n and doubled[j] == 0:
                    j += 1
                width = j - i
                if width >= self.valley_min:
                    start  = i % n
                    end    = (j - 1) % n
                    centre = (i + width // 2) % n
                    valleys.append((start, end, width, centre))
                i = j
            else:
                i += 1
        seen, unique = set(), []
        for v in valleys:
            k = (v[0], v[1])
            if k not in seen:
                seen.add(k)
                unique.append(v)
        return unique

    # ══════════════════════════════════════════════════════════
    #  VFH STEP 3 — PICK BEST VALLEY
    # ══════════════════════════════════════════════════════════
    def _pick_best_valley(self, valleys: list) -> int:
        n           = self.NUM_SECTORS
        best_sector = None
        best_cost   = float('inf')
        for (start, end, width, centre) in valleys:
            dist        = min(centre, n - centre)
            quarter     = n // 4
            penalty     = max(0, dist - quarter) * 4.0
            width_reward = -width * 0.3
            cost        = float(dist) + penalty + width_reward
            if cost < best_cost:
                best_cost   = cost
                best_sector = centre
        return best_sector

    # ══════════════════════════════════════════════════════════
    #  VFH STEP 4 — COMPUTE VELOCITY
    # ══════════════════════════════════════════════════════════
    def _compute_velocity(self, sector: int, binary: list):
        n         = self.NUM_SECTORS
        angle_deg = sector * self.SECTOR_DEG
        if angle_deg > 180.0:
            angle_deg -= 360.0
        norm    = max(-1.0, min(1.0, angle_deg / 90.0))
        angular = -norm * self.max_angular
        linear  = self.max_linear * (1.0 - abs(norm) * 0.85)
        if any(binary[s % n] for s in range(-2, 3)):
            linear *= 0.3
        linear  = max(0.04, linear)
        angular = max(-self.max_angular, min(self.max_angular, angular))
        return linear, angular

    # ══════════════════════════════════════════════════════════
    #  HELPERS
    # ══════════════════════════════════════════════════════════
    def _publish_cmd(self, linear: float, angular: float):
        msg           = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _pub_warn(self, text: str):
        msg      = String()
        msg.data = f'[VFH] {text}'
        self.dbg_pub.publish(msg)
        self.get_logger().warn(text, throttle_duration_sec=0.3)

    def _pub_debug(self, state: str, histogram: list, binary: list):
        bars = ''.join(
            '█' if binary[i] else
            ('▒' if histogram[i] > self.threshold * 0.3 else '░')
            for i in range(0, self.NUM_SECTORS, 4))
        msg      = String()
        msg.data = f'[VFH] {state}\n  POD: {bars}'
        self.dbg_pub.publish(msg)
        self.get_logger().info(msg.data, throttle_duration_sec=0.5)


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = VFHController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()