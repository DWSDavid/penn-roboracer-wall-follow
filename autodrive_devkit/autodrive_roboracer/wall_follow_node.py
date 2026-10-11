#!/usr/bin/env python3
"""
Wall-following node for the Penn RoboRacer technical challenge.

Steering (v1, F1TENTH lab 3 wall following)
    Reads the 2D LiDAR scan from the AutoDRIVE bridge and steers with a PID
    controller, centred between both walls (side:=center, default) or a fixed
    distance from one wall (side:=left or side:=right). For each wall, beam b
    is perpendicular to it (+-90 deg) and beam a is theta degrees further
    forward. Then
        alpha = atan((a*cos(theta) - b) / (a*sin(theta)))   heading vs. wall
        d_t   = b * cos(alpha)                              distance now
        d_t1  = d_t + L * sin(alpha)                        distance L m ahead
    One wall:  error = d_t1 - desired_dist, steer = side * PID(error).
    Centre:    error = (d_t1_left - d_t1_right) / 2, steer = PID(error).
    Positive error means too far from the wall (or right of centre), so the
    car steers toward it. Steering +1 is full left lock (about 0.52 rad).
    The lookahead L grows with speed (lookahead + lookahead_per_speed * v).

Speed (v2, speed_mode:=planner, default)
    The whole scan becomes points around the car. For a fan of constant-
    curvature arcs the node predicts how far the car can drive before its
    footprint meets a point. The target speed is the lowest of a cap, a
    lateral-acceleration limit for the current steering, the speed that can
    still stop within the most open arc, and the speed that keeps ttc_plan
    seconds of that room. Throttle is target speed / 24.8 (measured).
    A restraint layer cuts throttle only when no reachable arc has room for
    ttc_brake seconds, and counts every time it acts (restraint_count topic).
    speed_mode:=throttle keeps the v1 rule (throttle scaled by steering and by
    distance to a wall ahead).

Topics
    sub  /autodrive/roboracer_1/lidar             sensor_msgs/LaserScan
    sub  /autodrive/roboracer_1/odom              nav_msgs/Odometry (speed only)
    pub  /autodrive/roboracer_1/throttle_command  std_msgs/Float32
    pub  /autodrive/roboracer_1/steering_command  std_msgs/Float32
    pub  /wall_follow/restraint_count             std_msgs/Int32
    sub  lap_count, last_lap_time, best_lap_time, collision_count (lap report)

All tunables are ROS 2 parameters, read on every scan, so they can be changed
at launch or live:
    ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=5.0
    ros2 param set /wall_follow_node v_max 5.0
"""

import math
import signal
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

SIDES = {'left': 1.0, 'right': -1.0}
TOPIC = '/autodrive/roboracer_1/'

# Vehicle geometry. The bridge places the front wheels 0.33 m and the LiDAR
# 0.2733 m ahead of the roboracer_1 frame (rear axle); a measured turn at
# 2.9 m/s gave an effective wheelbase of 0.34 m. Steering 1.0 is 0.524 rad.
WHEELBASE = 0.33
LIDAR_X = 0.2733
MAX_STEER_RAD = 0.5236


# ---------------------------------------------------------------------------
# Pure functions (no ROS), so the control law can be unit tested offline.
# ---------------------------------------------------------------------------
def clean_ranges(raw, range_min, range_max):
    """Replace inf, nan and out-of-range readings with range_max."""
    r = np.asarray(raw, dtype=float).copy()
    bad = ~np.isfinite(r) | (r < range_min) | (r > range_max)
    r[bad] = range_max
    return r


def beam_range(ranges, angle_min, angle_inc, angle_deg, window=3):
    """Median of the beams within +-window of angle_deg (0 = ahead, + = left)."""
    idx = int(round((math.radians(angle_deg) - angle_min) / angle_inc))
    lo = max(0, idx - window)
    hi = min(len(ranges) - 1, idx + window)
    return float(np.median(ranges[lo:hi + 1]))


def wall_distance(ranges, angle_min, angle_inc, side, theta_deg, lookahead):
    """Return (d_t, d_t1, alpha) for the wall on `side` (+1 left, -1 right).

    alpha is negative when the car is turning toward the wall, which makes
    the projected distance d_t1 smaller than the current distance d_t.
    """
    theta = math.radians(theta_deg)
    b = beam_range(ranges, angle_min, angle_inc, side * 90.0)
    a = beam_range(ranges, angle_min, angle_inc, side * (90.0 - theta_deg))
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    d_t = b * math.cos(alpha)
    d_t1 = d_t + lookahead * math.sin(alpha)
    return d_t, d_t1, alpha


def throttle_for(steer, front, base, minimum, brake_dist, stop_dist):
    """Slow down in turns and when a wall is close ahead, never below minimum."""
    throttle = base - (base - minimum) * min(1.0, abs(steer))
    if front < brake_dist:
        k = (front - stop_dist) / max(1e-6, brake_dist - stop_dist)
        k = max(0.0, min(1.0, k))
        throttle = minimum + (throttle - minimum) * k
    return throttle


class PID:
    """PID on the wall-distance error, with a clamped integral (anti-windup)."""

    def __init__(self, i_limit=2.0):
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error = None

    def step(self, error, dt, kp, ki, kd):
        deriv = 0.0
        if dt is not None and dt > 0.0:
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
            if self.prev_error is not None:
                deriv = (error - self.prev_error) / dt
        self.prev_error = error
        return kp * error + ki * self.integral + kd * deriv


def control_step(ranges, angle_min, angle_inc, prm, pid, dt):
    """One controller update. Returns (steer, throttle, info dict).

    prm holds the node parameters (side, desired_dist, lookahead, theta_deg,
    kp, ki, kd, max_steer, base_throttle, min_throttle, front_brake_dist,
    front_stop_dist). The ROS node and the unit tests both call this.
    """
    _, d_left, a_left = wall_distance(ranges, angle_min, angle_inc, 1.0,
                                      prm['theta_deg'], prm['lookahead'])
    _, d_right, a_right = wall_distance(ranges, angle_min, angle_inc, -1.0,
                                        prm['theta_deg'], prm['lookahead'])
    front = beam_range(ranges, angle_min, angle_inc, 0.0, window=10)
    if prm['side'] == 'center':
        # positive error = right of centre, so steer left (+)
        sign, error, d_t1, alpha = 1.0, (d_left - d_right) / 2.0, d_left, a_left
    else:
        sign = SIDES[prm['side']]
        d_t1, alpha = (d_left, a_left) if sign > 0 else (d_right, a_right)
        # positive error = too far from the wall, so steer toward it
        error = d_t1 - prm['desired_dist']
    u = pid.step(error, dt, prm['kp'], prm['ki'], prm['kd'])
    steer = max(-prm['max_steer'], min(prm['max_steer'], sign * u))
    throttle = throttle_for(steer, front, prm['base_throttle'], prm['min_throttle'],
                            prm['front_brake_dist'], prm['front_stop_dist'])
    return steer, throttle, {'d_t1': d_t1, 'alpha': alpha, 'error': error, 'front': front,
                             'd_left': d_left, 'd_right': d_right}


# ---------------------------------------------------------------------------
# v2: predict along the car's path with the whole scan, then plan speed.
# ---------------------------------------------------------------------------
def scan_points(ranges, angle_min, angle_inc, range_max):
    """LiDAR hits as points in the rear-axle frame (x ahead, y left).

    Beams at max range (no return) are dropped."""
    ang = angle_min + angle_inc * np.arange(len(ranges))
    ok = ranges < range_max - 1e-3
    return LIDAR_X + ranges[ok] * np.cos(ang[ok]), ranges[ok] * np.sin(ang[ok])


def arc_free_distance(px, py, curvature, half_width, front_overhang, max_s):
    """How far the rear axle can travel on a constant-curvature arc before the
    car's front reaches a LiDAR point lying within half_width of that arc.

    Unlike per-beam time to collision, this follows the turn the car is
    actually making, so a wall it is steering away from does not count.
    """
    if abs(curvature) < 1e-4:
        s, lateral = px, np.abs(py)
    else:
        r = 1.0 / curvature                       # turn centre at (0, r)
        lateral = np.abs(np.hypot(px, py - r) - abs(r))
        s = abs(r) * np.arctan2(px, math.copysign(1.0, r) * (r - py))
    hit = (lateral < half_width) & (s > -0.15)    # -0.15 keeps points beside the body
    if not hit.any():
        return max_s
    return float(min(max_s, max(0.0, float(s[hit].min()) - front_overhang)))


def steer_to_curvature(steer):
    return math.tan(steer * MAX_STEER_RAD) / WHEELBASE


def ittc_min(ranges, angle_min, angle_inc, v, fov_deg=60.0):
    """Course Lab 2 instantaneous time to collision, r / max(-rdot, 0), with
    rdot = -v cos(theta), minimised over beams within +-fov_deg of ahead."""
    ang = angle_min + angle_inc * np.arange(len(ranges))
    closing = v * np.cos(ang)
    m = (np.abs(ang) < math.radians(fov_deg)) & (closing > 1e-3)
    if not m.any():
        return math.inf
    return float(np.min(ranges[m] / closing[m]))


def plan_speed(steer, free_best, free_now, prm):
    """Target speed (m/s), the lowest of four limits.

    v_max     the cap.
    v_curve   lateral acceleration a_lat at the current steering curvature.
    v_free    can still stop within the free distance on the most open arc ahead.
    v_react   keeps at least ttc_plan seconds of escape room: the free distance
              on the best arc the steering can still reach (free_now is passed
              as that escape distance). ttc_plan is set above the restraint
              threshold ttc_brake, so the plan slows the car first and the
              restraint layer only acts when the plan has failed.
    Returns (v_target, v_curve, v_free, v_react).
    """
    kappa = abs(steer_to_curvature(steer))
    v_curve = math.sqrt(prm['a_lat'] / kappa) if kappa > 1e-3 else prm['v_max']
    v_free = math.sqrt(2.0 * prm['a_brake'] * max(0.0, free_best - prm['stop_margin']))
    v_react = free_now / prm['ttc_plan']
    v_target = max(prm['v_min'], min(prm['v_max'], v_curve, v_free, v_react))
    return v_target, v_curve, v_free, v_react


def drive_command(ranges, angle_min, angle_inc, range_max, v, prm, pid, dt):
    """Full controller: steering from control_step, then speed.

    speed_mode 'throttle' keeps the v1 throttle rule. speed_mode 'planner'
    predicts free distance along a fan of arcs from the LiDAR points, plans a
    target speed, converts it to throttle, and lets a restraint layer cut
    throttle only when contact is unavoidable: even the best arc within
    +-steer_band of the current steering (what the steering can still reach in
    time) runs out of room within ttc_brake seconds.
    Returns (steer, throttle, info).
    """
    if prm['speed_mode'] != 'planner':
        return control_step(ranges, angle_min, angle_inc, prm, pid, dt)
    # look further ahead at higher speed (a fixed time horizon), so the
    # steering does not zig-zag on the straights
    steer, throttle, info = control_step(
        ranges, angle_min, angle_inc,
        dict(prm, lookahead=prm['lookahead'] + prm['lookahead_per_speed'] * max(0.0, v)),
        pid, dt)
    px, py = scan_points(ranges, angle_min, angle_inc, range_max)
    hw, oh, ms = prm['half_width'], prm['front_overhang'], prm['max_s']
    free_now = arc_free_distance(px, py, steer_to_curvature(steer), hw, oh, ms)
    fan = np.linspace(-1.0, 1.0, int(prm['n_arcs']))
    fan_free = np.array([arc_free_distance(px, py, steer_to_curvature(st), hw, oh, ms)
                         for st in fan])
    free_best = float(fan_free.max())
    reach = np.abs(fan - steer) <= prm['steer_band'] + 1e-9
    free_escape = max(free_now, float(fan_free[reach].max()) if reach.any() else 0.0)
    v_target, v_curve, v_free, v_react = plan_speed(steer, free_best, free_escape, prm)
    throttle = 0.0 if v > v_target + prm['v_hysteresis'] else v_target / prm['speed_per_throttle']
    ttc_arc = free_now / v if v > 0.05 else math.inf
    ttc_escape = free_escape / v if v > 0.05 else math.inf
    intervene = v > prm['v_min'] and ttc_escape < prm['ttc_brake']
    if intervene:
        throttle = 0.0
    info.update(free_now=free_now, free_best=free_best, v_target=v_target, v_curve=v_curve,
                v_free=v_free, v_react=v_react, ttc_arc=ttc_arc, free_escape=free_escape,
                ttc_escape=ttc_escape, intervene=intervene,
                ittc=ittc_min(ranges, angle_min, angle_inc, v))
    return steer, throttle, info


# ---------------------------------------------------------------------------
class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # ---- tunables (ROS 2 parameters) ---------------------------------
        self.declare_parameter('side', 'center')       # center, left or right
        self.declare_parameter('desired_dist', 0.65)   # m, one-wall modes only
        self.declare_parameter('lookahead', 0.6)       # m, projection distance L at standstill
        self.declare_parameter('theta_deg', 45.0)      # deg between beams a and b
        # kp 2.0 suits centre mode (its error is half the left-right difference);
        # in a narrow toy track lower gains ran wide and higher ones cut corners.
        # For one-wall modes start near kp 1.0.
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('max_steer', 1.0)
        # v1 throttle rule (speed_mode:=throttle). Speed is about 24.8 m/s per unit
        # throttle, so 0.05 is about 1.2 m/s (run 1: 3 clean laps, 25.9 s).
        self.declare_parameter('base_throttle', 0.05)    # throttle on a straight
        self.declare_parameter('min_throttle', 0.03)     # throttle at full lock / wall ahead
        self.declare_parameter('front_brake_dist', 1.5)  # m, start slowing for a wall ahead
        self.declare_parameter('front_stop_dist', 0.6)   # m, down to min_throttle here
        self.declare_parameter('log_period', 1.0)        # s between status lines, 0 = off
        # ---- v2 speed planner (speed_mode:=planner) ------------------------
        # Defaults below are the setting validated on the practice track
        # (8 consecutive clean laps, 8.14 to 9.14 s, no restraint interventions).
        self.declare_parameter('speed_mode', 'planner')    # planner (v2) or throttle (v1)
        self.declare_parameter('speed_per_throttle', 24.8)  # m/s per unit throttle, measured
        self.declare_parameter('v_max', 6.0)        # m/s
        self.declare_parameter('v_min', 0.6)        # m/s, crawl speed so the car never stalls
        self.declare_parameter('a_lat', 6.0)        # m/s^2, lateral acceleration limit
        self.declare_parameter('a_brake', 3.0)      # m/s^2, measured coast decel is 3.0
        self.declare_parameter('stop_margin', 0.5)  # m kept free when planning to stop
        self.declare_parameter('v_hysteresis', 0.2)  # m/s above target before throttle 0
        self.declare_parameter('half_width', 0.22)  # m, car half width 0.15 plus margin
        self.declare_parameter('front_overhang', 0.45)  # m, rear axle to front bumper
        self.declare_parameter('max_s', 8.0)        # m, longest arc checked
        self.declare_parameter('n_arcs', 21)        # arcs in the fan, steering -1..1
        # Steering reachable before the restraint threshold. The servo reached a
        # commanded angle within one 0.2 s sample in the probe, so within 0.5 s
        # every arc is reachable: 2.0 spans the whole fan (escape = best arc).
        self.declare_parameter('steer_band', 2.0)
        self.declare_parameter('ttc_brake', 0.5)    # s, restraint layer threshold
        self.declare_parameter('ttc_plan', 0.6)     # s of escape room the planner keeps
        self.declare_parameter('lookahead_per_speed', 0.25)  # s, lookahead += this * speed

        side_name = str(self.p('side')).lower()
        if side_name not in SIDES and side_name != 'center':
            raise ValueError(f"side must be 'center', 'left' or 'right', got {side_name!r}")
        self.pid = PID()
        self.prev_time = None

        # ---- lap report state -------------------------------------------
        self.lap_count = None
        self.collisions = None
        self.collisions_at_lap_start = None
        self.best_lap = 0.0
        self.last_reported_lap_time = None
        self.clean_streak = 0

        # ---- ROS 2 plumbing ---------------------------------------------
        qos_cmd = QoSProfile(depth=1)
        qos_scan = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_throttle = self.create_publisher(Float32, TOPIC + 'throttle_command', qos_cmd)
        self.pub_steering = self.create_publisher(Float32, TOPIC + 'steering_command', qos_cmd)
        self.create_subscription(LaserScan, TOPIC + 'lidar', self.scan_callback, qos_scan)
        self.speed = 0.0
        self.create_subscription(Odometry, TOPIC + 'odom', self.on_odom, qos_scan)
        self.interventions = 0
        self.pub_restraints = self.create_publisher(Int32, '/wall_follow/restraint_count', 10)
        self.interventions_at_lap_start = 0
        self.intervening = False

        self.create_subscription(Int32, TOPIC + 'lap_count', self.on_lap_count, 10)
        self.create_subscription(Int32, TOPIC + 'collision_count', self.on_collision_count, 10)
        self.create_subscription(Float32, TOPIC + 'best_lap_time', self.on_best_lap_time, 10)
        self.create_subscription(Float32, TOPIC + 'last_lap_time', self.on_last_lap_time, 10)

        what = 'centred between both walls' if side_name == 'center' else \
            f'following the {side_name.upper()} wall'
        self.get_logger().info(f'wall_follow_node started, {what}')

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    def params(self):
        prm = {name: float(self.p(name)) for name in (
            'desired_dist', 'lookahead', 'theta_deg', 'kp', 'ki', 'kd', 'max_steer',
            'base_throttle', 'min_throttle', 'front_brake_dist', 'front_stop_dist')}
        for name in ('speed_per_throttle', 'v_max', 'v_min', 'a_lat', 'a_brake', 'stop_margin',
                     'v_hysteresis', 'half_width', 'front_overhang', 'max_s', 'n_arcs',
                     'ttc_brake', 'ttc_plan', 'lookahead_per_speed', 'steer_band'):
            prm[name] = float(self.p(name))
        prm['side'] = str(self.p('side')).lower()
        prm['speed_mode'] = str(self.p('speed_mode')).lower()
        return prm

    def scan_callback(self, scan: LaserScan):
        ranges = clean_ranges(scan.ranges, scan.range_min, scan.range_max)
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = None if self.prev_time is None else max(1e-3, now - self.prev_time)
        self.prev_time = now

        steer, throttle, info = drive_command(ranges, scan.angle_min, scan.angle_increment,
                                              scan.range_max, self.speed, self.params(),
                                              self.pid, dt)
        if info.get('intervene') and not self.intervening:
            self.interventions += 1
            self.pub_restraints.publish(Int32(data=self.interventions))
            self.get_logger().warn(
                f"RESTRAINT: no escape arc, contact in {info['ttc_escape']:.2f} s "
                f"(escape room {info['free_escape']:.2f} m at {self.speed:.2f} m/s; current arc "
                f"{info['ttc_arc']:.2f} s, iTTC {info['ittc']:.2f} s), throttle cut")
        self.intervening = bool(info.get('intervene'))
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steer)))

        period = float(self.p('log_period'))
        if period > 0.0:
            left = beam_range(ranges, scan.angle_min, scan.angle_increment, 90.0)
            right = beam_range(ranges, scan.angle_min, scan.angle_increment, -90.0)
            self.get_logger().info(
                f"L={left:.2f} R={right:.2f} F={info['front']:.2f} "
                f"dL={info['d_left']:.2f} dR={info['d_right']:.2f} "
                f"alpha={math.degrees(info['alpha']):+.1f} err={info['error']:+.2f} "
                f'steer={steer:+.2f} thr={throttle:.3f} v={self.speed:.2f}'
                + (f" v_tgt={info['v_target']:.2f} free={info['free_now']:.2f}/"
                   f"{info['free_best']:.2f} ttc_arc={info['ttc_arc']:.2f} "
                   f"ttc_esc={info['ttc_escape']:.2f} "
                   f"ittc={info['ittc']:.2f}" if 'v_target' in info else ''),
                throttle_duration_sec=period)

    def on_odom(self, msg: Odometry):
        self.speed = float(msg.twist.twist.linear.x)

    def stop_car(self, repeats=10):
        """Send zero throttle and steering a few times so the bridge surely gets it."""
        for _ in range(repeats):
            self.pub_throttle.publish(Float32(data=0.0))
            self.pub_steering.publish(Float32(data=0.0))
            time.sleep(0.02)
        self.get_logger().info('stopped the car (throttle 0, steering 0)')

    # ---- lap report (tuning aid only, does not affect driving) -----------
    def on_lap_count(self, msg: Int32):
        if self.lap_count is not None and msg.data < self.lap_count:
            self.get_logger().info('reset detected, clean-lap streak back to 0')
            self.clean_streak = 0
            self.collisions_at_lap_start = self.collisions
        self.lap_count = msg.data

    def on_collision_count(self, msg: Int32):
        if self.collisions is not None and msg.data > self.collisions:
            self.get_logger().warn(f'WALL CONTACT (total {msg.data}), this lap will not count')
        if self.collisions is None or msg.data < self.collisions:
            self.collisions_at_lap_start = msg.data
        self.collisions = msg.data

    def on_best_lap_time(self, msg: Float32):
        self.best_lap = float(msg.data)

    def on_last_lap_time(self, msg: Float32):
        # The bridge publishes lap_count before last_lap_time, so a lap is
        # reported when the lap time value itself changes.
        t = float(msg.data)
        if self.last_reported_lap_time is None:
            self.last_reported_lap_time = t
            return
        if t <= 0.0 or t == self.last_reported_lap_time:
            self.last_reported_lap_time = t
            return
        self.last_reported_lap_time = t
        hits = 0
        if self.collisions is not None and self.collisions_at_lap_start is not None:
            hits = self.collisions - self.collisions_at_lap_start
        self.collisions_at_lap_start = self.collisions
        restraints = self.interventions - self.interventions_at_lap_start
        self.interventions_at_lap_start = self.interventions
        self.clean_streak = self.clean_streak + 1 if hits == 0 else 0
        verdict = 'CLEAN' if hits == 0 else f'{hits} wall contact(s)'
        self.get_logger().info(
            f'LAP {self.lap_count}: {t:.3f} s, {verdict}, best {self.best_lap:.3f} s, '
            f'clean streak {self.clean_streak}, restraint interventions {restraints}')


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle Ctrl-C (and docker stop / kill) ourselves. With rclpy's default
    # handler the context is shut down first, the final stop command is lost,
    # and the bridge keeps driving with the last throttle and steering it got.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
