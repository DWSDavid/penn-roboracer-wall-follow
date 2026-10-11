"""Offline tests for wall_follow_node: no simulator, no running ROS graph.

Synthetic LiDAR scans are ray-cast against straight wall segments using the
same geometry as the AutoDRIVE bridge (1081 beams, -135 to +135 deg, 0.25 deg
step, 10 m max). The closed-loop tests drive a kinematic bicycle model with
the exact control_step() the node runs. The car model is an assumption (speed
proportional to throttle, no tire slip), so these tests check that the control
law has the right signs and is stable, not what the real lap time will be.
"""

import math

import numpy as np

from autodrive_roboracer.wall_follow_node import (
    LIDAR_X, PID, arc_free_distance, beam_range, clean_ranges, control_step, drive_command,
    ittc_min, plan_speed, scan_points, steer_to_curvature, throttle_for, wall_distance)

N_BEAMS = 1081          # the live bridge sends 1081 (both ends of 270 deg)
ANGLE_MIN = -2.35619
ANGLE_INC = 0.004363323
RANGE_MAX = 10.0
WHEELBASE = 0.33        # m, from the wheel transforms in autodrive_bridge.py
MAX_STEER_RAD = 0.52    # steering command 1.0, from the challenge sheet
CAR_RADIUS = 0.20       # m, counts as wall contact if a wall is closer than this

DEFAULTS = {
    'side': 'left', 'desired_dist': 0.9, 'lookahead': 0.6, 'theta_deg': 45.0,
    'kp': 1.0, 'ki': 0.0, 'kd': 0.08, 'max_steer': 1.0,
    'base_throttle': 0.30, 'min_throttle': 0.12,
    'front_brake_dist': 1.2, 'front_stop_dist': 0.45,
    'speed_mode': 'throttle', 'speed_per_throttle': 24.8, 'v_max': 2.0, 'v_min': 0.6,
    'a_lat': 3.0, 'a_brake': 2.5, 'stop_margin': 0.5, 'v_hysteresis': 0.2,
    'half_width': 0.22, 'front_overhang': 0.45, 'max_s': 8.0, 'n_arcs': 21, 'ttc_brake': 0.5,
    'ttc_plan': 0.8, 'lookahead_per_speed': 0.0, 'steer_band': 2.0,
}
PLANNER = dict(DEFAULTS, side='center', kp=2.0, lookahead=1.0, speed_mode='planner')


def scan(x, y, yaw, segments):
    """Ray-cast all beams from pose (x, y, yaw) against wall segments."""
    ang = yaw + ANGLE_MIN + ANGLE_INC * np.arange(N_BEAMS)
    dx, dy = np.cos(ang), np.sin(ang)
    out = np.full(N_BEAMS, RANGE_MAX)
    for (ax, ay), (bx, by) in segments:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((ax - x) * ey - (ay - y) * ex) / den
            s = ((ax - x) * dy - (ay - y) * dx) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (s >= 0) & (s <= 1)
        out = np.where(hit & (t < out), t, out)
    return out


def dist_to_segment(px, py, seg):
    (ax, ay), (bx, by) = seg
    ex, ey = bx - ax, by - ay
    s = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (ax + s * ex), py - (ay + s * ey))


def drive(segments, x, y, yaw, prm, seconds, speed_per_throttle=7.0, rate=40.0):
    """Closed loop: scan -> controller -> bicycle model. Returns the path.

    v1 (speed_mode throttle) keeps the original toy model, speed = 7 x throttle.
    v2 (planner) uses the dynamics measured in the simulator: steady speed
    24.8 x throttle, reached with a 0.3 s lag, at most +2.6 m/s^2 and -3.0 m/s^2.
    """
    pid, dt = PID(), 1.0 / rate
    path, v = [], 0.0
    planner = prm.get('speed_mode') == 'planner'
    for k in range(int(seconds * rate)):
        ranges = clean_ranges(scan(x, y, yaw, segments), 0.06, RANGE_MAX)
        steer, thr, info = drive_command(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MAX, v, prm, pid,
                                         None if k == 0 else dt)
        if planner:
            v += max(-3.0, min(2.6, (prm['speed_per_throttle'] * thr - v) / 0.3)) * dt
        else:
            v = speed_per_throttle * thr
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v / WHEELBASE * math.tan(steer * MAX_STEER_RAD) * dt
        clearance = min(dist_to_segment(x, y, s) for s in segments)
        path.append((x, y, yaw, steer, thr, clearance, v, bool(info.get('intervene'))))
    return path


# ---- geometry ---------------------------------------------------------------
def test_beam_index_matches_challenge_sheet():
    # 0 deg is beam 540, +90 deg (left) is beam 900, -90 deg (right) is beam 180
    r = np.arange(N_BEAMS, dtype=float)
    assert beam_range(r, ANGLE_MIN, ANGLE_INC, 0.0, window=0) == 540
    assert beam_range(r, ANGLE_MIN, ANGLE_INC, 90.0, window=0) == 900
    assert beam_range(r, ANGLE_MIN, ANGLE_INC, -90.0, window=0) == 180


def test_clean_ranges_replaces_bad_values():
    r = clean_ranges([float('inf'), float('nan'), 0.01, 3.0, 12.0], 0.06, 10.0)
    assert list(r) == [10.0, 10.0, 10.0, 3.0, 10.0]


def test_parallel_wall_distance_is_exact():
    wall = [((-20.0, 0.7), (20.0, 0.7))]               # left wall 0.7 m away
    d_t, d_t1, alpha = wall_distance(scan(0, 0, 0, wall), ANGLE_MIN, ANGLE_INC, 1.0, 45.0, 0.6)
    assert abs(d_t - 0.7) < 0.01 and abs(d_t1 - 0.7) < 0.01 and abs(alpha) < 0.01


def test_heading_toward_wall_shrinks_projected_distance():
    wall = [((-20.0, 1.0), (20.0, 1.0))]
    yaw = math.radians(10)                               # nose turned 10 deg toward it
    d_t, d_t1, alpha = wall_distance(scan(0, 0, yaw, wall), ANGLE_MIN, ANGLE_INC,
                                     1.0, 45.0, 0.6)
    assert abs(math.degrees(alpha) + 10) < 0.5
    assert abs(d_t - 1.0) < 0.02
    assert d_t1 < d_t


# ---- steering sign (the v1 bug) --------------------------------------------
def first_steer(wall_y, side):
    wall = [((-20.0, wall_y), (20.0, wall_y))]
    prm = dict(DEFAULTS, side=side)
    steer, _, _ = control_step(clean_ranges(scan(0, 0, 0, wall), 0.06, RANGE_MAX),
                               ANGLE_MIN, ANGLE_INC, prm, PID(), None)
    return steer


def test_too_close_to_left_wall_steers_right():
    assert first_steer(0.4, 'left') < 0


def test_too_far_from_left_wall_steers_left():
    assert first_steer(1.6, 'left') > 0


def test_right_wall_is_mirror_image():
    assert first_steer(-0.4, 'right') > 0     # too close on the right, steer left
    assert first_steer(-1.6, 'right') < 0
    # beam angles in the bridge are rounded, so +90 and -90 deg differ by ~1e-6 rad
    assert abs(first_steer(-0.4, 'right') + first_steer(0.4, 'left')) < 1e-4


def test_center_mode_steers_back_to_the_middle():
    def steer_at(y):   # car at lateral offset y in a 1.3 m corridor (+y = left)
        walls = [((-20.0, 0.65 - y), (20.0, 0.65 - y)), ((-20.0, -0.65 - y), (20.0, -0.65 - y))]
        prm = dict(DEFAULTS, side='center')
        st, _, info = control_step(clean_ranges(scan(0, 0, 0, walls), 0.06, RANGE_MAX),
                                   ANGLE_MIN, ANGLE_INC, prm, PID(), None)
        return st, info
    st, info = steer_at(0.0)
    assert abs(st) < 1e-3 and abs(info['d_left'] - 0.65) < 0.01
    assert steer_at(-0.2)[0] > 0      # right of centre, steer left
    assert steer_at(+0.2)[0] < 0      # left of centre, steer right


# ---- throttle ---------------------------------------------------------------
def test_throttle_never_below_minimum():
    for steer in (0.0, 0.5, 1.0, -1.0):
        for front in (0.0, 0.3, 0.45, 0.8, 1.2, 5.0):
            thr = throttle_for(steer, front, 0.30, 0.12, 1.2, 0.45)
            assert 0.12 - 1e-9 <= thr <= 0.30 + 1e-9


def test_throttle_full_on_clear_straight():
    assert throttle_for(0.0, 8.0, 0.30, 0.12, 1.2, 0.45) == 0.30


# ---- closed loop ------------------------------------------------------------
def test_corridor_converges_to_desired_distance():
    corridor = [((-5.0, 1.2), (80.0, 1.2)), ((-5.0, -1.2), (80.0, -1.2))]
    path = drive(corridor, 0.0, -0.4, 0.0, DEFAULTS, seconds=8.0)  # starts 1.6 m from left
    x, y, yaw, *_ = path[-1]
    assert min(p[5] for p in path) > CAR_RADIUS, 'touched a wall'
    assert abs((1.2 - y) - DEFAULTS['desired_dist']) < 0.05
    assert abs(math.degrees(yaw)) < 3


def test_two_laps_of_a_rectangular_track_without_contact():
    # counter-clockwise loop, so the left wall is the inner wall
    outer = [(-1.0, -1.0), (13.0, -1.0), (13.0, 9.0), (-1.0, 9.0)]
    inner = [(1.0, 1.0), (11.0, 1.0), (11.0, 7.0), (1.0, 7.0)]
    walls = [(poly[i], poly[(i + 1) % 4]) for poly in (outer, inner) for i in range(4)]
    path = drive(walls, 6.0, 0.1, 0.0, DEFAULTS, seconds=60.0)
    turned = path[-1][2] - path[0][2]
    assert min(p[5] for p in path) > CAR_RADIUS, 'touched a wall'
    assert turned > 2 * (2 * math.pi) * 0.95, f'only turned {math.degrees(turned):.0f} deg'


def test_center_mode_laps_a_track_as_narrow_as_the_practice_track():
    # 1.3 m wide (measured at the start line), centre line is a 12 x 8 m rectangle
    o, i = 0.65, 0.65
    outer = [(-o, -o), (12 + o, -o), (12 + o, 8 + o), (-o, 8 + o)]
    inner = [(i, i), (12 - i, i), (12 - i, 8 - i), (i, 8 - i)]
    walls = [(poly[k], poly[(k + 1) % 4]) for poly in (outer, inner) for k in range(4)]
    # node defaults for centre mode; lower kp/lookahead ran wide, higher cut the corner
    prm = dict(DEFAULTS, side='center', kp=2.0, lookahead=1.0)
    path = drive(walls, 6.0, 0.0, 0.0, prm, seconds=60.0)
    turned = path[-1][2] - path[0][2]
    assert min(p[5] for p in path) > CAR_RADIUS, 'touched a wall'
    assert turned > 2 * (2 * math.pi) * 0.95, f'only turned {math.degrees(turned):.0f} deg'


# ---- v2 prediction and planner ---------------------------------------------
def pts(segments, x=0.0, y=0.0, yaw=0.0):
    r = clean_ranges(scan(x, y, yaw, segments), 0.06, RANGE_MAX)
    return scan_points(r, ANGLE_MIN, ANGLE_INC, RANGE_MAX)


def test_free_distance_to_a_wall_straight_ahead():
    px, py = pts([((3.0, -2.0), (3.0, 2.0))])          # wall 3 m ahead of the LiDAR
    free = arc_free_distance(px, py, 0.0, 0.22, 0.45, 8.0)
    assert abs(free - (3.0 + LIDAR_X - 0.45)) < 0.02


def test_free_distance_is_open_down_a_corridor():
    px, py = pts([((-5.0, 0.65), (30.0, 0.65)), ((-5.0, -0.65), (30.0, -0.65))])
    assert arc_free_distance(px, py, 0.0, 0.22, 0.45, 8.0) == 8.0


def test_turning_into_the_open_side_of_a_corner_gives_more_room():
    # corridor ends 3 m ahead, the way continues to the LEFT
    corner = [((-5.0, -0.65), (3.0, -0.65)), ((3.0, -0.65), (3.0, 5.0)),
              ((-5.0, 0.65), (1.7, 0.65)), ((1.7, 0.65), (1.7, 5.0))]
    px, py = pts(corner)
    straight = arc_free_distance(px, py, 0.0, 0.22, 0.45, 8.0)
    fan = np.linspace(0.02, 1.0, 50)
    best_left = max(arc_free_distance(px, py, steer_to_curvature(st), 0.22, 0.45, 8.0)
                    for st in fan)
    best_right = max(arc_free_distance(px, py, steer_to_curvature(-st), 0.22, 0.45, 8.0)
                     for st in fan)
    hard_left = arc_free_distance(px, py, steer_to_curvature(0.6), 0.22, 0.45, 8.0)
    # One constant arc cannot round a 1.3 m wide 90 deg corner from mid-lane (the car does it
    # by changing curvature), so the prediction is conservative here: slower, never unsafe.
    assert best_left > straight            # a gentle left arc gets further than straight
    assert best_right <= straight + 1e-6   # nothing to the right
    assert hard_left < straight            # turning in too early clips the inner corner


def test_free_distance_is_mirror_symmetric():
    walls = [((2.0, 0.3), (4.0, 1.5)), ((-5.0, -0.7), (10.0, -0.7))]
    mirrored = [((ax, -ay), (bx, -by)) for (ax, ay), (bx, by) in walls]
    for steer in (-0.5, 0.0, 0.3):
        a = arc_free_distance(*pts(walls), steer_to_curvature(steer), 0.22, 0.45, 8.0)
        b = arc_free_distance(*pts(mirrored), steer_to_curvature(-steer), 0.22, 0.45, 8.0)
        assert abs(a - b) < 0.03


def test_ittc_matches_lab2_definition():
    r = clean_ranges(scan(0, 0, 0, [((2.0, -3.0), (2.0, 3.0))]), 0.06, RANGE_MAX)
    assert abs(ittc_min(r, ANGLE_MIN, ANGLE_INC, 2.0) - 1.0) < 0.01   # 2 m at 2 m/s
    assert ittc_min(r, ANGLE_MIN, ANGLE_INC, 0.0) == math.inf


def test_plan_speed_limits():
    prm = PLANNER
    assert plan_speed(0.0, 8.0, 8.0, prm)[0] == prm['v_max']        # straight, open road
    assert plan_speed(0.0, 0.2, 8.0, prm)[0] == prm['v_min']        # no room: crawl
    tight, wide = plan_speed(1.0, 8.0, 8.0, prm)[0], plan_speed(0.3, 8.0, 8.0, prm)[0]
    assert tight < wide <= prm['v_max']
    # heading at a wall 1.2 m away on the current arc: keep ttc_plan seconds
    assert abs(plan_speed(0.0, 8.0, 1.2, prm)[0] - 1.2 / prm['ttc_plan']) < 1e-9


def test_throttle_mode_is_unchanged_by_v2():
    walls = [((-20.0, 0.5), (20.0, 0.5)), ((-20.0, -0.8), (20.0, -0.8))]
    r = clean_ranges(scan(0, 0, 0, walls), 0.06, RANGE_MAX)
    a = control_step(r, ANGLE_MIN, ANGLE_INC, DEFAULTS, PID(), None)[:2]
    b = drive_command(r, ANGLE_MIN, ANGLE_INC, RANGE_MAX, 1.0, DEFAULTS, PID(), None)[:2]
    assert a == b


def test_planner_laps_the_narrow_track_faster_without_contact():
    o, i = 0.65, 0.65
    outer = [(-o, -o), (12 + o, -o), (12 + o, 8 + o), (-o, 8 + o)]
    inner = [(i, i), (12 - i, i), (12 - i, 8 - i), (i, 8 - i)]
    walls = [(poly[k], poly[(k + 1) % 4]) for poly in (outer, inner) for k in range(4)]
    prm = dict(PLANNER, v_max=4.0, a_lat=5.0, lookahead=0.6, lookahead_per_speed=0.25)
    path = drive(walls, 6.0, 0.0, 0.0, prm, seconds=40.0)
    turned = path[-1][2] - path[0][2]
    onsets = sum(1 for a, b in zip(path, path[1:]) if b[7] and not a[7])
    assert min(p[5] for p in path) > CAR_RADIUS, 'touched a wall'
    assert max(p[6] for p in path) > 2.5, 'never reached speed on the straights'
    assert onsets <= 2, f'restraint layer fired {onsets} times'
    assert turned > 2 * (2 * math.pi) * 0.95, f'only turned {math.degrees(turned):.0f} deg'


if __name__ == '__main__':
    import sys
    import pytest
    sys.exit(pytest.main([__file__, '-q']))
