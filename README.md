# Penn RoboRacer technical challenge

Rubi Wu, Penn M&T '30

A ROS 2 wall follower for the AutoDRIVE RoboRacer practice track (IROS 2026 release). The car steers by staying centred between the two walls. It sets its speed from a LiDAR prediction of how much free space lies along the arcs it can still drive.

| Result | Value |
|---|---|
| Fastest clean lap | **8.142 s** |
| Longest run of clean laps | 45 in a row |
| Wall contacts with the default settings | 0 in every validation run |
| Video (Driver's Eye, unlisted YouTube) | link will be added here |

Lap times are the simulator's own lap clock, the same number the HUD shows.

![Speed on three clean laps with the default settings](docs/speed_map.png)

## Video

The unlisted YouTube link of the car driving on its own in Driver's Eye view will go here once the recording is uploaded.

## How to run

Use the setup from the challenge sheet (Docker image, bridge, practice simulator). Then, in a second terminal inside the container,

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

Switch Driving Mode to Autonomous. The defaults are the validated setting. Every tunable is a ROS 2 parameter that the node reads on every scan, so it can be set at launch or changed while the car is driving.

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=4.0
ros2 param set /wall_follow_node v_max 4.0
```

Two presets are useful for comparison.

```bash
# v1, slow and steady: fixed low throttle, about 1.2 m/s, 25.9 s laps
ros2 run autodrive_roboracer wall_follow_node --ros-args -p speed_mode:=throttle
# a more conservative v2: about 9 s laps
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=5.0 -p a_lat:=5.0 -p a_brake:=2.5 -p ttc_plan:=0.8
```

Stop the node with Ctrl-C. It sends throttle 0 and steering 0 before it exits, because the bridge keeps applying the last command it received.

## How it works

### Steering

This is the wall-following controller from the RoboRacer course (Lab 3). For each wall, beam b points straight at it (90 degrees) and beam a points theta degrees further forward. The node estimates the car's angle to the wall and projects the distance a lookahead L ahead.

```
alpha = atan((a*cos(theta) - b) / (a*sin(theta)))
d_t   = b * cos(alpha)
d_t1  = d_t + L * sin(alpha)
```

The practice track is only about 1.3 m wide at the start line, so the default mode follows both walls and steers to the centre, with `error = (d_t1_left - d_t1_right) / 2` and `steer = PID(error)`. Following one wall at a fixed distance is still available with `side:=left` or `side:=right`. The lookahead grows with speed (`L = 0.6 + 0.25 * v`), which stopped the steering from zig-zagging on the straights.

### Speed from a LiDAR prediction

Every scan becomes a set of points around the car. For a fan of 21 constant-curvature arcs, from full left lock to full right lock, the node computes how far the rear axle can travel before the car's footprint (half width 0.22 m, front overhang 0.45 m) reaches one of those points. The target speed is the lowest of four limits.

| Limit | Meaning |
|---|---|
| `v_max` | a cap, 6 m/s |
| `v_curve` | lateral acceleration `a_lat` at the current steering curvature |
| `v_free` | the speed from which the car can still stop within the free distance on the most open arc |
| `v_react` | keeps `ttc_plan` seconds of that free distance ahead |

Throttle is the target speed divided by 24.8, a gain measured in the simulator. If the car is more than 0.2 m/s above the target, throttle goes to 0, which brakes at about 3 m/s².

### Restraint layer

A separate check cuts throttle only when contact can no longer be avoided, meaning even the best arc has less than `ttc_brake` (0.5 s) of room at the current speed. The planner keeps a larger margin (`ttc_plan`, 0.6 s), so in normal driving it slows the car first and the restraint layer stays silent. The node counts every intervention, prints it in the lap report and publishes it on `/wall_follow/restraint_count`. With the default settings it did not fire once in the validation runs.

## Measured before tuning

A short scripted probe checked the conventions on the sheet and measured the car before any controller ran.

| Quantity | Measured |
|---|---|
| Steering command 1.0 | 0.524 rad, positive turns left |
| LiDAR | 1081 beams, +90 degrees is the car's left |
| Speed per unit throttle | about 24.8 m/s (0.05 gives 1.24 m/s, 0.12 gives 2.97 m/s) |
| Deceleration at throttle 0 | 3.0 m/s² |
| Deceleration at throttle -0.12 | 2.9 m/s², no stronger than coasting |
| Effective wheelbase | 0.34 m, from yaw rate in a measured turn |
| Track | about 29.4 m per lap, clockwise, 1.15 to 2.68 m wide |
| Simulator speed on the test Mac | 0.67 times real time |

The throttle gain mattered most. The first draft's default throttle of 0.30 would have meant about 7.4 m/s on a 1.3 m wide track.

## Results

| Version | Best lap | Notes |
|---|---|---|
| v1, centred PID, fixed low throttle | 25.89 s | first three laps all clean |
| v2 planner, top speed 2 m/s | 15.03 s | clean |
| v2 planner, top speed 4 m/s | 10.04 s | clean |
| v2 planner, top speed up to 7 m/s | 9.13 s | at 7 m/s it touched the wall twice, and the restraint layer fired 41 to 59 times per lap |
| v2.1 escape-based planner, default settings | 8.142 s | 45 clean laps in a row, restraint layer silent |

Settings were swept live with a script that changes parameters on the running node, skips one transition lap and then records lap times, contacts and restraint interventions. Above about 5 m/s the cap stops mattering because the corners limit the lap. Lap-to-lap scatter at the end was about 0.4 s, larger than the differences between the last few settings, so the defaults favour the setting that stayed clean the longest over the single fastest lap.

## Three ways to predict a collision

The RoboRacer course's Lab 2 uses instantaneous time to collision, `iTTC = r / max(-r_dot, 0)` with `r_dot = -v cos(theta)`, and asks for a brake that does not trigger falsely. On this track that is hard. At 2 m/s on the start straight, centred with 0.62 m to each wall, the minimum iTTC over the front 120 degrees was 0.61 s, because beams angled toward the side walls see them as approaching. A threshold above 0.6 s would brake the whole straight. Time to contact along the arc the car was actually driving was 3.7 s at the same moment.

The arc the car is on right now has its own problem at speed. The centring controller turns in sharply at corner entry, and that arc briefly points at a wall even though the car is about to steer away. A restraint layer based on it fired 41 to 59 times per lap at 7 m/s, and the car still touched the wall twice. Asking instead whether any reachable arc still has room (the steering servo reached its commanded angle within 0.2 s in the probe, so the whole fan is reachable within the 0.5 s threshold) removed those false interventions. The same quantity drives the planner, so the planner and the safety check agree about what is dangerous.

## Problems found along the way

1. The first draft of the controller had the error sign reversed and steered into the wall it was following. A numerical check with a synthetic scan caught it before the simulator was used, and a unit test now guards it.
2. With rclpy's default signal handling, Ctrl-C shut ROS down before the node's final stop command, which then failed with "publisher's context is invalid". The bridge kept the last throttle and the car drove into a wall. The node now handles SIGINT and SIGTERM itself and sends the stop command before shutting down.
3. Closing the terminal that runs `docker exec` does not stop the node inside the container. Stop it with Ctrl-C.

## Tests

`autodrive_devkit/test/test_wall_follow.py` has 21 offline tests that need no simulator. They cover beam indexing, the wall geometry, the steering sign for both sides and centre mode, the throttle floor, the arc prediction, iTTC and the speed planner. Four closed-loop tests drive a kinematic bicycle model around synthetic tracks, two of them as narrow as the practice track. The planner test uses the speed dynamics measured in the simulator. From the cloned challenge repo,

```bash
docker run --platform linux/amd64 --rm -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$(pwd)/autodrive_devkit":/home/autodrive_devkit/src/autodrive_devkit --entrypoint /bin/bash \
  penn-challenge -c 'source /opt/ros/humble/setup.bash && cd src/autodrive_devkit && python3 -m pytest -p no:cacheprovider test/test_wall_follow.py -q'
```

The toy car model is an approximation, so these tests check signs, geometry and stability. Lap times come only from the simulator.

## Files

`autodrive_devkit` is the package from the AutoDRIVE RoboRacer Sim Racing repository (BSD 2-Clause, see `LICENSE-AutoDRIVE`). Three files were added or changed, and `autodrive_bridge.py` is untouched.

| File | Change |
|---|---|
| `autodrive_roboracer/wall_follow_node.py` | new, the controller |
| `setup.py` | one console_scripts entry for `wall_follow_node` |
| `test/test_wall_follow.py` | new, offline tests |

Built with Claude (Anthropic) as a coding assistant, which the challenge allows.
