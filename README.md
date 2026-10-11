# Penn RoboRacer technical challenge, wall following

Rubi Wu, Penn M&T '30

A ROS 2 node that wall-follows in the AutoDRIVE RoboRacer practice simulator (IROS 2026 release), using only the 2D LiDAR.

## Status

Work in progress. Code for the first controller is here. Lap video and fastest lap time will be added once the three clean laps are in.

- Video (unlisted): coming
- Fastest lap: coming

## What is in the repo

`autodrive_devkit/` is the ROS 2 package from the AutoDRIVE bridge, plus one new file.

- `autodrive_roboracer/wall_follow_node.py` is the controller.
- `setup.py` registers it as `wall_follow_node`.

The bridge, teleop and launch files are unchanged from upstream.

## How the controller works

The node follows the left wall. It takes two LiDAR beams, one at 90 degrees (perpendicular to the wall) and one at 45 degrees (forward-left), and from them computes the angle between the car and the wall and the car's distance to it. It projects that distance a short lookahead ahead of the car, compares it to the target distance, and feeds the error into a PID controller that outputs steering. Throttle is highest on straights and drops as the steering angle grows. A separate check on the beam straight ahead brakes the car if a wall is close in front.

All gains are ROS 2 parameters, so tuning happens on the command line without editing code.

## Run

Build and start the bridge inside the devkit container, then in a second shell

```
ros2 run autodrive_roboracer wall_follow_node --ros-args -p kp:=1.0 -p base_throttle:=0.3
```

## Tuning notes

To be filled in as runs happen (parameter, result, lap time).
