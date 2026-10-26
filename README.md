# Task 2 — Multi-Pedestrian LiDAR Simulation

Environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, simulated Clearpath Jackal.
Robot stays at its spawn location (x=0, y=0, z=0.3; yaw=0). Its stationary
behavior is evaluated from recorded `/j100_0000/platform/odom` in validation.csv.

## Scenarios
- S01: human_1; recordings: S01_run02, S01_run03
- S02: human_2; recordings: S02_run01, S02_run02
- S03: human_3; recordings: S03_run01, S03_run02
- S04: human_1 + human_2; recordings: S04_run01, S04_run02
- S05: human_1 + human_3; recordings: S05_run02, S05_run03
- S06: human_2 + human_3; recordings: S06_run01, S06_run02
- S07: human_1 + human_2 + human_3; recordings: S07_run01, S07_run02

Each run records >30 seconds of Gazebo simulation time, with five ROS topics:
`/clock`, `/j100_0000/sensors/lidar3d_0/points`,
`/j100_0000/platform/odom`, `/j100_0000/tf`, `/j100_0000/tf_static`.

## How to launch a scenario
```bash
source /opt/ros/humble/setup.bash
source ~/jackal_actor_ws/install/setup.bash
export IGN_IP=127.0.0.1
# Install worlds/*.sdf in $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/worlds/ if necessary.
ros2 launch clearpath_gz simulation.launch.py world:=task2_s07_h123 x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 rviz:=false
```

## Bag analysis
After sourcing ROS 2, use `ros2 bag info rosbags/S07_run02` or
`ros2 bag play rosbags/S07_run02 --clock`. Do not run playback at the same
time as a live simulation on the same ROS domain.

The Actor paths are pre-scripted SDF trajectories, NOT live collision avoidance.
The SDFs contain human mesh/animation Fuel URLs; external resource access or
local caching may be required to recreate visuals on another computer.

The robot stationary statistics are odometry-relative measurements, not
independent ground-truth motion-capture measurements.
