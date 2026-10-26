#!/usr/bin/env python3
"""Verify 14 Task-2 bags and stage a self-contained research handoff.
Run after the automatic second recording pass on Ubuntu ROS 2 Humble.
"""
import csv
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

HOME = Path.home()
BAG_DIR = HOME / "task2_rosbags"
WORLDS = HOME / "jackal_actor_sim" / "worlds"
OUTPUT = HOME / "task2_submission_final"
CASES = [
    ("S01", "human_1", "S01_run02", "S01_run03"),
    ("S02", "human_2", "S02_run01", "S02_run02"),
    ("S03", "human_3", "S03_run01", "S03_run02"),
    ("S04", "human_1 + human_2", "S04_run01", "S04_run02"),
    ("S05", "human_1 + human_3", "S05_run02", "S05_run03"),
    ("S06", "human_2 + human_3", "S06_run01", "S06_run02"),
    ("S07", "human_1 + human_2 + human_3", "S07_run01", "S07_run02"),
]
WORLD_FILES = ["task2_curve_light.sdf"] + [
    f"task2_s{i:02d}_{suffix}.sdf"
    for i, suffix in enumerate(("h1", "h2", "h3", "h12", "h13", "h23", "h123"), 1)
]
TOPICS = (
    "/clock", "/j100_0000/sensors/lidar3d_0/points",
    "/j100_0000/platform/odom", "/j100_0000/tf", "/j100_0000/tf_static",
)


def inspect_bag(folder):
    info = subprocess.check_output(["ros2", "bag", "info", str(folder)], text=True)
    match = re.search(r"Duration:\s+([\d.]+)s", info)
    if match is None:
        raise ValueError(f"Duration 정보를 찾지 못했습니다: {folder}")
    duration = float(match.group(1))
    counts = {}
    for topic in TOPICS:
        m = re.search(rf"Topic:\s*{re.escape(topic)}\s*\|[^\n]*?Count:\s*(\d+)", info)
        counts[topic] = int(m.group(1)) if m else 0
    if duration < 30.0 or not all(counts.values()):
        raise ValueError(f"기록 품질 실패: {folder.name} / {duration:.3f}s / {counts}")
    lidar_rate = counts[TOPICS[1]] / duration
    clock_rate = counts[TOPICS[0]] / duration
    if lidar_rate > 75 or clock_rate > 1400:
        raise ValueError(
            f"중복 발행 의심 파일이므로 제출 불가: {folder.name} "
            f"LiDAR={lidar_rate:.1f}/s /clock={clock_rate:.1f}/s"
        )
    return duration, counts


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def check_odom(folder):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(folder), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/j100_0000/platform/odom"]))
    start = None
    last = None
    max_xy = 0.0
    max_yaw = 0.0
    n = 0
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if topic != "/j100_0000/platform/odom":
            continue
        msg = deserialize_message(payload, Odometry)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        current = (p.x, p.y, yaw(q))
        if start is None:
            start = current
        x0, y0, a0 = start
        x, y, a = current
        max_xy = max(max_xy, math.hypot(x-x0, y-y0))
        max_yaw = max(max_yaw, abs(math.degrees(math.atan2(math.sin(a-a0), math.cos(a-a0)))))
        last = current
        n += 1
    if start is None:
        raise ValueError(f"Odometry 누락: {folder.name}")
    final_xy = math.hypot(last[0]-start[0], last[1]-start[1])
    return n, final_xy, max_xy, max_yaw


def hardlink_copy(src, dest):
    try:
        os.link(src, dest)
    except OSError:
        shutil.copy2(src, dest)
    return dest


def main():
    if OUTPUT.exists():
        raise SystemExit(f"이미 제출 폴더가 있습니다: {OUTPUT} (기존 결과 보호를 위해 종료)")
    for name in WORLD_FILES:
        if not (WORLDS / name).is_file():
            raise SystemExit(f"SDF 파일이 없습니다: {WORLDS / name}")
    checks = []
    for scenario, actors, first, second in CASES:
        for repeat, bagname in enumerate((first, second), 1):
            bag = BAG_DIR / bagname
            if not bag.is_dir():
                raise SystemExit(f"ROSBag 폴더가 없습니다: {bag}")
            duration, counts = inspect_bag(bag)
            n, final_xy, max_xy, max_yaw = check_odom(bag)
            checks.append((scenario, actors, repeat, bagname, duration,
                           counts[TOPICS[1]], counts[TOPICS[3]], n,
                           final_xy, max_xy, max_yaw))
            print(f"{bagname}: {duration:.3f}s, 최대 위치차 {max_xy:.6f}m, 최대 각도차 {max_yaw:.4f}deg")

    (OUTPUT / "worlds").mkdir(parents=True)
    (OUTPUT / "scripts").mkdir()
    (OUTPUT / "rosbags").mkdir()
    for name in WORLD_FILES:
        shutil.copy2(WORLDS / name, OUTPUT / "worlds" / name)
    for name in ("task2_record_30s.py", "task2_repeat_all.py", "task2_check_stationary.py", "task2_tf_qos.yaml"):
        src = HOME / name
        if src.is_file():
            shutil.copy2(src, OUTPUT / "scripts" / name)
    for row in checks:
        bagname = row[3]
        shutil.copytree(BAG_DIR / bagname, OUTPUT / "rosbags" / bagname, copy_function=hardlink_copy)

    with (OUTPUT / "validation.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(("scenario", "pedestrians", "repeat", "bag_folder", "duration_sim_s",
                         "lidar_count", "tf_count", "odom_count", "final_xy_m",
                         "max_xy_m", "max_yaw_deg"))
        writer.writerows(checks)

    description = "\n".join(
        f"- {code}: {actors}; recordings: {first}, {second}"
        for code, actors, first, second in CASES
    )
    readme = f"""# Task 2 — Multi-Pedestrian LiDAR Simulation

Environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, simulated Clearpath Jackal.
Robot stays at its spawn location (x=0, y=0, z=0.3; yaw=0). Its stationary
behavior is evaluated from recorded `/j100_0000/platform/odom` in validation.csv.

## Scenarios
{description}

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
"""
    (OUTPUT / "README.md").write_text(readme)
    print(f"\n14개 ROSBag 검증 및 제출 폴더 준비 완료: {OUTPUT}")
    print("rosbags/의 DB 파일은 가능한 경우 원본과 하드링크하여 디스크 중복 사용을 줄입니다.")


if __name__ == "__main__":
    main()
