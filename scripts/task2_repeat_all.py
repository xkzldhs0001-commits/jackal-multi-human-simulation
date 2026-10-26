#!/usr/bin/env python3
"""Record a second set of the seven Task-2 pedestrian scenarios.
Run on the Ubuntu 22.04 / ROS 2 Humble simulator machine after sourcing ROS.
"""
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

HOME = Path.home()
BAG_DIR = HOME / "task2_rosbags"
WORLD_DIR = HOME / "jackal_actor_sim" / "worlds"
RECORDER = HOME / "task2_record_30s.py"
CASES = {
    "S01": ("task2_s01_h1", "S01_run03"),
    "S02": ("task2_s02_h2", "S02_run02"),
    "S03": ("task2_s03_h3", "S03_run02"),
    "S04": ("task2_s04_h12", "S04_run02"),
    "S05": ("task2_s05_h13", "S05_run03"),
    "S06": ("task2_s06_h23", "S06_run02"),
    "S07": ("task2_s07_h123", "S07_run02"),
}
LIDAR_TOPIC = "/j100_0000/sensors/lidar3d_0/points"
MAX_CLOCK_HZ = 1400
MAX_LIDAR_HZ = 75
DOMAINS = {"S01": 61, "S02": 62, "S03": 63, "S04": 64, "S05": 65, "S06": 66, "S07": 67}
REQUIRED = [
    "/clock",
    "/j100_0000/sensors/lidar3d_0/points",
    "/j100_0000/platform/odom",
    "/j100_0000/tf",
    "/j100_0000/tf_static",
]


def verify_bag(name):
    location = BAG_DIR / name
    result = subprocess.run(
        ["ros2", "bag", "info", str(location)],
        text=True, capture_output=True, timeout=25,
    )
    if result.returncode:
        raise RuntimeError(f"ROSBag 검증 실패: {result.stderr.strip()}")
    info = result.stdout
    duration = re.search(r"Duration:\s+([0-9.]+)s", info)
    if not duration or float(duration.group(1)) < 30.0:
        raise RuntimeError("ROSBag Duration이 30초 미만입니다.")
    counts = {}
    for topic in REQUIRED:
        pattern = rf"Topic:\s*{re.escape(topic)}\s*\|[^\n]*?Count:\s*(\d+)"
        match = re.search(pattern, info)
        if not match or int(match.group(1)) == 0:
            raise RuntimeError(f"필수 토픽 누락 또는 메시지 0개: {topic}")
        counts[topic] = int(match.group(1))
    secs = float(duration.group(1))
    lidar_hz = counts[LIDAR_TOPIC] / secs
    clock_hz = counts["/clock"] / secs
    if lidar_hz > MAX_LIDAR_HZ or clock_hz > MAX_CLOCK_HZ:
        raise RuntimeError(
            f"중복 발행 의심: LiDAR {lidar_hz:.1f}/s, /clock {clock_hz:.1f}/s "
            f"(허용 상한 {MAX_LIDAR_HZ}, {MAX_CLOCK_HZ})"
        )
    return secs



def preflight_no_publishers():
    """Fail before launch if previous ROS2 simulation publishers are still alive."""
    rclpy.init()
    node = rclpy.create_node("task2_preflight_publishers")
    try:
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.2)
        counts = {topic: node.count_publishers(topic)
                  for topic in ("/clock", LIDAR_TOPIC)}
        if any(counts.values()):
            raise RuntimeError(
                f"기존 ROS 발행자 감지: {counts}. 기존 Gazebo/브리지를 완전히 종료하세요."
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_for_sensors(launch, timeout=150):
    """Do not start recording until fresh simulated sensor and TF messages arrive."""
    rclpy.init()
    node = rclpy.create_node("task2_capture_readiness")
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    tf_qos = QoSProfile(
        depth=10, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    received = set()
    subscriptions = []
    try:
        subscriptions.append(node.create_subscription(
            Clock, "/clock", lambda msg: received.add("clock")
            if msg.clock.sec > 0 or msg.clock.nanosec > 0 else None,
            best_effort))
        subscriptions.append(node.create_subscription(
            PointCloud2, REQUIRED[1], lambda msg: received.add("lidar"),
            best_effort))
        subscriptions.append(node.create_subscription(
            Odometry, REQUIRED[2], lambda msg: received.add("odom"),
            best_effort))
        subscriptions.append(node.create_subscription(
            TFMessage, REQUIRED[3], lambda msg: received.add("tf")
            if msg.transforms else None, tf_qos))
        needed = {"clock", "lidar", "odom", "tf"}
        deadline = time.monotonic() + timeout
        while not needed.issubset(received):
            if launch.poll() is not None:
                raise RuntimeError("Gazebo 실행 프로세스가 종료되었습니다.")
            if time.monotonic() > deadline:
                missing = ", ".join(sorted(needed - received))
                raise RuntimeError(f"센서/TF 준비 시간 초과. 미수신: {missing}")
            rclpy.spin_once(node, timeout_sec=0.2)
        # Allow late DDS discoveries; check both publisher counts repeatedly.
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.2)
            clock_publishers = node.count_publishers("/clock")
            lidar_publishers = node.count_publishers(LIDAR_TOPIC)
            if clock_publishers != 1 or lidar_publishers != 1:
                raise RuntimeError(
                    f"중복/누락 발행자 감지: /clock={clock_publishers}, "
                    f"LiDAR={lidar_publishers}. 녹화하지 않습니다."
                )
        print("  센서 준비 완료 + 발행자 1개씩 확인: clock / lidar / odom / tf", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def stop_launch(process):
    if process.poll() is not None:
        return
    for sig, seconds in [(signal.SIGINT, 12), (signal.SIGTERM, 8), (signal.SIGKILL, 3)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=seconds)
            break
        except subprocess.TimeoutExpired:
            continue


def run_one(code):
    world, name = CASES[code]
    # Separate ROS domains prevent orphaned bridges from earlier scenarios
    # sharing the recording topics. All spawned ROS nodes inherit this setting.
    os.environ["ROS_DOMAIN_ID"] = str(DOMAINS[code])
    print(f"[{code}] ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}", flush=True)
    world_file = WORLD_DIR / (world + ".sdf")
    if not world_file.is_file():
        raise RuntimeError(f"World SDF 없음: {world_file}")
    destination = BAG_DIR / name
    if destination.exists():
        duration = verify_bag(name)
        print(f"[{code}] 이미 검증된 파일이 있어 건너뜀: {name} ({duration:.3f}s)", flush=True)
        return
    preflight_no_publishers()
    log_path = BAG_DIR / f"{name}_gazebo.log"
    print(f"\n[{code}] Gazebo 시작: {world}", flush=True)
    with log_path.open("w") as log:
        launch = subprocess.Popen(
            ["ros2", "launch", "clearpath_gz", "simulation.launch.py",
             f"world:={world}", "x:=0.0", "y:=0.0", "z:=0.3",
             "yaw:=0.0", "rviz:=false"],
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            wait_for_sensors(launch)
            print(f"[{code}] 30초 이상 ROSBag 녹화: {name}", flush=True)
            subprocess.run(
                [sys.executable, str(RECORDER), name],
                check=True, timeout=240,
            )
            duration = verify_bag(name)
            print(f"[{code}] 검증 완료: {duration:.3f}s, 5개 필수 토픽 수신", flush=True)
        finally:
            print(f"[{code}] Gazebo 종료", flush=True)
            stop_launch(launch)
    time.sleep(4)


def main():
    requested = ["S01", "S02", "S03", "S04", "S05", "S06", "S07"] \
        if len(sys.argv) == 2 and sys.argv[1].upper() == "ALL" \
        else [arg.upper() for arg in sys.argv[1:]]
    if not requested or any(x not in CASES for x in requested):
        raise SystemExit("사용법: python3 ~/task2_repeat_all.py S01  또는 ... ALL")
    if not RECORDER.is_file():
        raise SystemExit(f"녹화 스크립트 없음: {RECORDER}")
    if "--qos-profile-overrides-path" not in RECORDER.read_text():
        raise SystemExit("기존 녹화 스크립트에 TF QoS 설정이 없습니다.")
    BAG_DIR.mkdir(exist_ok=True)
    for code in requested:
        run_one(code)
    print("\n요청한 반복 녹화와 중복 발행 검사까지 완료했습니다.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"\nERROR: {exc}\n문제 해결 전 다음 시나리오를 시작하지 않습니다.", file=sys.stderr)
        sys.exit(1)
