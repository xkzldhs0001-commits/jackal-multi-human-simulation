import os
import sys
import time
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

if len(sys.argv) != 2:
    raise SystemExit("사용법: python3 ~/task2_record_30s.py S01_run02")

name = sys.argv[1]
folder = Path.home() / "task2_rosbags"
folder.mkdir(exist_ok=True)

output = folder / name

if output.exists():
    raise SystemExit(f"이미 존재하는 녹화 파일입니다: {output}")

available = set(
    subprocess.check_output(
        ["ros2", "topic", "list"], text=True
    ).splitlines()
)

lidar = "/j100_0000/sensors/lidar3d_0/points"

if "/clock" not in available or lidar not in available:
    raise SystemExit("ERROR: /clock 또는 LiDAR 토픽이 없습니다.")

topics = ["/clock", lidar]

# 현재 실제로 존재하는 TF / 오도메트리 토픽도 기록
extras = sorted(
    t for t in available
    if t.endswith(("/tf", "/tf_static", "/odom"))
)

topics += [t for t in extras if t not in topics]

print("기록할 토픽:")
for topic in topics:
    print(" -", topic)

rclpy.init()
node = rclpy.create_node("task2_bag_duration_monitor")

latest = None
start = None

def on_clock(msg):
    global latest
    latest = msg.clock.sec + msg.clock.nanosec * 1e-9

qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)

node.create_subscription(Clock, "/clock", on_clock, qos)

cmd = [
    "ros2", "bag", "record",
    "--use-sim-time",
    "--qos-profile-overrides-path",
    str(Path.home() / "task2_tf_qos.yaml"),
    "-o", str(output),
    *topics
]

log_path = folder / f"{name}.log"

with log_path.open("w") as log:
    process = subprocess.Popen(
        cmd,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True
    )

    wall_start = time.monotonic()
    ready_after = wall_start + 3
    last_print = -1

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)

            if process.poll() is not None:
                raise RuntimeError("ROSBag 녹화 프로세스가 종료됐습니다.")

            if time.monotonic() - wall_start > 180:
                raise RuntimeError("시뮬레이션 시간 진행이 너무 느립니다.")

            if time.monotonic() < ready_after or latest is None:
                continue

            if start is None:
                start = latest
                print("녹화 시간 측정 시작")

            elapsed = latest - start

            if elapsed < 0:
                raise RuntimeError("시뮬레이션 시간이 초기화됐습니다.")

            seconds = int(elapsed)

            if seconds % 5 == 0 and seconds != last_print:
                print(f"시뮬레이션 시간: {elapsed:.1f}초")
                last_print = seconds

            if elapsed >= 30.5:
                print("30초 이상 측정 완료. 녹화를 종료합니다.")
                break

    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)

        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)

        node.destroy_node()
        rclpy.shutdown()

print("ROSBag:", output)
print("로그:", log_path)
