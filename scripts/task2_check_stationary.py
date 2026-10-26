from pathlib import Path
from math import atan2, sin, cos, hypot, degrees
import csv
import rosbag2_py

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

base = Path.home() / "task2_rosbags"
topic_name = "/j100_0000/platform/odom"

bags = {
    "S01": "S01_run02",
    "S02": "S02_run01",
    "S03": "S03_run01",
    "S04": "S04_run01",
    "S05": "S05_run02",
    "S06": "S06_run01",
    "S07": "S07_run01",
}

def yaw_from_quaternion(q):
    return atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)
    )

results = []

for scenario, name in bags.items():
    path = base / name

    if not path.exists():
        print(f"{scenario}: 파일 없음: {path}")
        continue

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(path),
            storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions("", "")
    )

    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[topic_name])
    )

    positions = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != topic_name:
            continue

        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        positions.append((
            p.x,
            p.y,
            yaw_from_quaternion(q)
        ))

    if not positions:
        print(f"{scenario}: Odometry 데이터 없음")
        continue

    x0, y0, yaw0 = positions[0]
    x1, y1, yaw1 = positions[-1]

    final_distance = hypot(x1-x0, y1-y0)

    max_distance = max(
        hypot(x-x0, y-y0)
        for x, y, yaw in positions
    )

    max_yaw = max(
        abs(degrees(
            atan2(sin(yaw-yaw0), cos(yaw-yaw0))
        ))
        for x, y, yaw in positions
    )

    results.append([
        scenario,
        name,
        len(positions),
        round(final_distance, 6),
        round(max_distance, 6),
        round(max_yaw, 4)
    ])

    print(
        f"{scenario}: "
        f"시작-종료 이동={final_distance:.5f}m, "
        f"최대 위치 변화={max_distance:.5f}m, "
        f"최대 방향 변화={max_yaw:.3f}도"
    )

output = base / "jackal_stationary_check.csv"

with output.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "Scenario",
        "Bag",
        "Odom Messages",
        "Final Displacement (m)",
        "Max Displacement (m)",
        "Max Yaw Change (deg)"
    ])
    writer.writerows(results)

print("\n검증 결과 저장:", output)
