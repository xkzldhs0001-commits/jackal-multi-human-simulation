#!/usr/bin/env python3
"""Audit Task 2 bags for duplicate /clock and LiDAR publishers by message rate.
Run: python3 ~/task2_audit_bags.py
"""
import re
import subprocess
from pathlib import Path

BASE = Path.home() / 'task2_rosbags'
CASES = [
    ('S01', 'S01_run02', 'S01_run03'),
    ('S02', 'S02_run01', 'S02_run02'),
    ('S03', 'S03_run01', 'S03_run02'),
    ('S04', 'S04_run01', 'S04_run02'),
    ('S05', 'S05_run02', 'S05_run03'),
    ('S06', 'S06_run01', 'S06_run02'),
    ('S07', 'S07_run01', 'S07_run02'),
]
LIDAR = '/j100_0000/sensors/lidar3d_0/points'
REQUIRED = ('/clock', LIDAR, '/j100_0000/platform/odom', '/j100_0000/tf', '/j100_0000/tf_static')
# Expected from the first seven good captures: ~1,000 /clock and ~50 LiDAR messages per sim second.
# These are diagnostic upper bounds for this particular simulator, not universal ROS 2 limits.
MAX_CLOCK_HZ = 1400
MAX_LIDAR_HZ = 75


def inspect_bag(name):
    dest = BASE / name
    if not dest.is_dir():
        return None, None, None, '없음'
    result = subprocess.run(['ros2', 'bag', 'info', str(dest)], capture_output=True, text=True)
    if result.returncode:
        return None, None, None, '정보 조회 실패'
    info = result.stdout
    m = re.search(r'Duration:\s*([\d.]+)s', info)
    if not m:
        return None, None, None, '시간 정보 없음'
    duration = float(m.group(1))
    counts = {}
    for topic in REQUIRED:
        mm = re.search(rf'Topic:\s*{re.escape(topic)}\s*\|[^\n]*?Count:\s*(\d+)', info)
        counts[topic] = int(mm.group(1)) if mm else 0
    if duration < 30 or any(v <= 0 for v in counts.values()):
        verdict = '필수 기록 불충분'
    elif counts['/clock'] / duration > MAX_CLOCK_HZ or counts[LIDAR] / duration > MAX_LIDAR_HZ:
        verdict = '중복 발행 의심'
    else:
        verdict = '기초 검사 통과'
    return duration, counts[LIDAR] / duration, counts['/clock'] / duration, verdict


def main():
    print('시나리오   파일명          길이(s)  LiDAR/s  CLOCK/s    판정')
    print('-' * 78)
    suspect = []
    for scenario, first, second in CASES:
        for name in (first, second):
            duration, lidar_hz, clock_hz, verdict = inspect_bag(name)
            if verdict != '기초 검사 통과':
                suspect.append(name)
            print(f'{scenario:<8} {name:<15} {duration if duration is not None else 0:>6.2f} '
                  f'{lidar_hz if lidar_hz is not None else 0:>8.1f} '
                  f'{clock_hz if clock_hz is not None else 0:>8.1f}   {verdict}')
    print('\n참고: 기초 검사 통과는 발행자 중복이 완전히 없었다는 독립적 증명이 아닙니다.')
    if suspect:
        print('재검증/재녹화 대상:', ', '.join(suspect))
    else:
        print('모든 14개 파일이 기초 검사를 통과했습니다.')

if __name__ == '__main__':
    main()
