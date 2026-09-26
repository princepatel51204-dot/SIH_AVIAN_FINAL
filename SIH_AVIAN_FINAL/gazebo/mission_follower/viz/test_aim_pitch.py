"""Unit checks for the follower's plan-driven gimbal aiming (no ROS graph needed).

Run:  source /opt/ros/jazzy/setup.bash; source ~/GarudaNEX/ros2_ws/install/setup.bash
      ROS_DOMAIN_ID=71 python3 gazebo/mission_follower/viz/test_aim_pitch.py
"""
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('mf', os.path.join(HERE, '..', 'mission_follower_node.py'))
mf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mf)
f = lambda *a: math.degrees(mf.MissionFollower.target_pitch_up(None, *a))  # noqa: E731
cases = [
    ('level target 8 m ahead of the pivot', f([0, 0, 0], 0.0, [8.35, 0, 0.05]), 0.0),
    ('8 ahead, 8 below the pivot', f([0, 0, 0], 0.0, [8.35, 0, -7.95]), -45.0),
    ('8 ahead, 8 above the pivot', f([0, 0, 0], 0.0, [8.35, 0, 8.05]), 45.0),
    ('heading north, target 8 m north', f([0, 0, 0], math.pi / 2, [0, 8.35, 0.05]), 0.0),
    ('straight above the body: behind the pivot, past vertical, clamped to the joint limit', f([0, 0, 0], 0.0, [0, 0, 8.0]), 90.0),
    ('straight below the body, clamped', f([0, 0, 0], 0.0, [0, 0, -8.0]), -90.0),
    ('body pitch term: 5 deg nose-up, level target', math.degrees(mf.MissionFollower.target_pitch_up(None, [0, 0, 0], 0.0, [8.35, 0, 0.05], math.radians(5))), -5.0),
]
bad = 0
for name, got, want in cases:
    ok = abs(got - want) < 1e-3
    bad += not ok
    print(('PASS ' if ok else 'FAIL ') + f'{name}: {got:+.3f} deg (expect {want:+.1f})')
print('RESULT:', 'ALL PASS' if not bad else f'{bad} FAILED')
sys.exit(1 if bad else 0)
