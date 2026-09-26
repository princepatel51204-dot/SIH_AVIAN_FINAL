"""Offline check of the follower's loop order and retreat route (no PX4, no simulator).

Builds the node on a tiny made-up plan, fakes only what those methods read, and
checks (1) the ping-pong visiting order, (2) the route flown home from a mid-leg
interrupt in each direction, (3) that the first interrupt returns home and the
second lands. Run on an isolated domain:
  ROS_DOMAIN_ID=88 python3 gazebo/mission_follower/test_follower_loop.py
"""
import json
import os
import sys
import tempfile

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
fails = 0


def check(name, ok, detail=''):
    global fails
    fails += (not ok)
    print(('PASS ' if ok else 'FAIL ') + name + (f'  {detail}' if detail else ''))


home_air = [20.0, -30.0, 5.0]
def wp(i, x, via):
    return {'waypoint_id': f'T{i}', 'position_m': [x, 0.0, 8.0], 'heading_rad': 0.0, 'gimbal_pitch_rad': 0.0,
            'route_in': via, 'route_length_m': 10.0, 'reachable_in_plan': True}
W = [wp(0, 30.0, [[25.0, -30.0, 8.0]]), wp(1, 40.0, []), wp(2, 50.0, [[45.0, 5.0, 8.0]]), wp(3, 60.0, [])]
plan = {'home_ground_world_m': [20.0, -30.0, 0.0], 'home_air_world_m': home_air, 'rth': {'route': []}, 'waypoints': W}
d = tempfile.mkdtemp(prefix='avian_follower_test_')
pp = os.path.join(d, 'plan.json')
json.dump(plan, open(pp, 'w'))

rclpy.init(args=['--ros-args', '-p', f'plan:={pp}', '-p', f'results_dir:={d}', '-p', 'demo_laps:=2'])
import mission_follower_node as M  # noqa: E402
n = M.MissionFollower()

class Pos:
    def __init__(self, ned): self.x, self.y, self.z = ned; self.heading = 0.0; self.vx = self.vy = self.vz = 0.0
def put(world):
    n.pos = Pos(n.ned_from_world(world))

# 1. visiting order over 2 laps: replay advance() with next_waypoint stubbed to record the target
order, ended = [0], []
n.next_waypoint = lambda: order.append(n.i)
n.begin_retreat = lambda reason, mid_leg: ended.append((reason, mid_leg, n.i))
n.t_start_wall = 1e18
n.i = 0
for _ in range(20):
    if ended:
        break
    n.advance()
check('ping-pong order for 2 laps: 0 1 2 3 2 1 0 1 2 3 2 1 0', order == [0, 1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 0], str(order))
check('after the last lap the vehicle heads home from waypoint 0', ended == [('demo_laps_done', False, 0)], str(ended))
n.begin_retreat = M.MissionFollower.begin_retreat.__get__(n)

# 2. retreat route from a mid-leg interrupt, forward pass, heading to waypoint 2 (leg 1 -> 2 via [45,5,8])
n.dir, n.i = 1, 2
put([44.0, 3.0, 8.0])
n.start_leg('T2', W[2]['route_in'], W[2]['position_m'], 0.0)
n.leg['seg'] = 1                                    # passed the via point at 45,5? no: still on 1 -> via
# leg pts: [pos(44,3,8), via(45,5,8), final(50,0,8)]; seg 0 = pos->via. Put vehicle past the via:
put([46.0, 4.0, 8.0]); n.leg['seg'] = 1
sent = {}
n.start_leg_orig = n.start_leg
def rec(name, via, final, yaw, **kw):
    sent.update(name=name, via=via, final=final)
n.start_leg = rec
n.set_phase = lambda p: sent.update(phase=p)
n.begin_retreat('interrupt', mid_leg=True)
want = [[45.0, 5.0, 8.0], [40.0, 0.0, 8.0], [30.0, 0.0, 8.0], [25.0, -30.0, 8.0]]
got = [[round(v, 2) for v in p] for p in sent['via']]
check('forward interrupt: back along the leg flown, then reversed route pieces down to waypoint 0',
      got == want, str(got))
check('...ends over the home point and phase is RTH', sent['final'] == home_air and sent['phase'] == 'RTH')
check('...passes waypoints 1 and 0 on the way (T1 at x=40, T0 at x=30)',
      any(abs(p[0] - 40.0) < 0.01 for p in got) and any(abs(p[0] - 30.0) < 0.01 for p in got), str(got))
check('...contains only route points from the plan or flown vertices (no invented points)',
      all(any(np.allclose(p, q, atol=0.01) for q in
              [w['position_m'] for w in W] + sum([w['route_in'] for w in W], []) + [[45.0, 5.0, 8.0]]) for p in got), str(got))

# 3. backward pass, heading to waypoint 1 from waypoint 2: finish this leg, then carry on down
n.dir, n.i = -1, 1
put([48.0, 0.0, 8.0])
n.start_leg = n.start_leg_orig
n.start_leg('T1', list(reversed(W[2]['route_in'])), W[1]['position_m'], 0.0)
put([44.0, 4.0, 8.0]); n.leg['seg'] = 1              # past the via on the way back
sent.clear(); n.start_leg = rec
n.begin_retreat('interrupt', mid_leg=True)
got = [[round(v, 2) for v in p] for p in sent['via']]
check('backward interrupt: continues to waypoint 1, then 0, then home', got[0][0] > 0 and any(abs(p[0] - 40.0) < 0.01 for p in got)
      and any(abs(p[0] - 30.0) < 0.01 for p in got) and sent['final'] == home_air, str(got))

# 4. interrupt state machine
phases = []
n.set_phase = lambda p: (phases.append(p), setattr(n, 'phase', p))
n.phase = 'MISSION'; n.interrupts = 1
retreats = []
n.begin_retreat = lambda reason, mid_leg: (retreats.append(reason), setattr(n, 'phase', 'RTH'))
n.handle_interrupt()
check('first interrupt in MISSION -> retreat home', retreats == ['interrupt'] and n.phase == 'RTH')
n.interrupts = 2; n.handle_interrupt()
check('second interrupt while returning -> land here', phases[-1] == 'LAND')
n.phase = 'TAKEOFF'; n.interrupts = 3; phases.clear(); n.handle_interrupt()
check('interrupt during takeoff -> land here', phases[-1] == 'LAND')

print('\nRESULT:', 'ALL PASS' if not fails else f'{fails} FAILED')
n.destroy_node(); rclpy.shutdown()
sys.exit(1 if fails else 0)
