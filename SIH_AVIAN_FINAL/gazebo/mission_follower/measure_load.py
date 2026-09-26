#!/usr/bin/env python3
"""Sample the demo's load while it runs: CPU per process group, package temperature,
CPU clock and the simulator's real-time factor. Measurement only; touches nothing.

  python3 measure_load.py <out_prefix> [--secs 240] [--every 2] [--skip 30] [--abort-temp 96]

Every --every seconds it records, for each group below, the summed CPU % (100 = one
core), the package temperature and mean clock, and every 5 s the windowed RTF
(delta sim time / delta wall time from the world's own /stats). The first --skip
seconds are excluded from the summary (start-up transients). If the package stays
>= --abort-temp for 3 consecutive samples it sends the follower a SIGINT (return +
land) and stops, so a measurement can never cook the machine.
"""
import argparse
import csv
import json
import os
import re
import subprocess
import time

import numpy as np
import psutil

GROUPS = [
    ('gz_server', r'gz sim .*-s|gz-sim-server|ruby.*gz.* sim.*-s'),
    ('gz_gui', r'gz sim .*-g'),
    ('px4', r'/px4( |$)|bin/px4'),
    ('xrce_agent', r'MicroXRCEAgent'),
    ('ros_gz_bridge', r'parameter_bridge'),
    ('follower', r'mission_follower_node'),
    ('detector', r'live_detector_node'),
    ('map_localizer', r'map_viz_node'),
    ('robot_state_pub', r'robot_state_publisher'),
    ('rviz2', r'rviz2/rviz2|rviz2 -d'),
    ('camera_window', r'rqt_image_view'),
    ('recorders_audit', r'camera_recorder_node|pose_audit_node|contact_counter_node|ffmpeg'),
]
WORLD = 'sih_avian_final'


def pkg_temp():
    try:
        o = subprocess.run(['sensors'], capture_output=True, text=True, timeout=5).stdout
        m = re.search(r'Package id 0:\s+\+([0-9.]+)', o)
        return float(m.group(1)) if m else None
    except Exception:
        return None


def sim_time():
    try:
        o = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{WORLD}/stats'], capture_output=True,
                           text=True, timeout=6).stdout
        m = re.search(r'sim_time\s*\{\s*sec:\s*(\d+)\s*(?:nsec:\s*(\d+))?', o)
        return int(m.group(1)) + int(m.group(2) or 0) * 1e-9 if m else None
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--secs', type=float, default=240)
    ap.add_argument('--every', type=float, default=2.0)
    ap.add_argument('--skip', type=float, default=30.0)
    ap.add_argument('--abort-temp', type=float, default=96.0)
    a = ap.parse_args()
    rx = [(n, re.compile(p)) for n, p in GROUPS]
    procs = {}
    rows, t0 = [], time.time()
    last_rtf = (None, None)
    hot = 0
    aborted = False
    print(f'sampling {a.secs:.0f} s every {a.every} s -> {a.prefix}_*.csv/json', flush=True)
    while time.time() - t0 < a.secs:
        now = time.time()
        cur = {}
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                cl = ' '.join(p.info['cmdline'] or [])
                for n, r in rx:
                    if r.search(cl):
                        cur[p.pid] = (n, p)
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        cpu = {n: 0.0 for n, _ in GROUPS}
        for pid, (n, p) in cur.items():
            if pid not in procs:
                procs[pid] = p
                p.cpu_percent(None)
                continue
            try:
                cpu[n] += procs[pid].cpu_percent(None)
            except psutil.NoSuchProcess:
                pass
        tmp = pkg_temp()
        freq = float(np.mean([f.current for f in psutil.cpu_freq(percpu=True)])) if psutil.cpu_freq(percpu=True) else None
        tot = psutil.cpu_percent(None)
        rtf = None
        if int(now - t0) % 5 < a.every:
            st = sim_time()
            if st is not None:
                if last_rtf[0] is not None and now - last_rtf[1] > 1.0:
                    rtf = (st - last_rtf[0]) / (now - last_rtf[1])
                last_rtf = (st, now)
        rows.append({'t_s': round(now - t0, 1), 'pkg_temp_c': tmp, 'cpu_mhz': None if freq is None else round(freq),
                     'total_cpu_pct_of_16_cores': tot, 'rtf': None if rtf is None else round(rtf, 3),
                     **{f'cpu_{n}': round(v, 1) for n, v in cpu.items()}})
        if tmp is not None and tmp >= a.abort_temp:
            hot += 1
            if hot >= 3:
                print(f'THERMAL ABORT: package {tmp} C >= {a.abort_temp} C for 3 samples -- follower told to return and land', flush=True)
                subprocess.run(['pkill', '-INT', '-f', 'mission_follower_node.py'])
                aborted = True
                break
        else:
            hot = 0
        time.sleep(max(0.0, a.every - (time.time() - now)))

    with open(a.prefix + '_samples.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    body = [r for r in rows if r['t_s'] >= a.skip]

    def st(key):
        v = [r[key] for r in body if r[key] is not None]
        return None if not v else {'mean': round(float(np.mean(v)), 2), 'p95': round(float(np.percentile(v, 95)), 2),
                                   'max': round(float(np.max(v)), 2), 'n': len(v)}
    summ = {'seconds_sampled': round(rows[-1]['t_s'], 1), 'summary_skips_first_s': a.skip, 'thermal_abort': aborted,
            'pkg_temp_c': st('pkg_temp_c'), 'cpu_mhz': st('cpu_mhz'), 'rtf': st('rtf'),
            'total_cpu_pct_of_16_cores': st('total_cpu_pct_of_16_cores'),
            'per_group_cpu_pct_of_one_core': {n: st(f'cpu_{n}') for n, _ in GROUPS}}
    json.dump(summ, open(a.prefix + '_summary.json', 'w'), indent=1)
    print(json.dumps({k: summ[k] for k in ('pkg_temp_c', 'cpu_mhz', 'rtf', 'total_cpu_pct_of_16_cores')}, indent=1))
    for n, v in summ['per_group_cpu_pct_of_one_core'].items():
        if v:
            print(f'  {n:16s} mean {v["mean"]:7.1f}   p95 {v["p95"]:7.1f}')


if __name__ == '__main__':
    main()
