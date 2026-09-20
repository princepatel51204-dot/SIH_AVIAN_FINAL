"""Run every phase test and write a consolidated gate report."""
import json, os, subprocess, sys, time, uuid
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITES = [("phase1a_description", "tests/test_description.py"),
          ("phase1c_environment", "tests/test_environment_collision.py"),
          ("phase1d_dynamics", "tests/test_vehicle_dynamics.py"),
          ("phase1d_determinism", "tests/test_determinism.py"),
          ("phase1e_sensors", "tests/test_sensors.py"),
          ("phase1f_manipulator", "tests/test_manipulator.py")]

# Every suite (and make_phase1_report.py) runs with this id in its
# environment and stamps it into the JSON it writes. That is what lets the
# report tell "this suite ran cleanly just now" apart from "an old log from a
# previous run is still sitting in logs/ because this run crashed before
# overwriting it" -- the two looked identical before, and a crashed suite's
# stale PASS from a prior run got reported as if it were fresh.
run_id = uuid.uuid4().hex[:12]
env = {**os.environ, "AVIAN_RUN_ID": run_id}

out, tp, tf = {"run_id": run_id}, 0, 0
t0 = time.time()
for name, path in SUITES:
    r = subprocess.run([sys.executable, path], cwd=ROOT, env=env,
                       capture_output=True, text=True)
    tail = [l for l in r.stdout.splitlines() if " pass, " in l]
    out[name] = {"returncode": r.returncode, "summary": tail[-1] if tail else "",
                 "stdout_tail": r.stdout.splitlines()[-14:]}
    for l in tail:
        p, f = l.split(" pass, ")[0].strip(), l.split(" pass, ")[1].split(" fail")[0]
        tp += int(p); tf += int(f)
    print(f"{name:22s} {'OK' if r.returncode==0 else 'FAIL'}  {tail[-1] if tail else ''}")
out["total"] = {"pass": tp, "fail": tf, "wall_s": round(time.time()-t0,1)}
json.dump(out, open(os.path.join(ROOT,"logs","gate_report.json"),"w"), indent=2)
print(f"\nGATE: {tp} pass, {tf} fail")

# Consolidated Phase 1 report, built from the JSON the suites just wrote.
# Same run_id passed through, so it can tell fresh logs from stale ones.
r = subprocess.run([sys.executable, "scripts/make_phase1_report.py"],
                   cwd=ROOT, env=env, capture_output=True, text=True)
print(r.stdout.strip() or r.stderr.strip())
sys.exit(0 if tf==0 else 1)
