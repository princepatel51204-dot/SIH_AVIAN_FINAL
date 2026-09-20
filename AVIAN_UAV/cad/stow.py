"""Solve a stow pose with no arm self-collision and adequate ground clearance."""
import itertools, math
import numpy as np
import params_b as P, kin_b as K

# One capsule per RIGID link. The wrist links plus the F/T sensor, tool changer
# and tool are collinear and bolted together -- treating them as separate
# segments makes every pose report a false self-collision.
LINK_R = [P.vJ1D / 2, P.vJ2D / 2, P.vJ3D / 2,
          P.vJ4D / 2, P.vJ5D / 2, P.vJ6D / 2]


def segs(q):
    """[(a, b, r)] one capsule per link: base->J2, J2->J3 ... J6->tool tip."""
    f = K.joint_frames(q)
    tip = K.tool_frames(q)[3]
    pts = [f[1][:3, 3]] + [F[:3, 3] for F in f[2:7]] + [tip[:3, 3]]
    return [(pts[i], pts[i + 1], LINK_R[i]) for i in range(6)]


def seg_dist(p1, q1, p2, q2):
    u, v = q1 - p1, q2 - p2
    w = p1 - p2
    a, b, c = u @ u, u @ v, v @ v
    d, e = u @ w, v @ w
    D = a * c - b * b
    if D < 1e-9:
        sc = 0.0
        tc_ = float(np.clip(e / c, 0, 1)) if c > 1e-9 else 0.0
    else:
        sc = np.clip((b * e - c * d) / D, 0, 1)
        tc_ = np.clip((a * e - b * d) / D, 0, 1)
    return float(np.linalg.norm(w + sc * u - tc_ * v))


def self_clear(q):
    S = segs(q)
    worst = 1e9
    for i in range(len(S)):
        for j in range(i + 2, len(S)):        # skip adjacent (shared joint)
            d = seg_dist(S[i][0], S[i][1], S[j][0], S[j][1]) - S[i][2] - S[j][2]
            worst = min(worst, d)
    return worst


def ground_clear(q):
    S = segs(q)
    return min(min(a[2], b[2]) - r for a, b, r in S) - P.vGearGround


def envelope(q):
    S = segs(q)
    return (max(max(abs(a[0]), abs(b[0])) for a, b, _ in S),
            max(max(abs(a[1]), abs(b[1])) for a, b, _ in S))


if __name__ == "__main__":
    best = []
    for j2 in np.arange(60, 116, 2.0):
        for j3 in np.arange(-179, -90, 2.0):
            for j5 in np.arange(-90, 110, 10.0):
                q = [0.0, float(j2), float(j3), 0.0, float(j5), 0.0]
                sc = self_clear(q)
                if sc < 8.0:
                    continue
                g = ground_clear(q)
                if g < 90.0:
                    continue
                ex, ey = envelope(q)
                best.append((ex, g, sc, q))
    best.sort()
    print(f"{len(best)} collision-free stow poses")
    for e, g, s, q in best[:10]:
        print(f"  x_env {e:6.1f}  ground {g:6.1f}  self {s:5.1f}  "
              f"q {[round(v,0) for v in q]}")
