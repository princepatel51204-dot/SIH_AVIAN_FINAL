"""AVIAN manipulator kinematics. Frames match the Onshape mate scheme 1:1."""
from __future__ import annotations
import math
import numpy as np
import params_b as P
from helpers import T, R, chain, xform_vec

ARM_BASE = chain(T(*P.vArmBase))
JN = [j[0] for j in P.JOINTS]


def joint_frames(q):
    """[arm_base_link, J1..J6, tool0] as 4x4 in base_link."""
    M = ARM_BASE.copy()
    out = [M.copy()]
    for i, (nm, off, ax, lim, tq, sp, fn) in enumerate(P.JOINTS):
        M = chain(M, T(*off), R(ax, q[i]))
        out.append(M.copy())
    out.append(chain(M, T(0, 0, -P.vL5)))
    return out


def tool_frames(q):
    f = joint_frames(q)
    t0 = f[-1]
    fts = chain(t0, T(0, 0, -P.vFTSH))
    tc = chain(fts, T(0, 0, -P.vTCMasterH - P.vTCToolH))
    tip = chain(t0, T(0, 0, -P.vToolOffset))
    return t0, fts, tc, tip


def tool_point(q):
    return tool_frames(q)[3][:3, 3]


def tool_axis(q):
    return xform_vec(tool_frames(q)[3], (0, 0, -1))


def within_limits(q):
    return all(j[3][0] - 1e-6 <= v <= j[3][1] + 1e-6 for v, j in zip(q, P.JOINTS))


def skeleton(q, n=18, from_joint=0):
    """Dense point cloud along the arm centreline incl. the tool stack."""
    f = joint_frames(q)
    pts = [F[:3, 3] for F in f][from_joint:]
    t0, fts, tc, tip = tool_frames(q)
    pts += [fts[:3, 3], tc[:3, 3], tip[:3, 3]]
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        for k in range(n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)


def prop_forward_extent():
    return max(P.vRMotor * math.cos(math.radians(a)) + P.vPropR
               for a in P.ARM_ANGLES)


def rotor_centres():
    return [(P.vRMotor * math.cos(math.radians(a)),
             P.vRMotor * math.sin(math.radians(a))) for a in P.ARM_ANGLES]


def workspace(n=80):
    """Reachable tool points in the J1 = 0 plane, tool axis unconstrained."""
    pts = []
    l2, l3 = P.JOINTS[1][3], P.JOINTS[2][3]
    for q2 in np.linspace(l2[0], l2[1], n):
        for q3 in np.linspace(l3[0], l3[1], n):
            q = [0.0, float(q2), float(q3), 0.0, 0.0, 0.0]
            pts.append(tool_point(q))
    return np.array(pts)
