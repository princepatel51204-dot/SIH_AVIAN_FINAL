"""Solve mechanical J2/J3/J5 limits that are collision-free by construction.

A 900 mm 6-DOF arm on a 510 mm airframe can always be COMMANDED into its own
aircraft over the full +/-115 / +/-160 / +/-120 deg range. Relying on a
software keep-out to prevent that is weaker than choosing hard stops that make
the collision poses mechanically unreachable, so the limits are solved here and
then frozen into params_b.JOINTS.

Clearance sources, all evaluated together:
  * voxel field  -- airframe + equipment (12 mm pitch)
  * rotor discs  -- at the real propeller world height
  * gear boxes   -- landing gear only
  * self         -- link-vs-link capsules
"""
from __future__ import annotations
import numpy as np
import params_b as P
import kin_b as K
import stow
import voxel
import assy_b

CLEAR_BODY = 15.0
CLEAR_ROTOR = 50.0
CLEAR_SELF = 10.0
NSEG = 8
SEG_R = [P.vJ2D / 2, P.vJ3D / 2, P.vJ4D / 2, P.vJ5D / 2, P.vJ6D / 2,
         P.vFTSD / 2, P.vTCMasterD / 2, P.vTCToolD / 2]


def field_map(n2=25, n3=27, n5=5, n1=73):
    reg = assy_b.build("01_FLIGHT")
    occ, F = voxel.build("01_FLIGHT")
    j2 = np.linspace(*P.JOINTS[1][3], n2)
    j3 = np.linspace(*P.JOINTS[2][3], n3)
    j5 = np.linspace(*P.JOINTS[4][3], n5)
    j1 = np.linspace(-180.0, 180.0, n1)

    local = []
    for b in j2:
        for c in j3:
            for e in j5:
                local.append(K.skeleton([0.0, float(b), float(c), 0.0,
                                         float(e), 0.0],
                                        n=NSEG, from_joint=2))
    L = np.stack(local)
    NP = L.shape[1]
    RAD = np.repeat(np.array(SEG_R), NSEG + 1)[:NP]

    ax, ay = P.vArmBase[0], P.vArmBase[1]
    dx, dy, z = L[..., 0] - ax, L[..., 1] - ay, L[..., 2]
    th = np.radians(j1)
    ct, st = np.cos(th)[:, None, None], np.sin(th)[:, None, None]
    X = ax + ct * dx[None] - st * dy[None]
    Y = ay + st * dx[None] + ct * dy[None]
    Z = np.broadcast_to(z[None], X.shape)

    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    body = voxel.sample(F, pts).reshape(X.shape) - RAD[None, None, :]

    rz = sorted({round(float(p.world[2, 3]), 1) for p in reg.parts
                 if "propeller" in p.name})
    rot = np.full(X.shape, 1e9)
    for zc in rz:
        dz = np.abs(Z - zc) - 8.0
        for (mx, my) in K.rotor_centres():
            rad = np.hypot(X - mx, Y - my)
            rot = np.minimum(rot,
                             np.where(rad <= P.vPropR, dz,
                                      np.hypot(np.clip(dz, 0, None),
                                               rad - P.vPropR))
                             - RAD[None, None, :])
    for p in [q for q in reg.parts if q.group == "10_LANDING_GEAR"]:
        bb = voxel.placed(p).BoundingBox() if hasattr(voxel, "placed") else None
    from checks_b import placed
    gear = np.full(X.shape, 1e9)
    for p in [q for q in reg.parts if q.group == "10_LANDING_GEAR"]:
        bb = placed(p).BoundingBox()
        lo = np.array([bb.xmin, bb.ymin, bb.zmin])
        hi = np.array([bb.xmax, bb.ymax, bb.zmax])
        gear = np.minimum(gear,
                          np.sqrt(np.maximum(lo[0] - X, X - hi[0]).clip(0) ** 2
                                  + np.maximum(lo[1] - Y, Y - hi[1]).clip(0) ** 2
                                  + np.maximum(lo[2] - Z, Z - hi[2]).clip(0) ** 2)
                          - RAD[None, None, :])

    sh = (len(j1), n2, n3 * n5, NP)
    bmin = body.reshape(sh).min(axis=3)
    rmin = rot.reshape(sh).min(axis=3)
    gmin = gear.reshape(sh).min(axis=3)
    # keep the J1 axis: the clear envelope of a vehicle-mounted arm
    # is azimuth-dependent (gear legs and rotor arms sit at fixed
    # bearings), so collapsing J1 first throws the answer away.
    okf = ((bmin >= CLEAR_BODY) & (rmin >= CLEAR_ROTOR) &
           (gmin >= CLEAR_BODY)).reshape(len(j1), n2, n3, n5)
    ok_j1 = okf.all(axis=3)          # (j1, j2, j3)

    self_ok = np.zeros((n2, n3), dtype=bool)
    for i, b in enumerate(j2):
        for k, c in enumerate(j3):
            self_ok[i, k] = all(
                stow.self_clear([0.0, float(b), float(c), 0.0, float(e), 0.0])
                >= CLEAR_SELF for e in j5)
    return j1, j2, j3, ok_j1 & self_ok[None, :, :]


def largest_box(j2, j3, ok):
    """Widest axis-aligned (J2, J3) rectangle that is entirely clear.

    Scored on retained tool reach, not on area: a wide J3 range with no J2
    travel is useless.
    """
    n2, n3 = ok.shape
    best = None
    for i0 in range(n2):
        for i1 in range(i0 + 2, n2 + 1):
            for k0 in range(n3):
                for k1 in range(k0 + 2, n3 + 1):
                    if not ok[i0:i1, k0:k1].all():
                        continue
                    r = 0.0
                    for b in (j2[i0], j2[i1 - 1], (j2[i0] + j2[i1 - 1]) / 2):
                        for c in np.linspace(j3[k0], j3[k1 - 1], 15):
                            t = K.tool_point([0.0, float(b), float(c),
                                              0.0, 0.0, 0.0])
                            r = max(r, float(np.hypot(t[0] - P.vArmBase[0],
                                                      t[2] - P.vArmBase[2])))
                    span = (j2[i1 - 1] - j2[i0]) + (j3[k1 - 1] - j3[k0])
                    score = r + 0.6 * span
                    if best is None or score > best[0]:
                        best = (score, (j2[i0], j2[i1 - 1]),
                                (j3[k0], j3[k1 - 1]), r, span)
    return best


def sector_report():
    j1, j2, j3, ok = field_map()
    print(f"grid: J1 {len(j1)} x J2 {len(j2)} x J3 {len(j3)}")
    for half in (30.0, 45.0, 60.0, 90.0, 120.0, 180.0):
        m = np.abs(j1) <= half + 1e-6
        sub = ok[m].all(axis=0)
        n = int(sub.sum())
        b = largest_box(j2, j3, sub) if n else None
        if b is None:
            print(f"  J1 +/-{half:5.0f} deg : {n:4d} cells  -- no usable box")
        else:
            _, l2, l3, r, span = b
            print(f"  J1 +/-{half:5.0f} deg : {n:4d} cells  "
                  f"J2 [{l2[0]:+6.1f},{l2[1]:+6.1f}]  "
                  f"J3 [{l3[0]:+6.1f},{l3[1]:+6.1f}]  reach {r:5.0f} mm")
    return j1, j2, j3, ok


if __name__ == "__main__":
    j1, j2, j3, ok = sector_report()
    np.savez("avian_arm_envelope.npz", j1=j1, j2=j2, j3=j3, ok=ok)
    print("saved avian_arm_envelope.npz")
