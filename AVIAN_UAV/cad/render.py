"""Offscreen VTK rendering of the assembly + an analytic projection helper
so engineering annotations can be overlaid exactly on the orthographic views."""
from __future__ import annotations

import math
import os
import numpy as np
import cadquery as cq
import vtk

import params_b as P
from helpers import Part

_CACHE = {}


# ---------------------------------------------------------------------------
# tessellation
# ---------------------------------------------------------------------------
def tessellate(part: Part, tol=None, ang=None):
    tol = P.TESS_TOL if tol is None else tol
    ang = P.TESS_ANG if ang is None else ang
    key = (id(part.solid), tol, ang)
    if key in _CACHE:
        return _CACHE[key]
    try:
        shape = part.solid.val()
    except Exception:
        shape = cq.Compound.makeCompound(part.solid.vals())
    v, t = shape.tessellate(tol, ang)
    verts = np.array([[p.x, p.y, p.z] for p in v], dtype=float)
    tris = np.array(t, dtype=np.int64) if len(t) else np.zeros((0, 3), np.int64)
    _CACHE[key] = (verts, tris)
    return verts, tris


def world_mesh(part: Part, explode=0.0):
    verts, tris = tessellate(part)
    if len(verts) == 0:
        return verts, tris
    M = part.world
    w = (M[:3, :3] @ verts.T).T + M[:3, 3]
    if explode:
        w = w + np.array(part.explode, dtype=float) * explode
    return w, tris


def scene_meshes(reg, explode=0.0, skip=()):
    out = []
    for p in reg.parts:
        if p.name in skip or p.group in skip:
            continue
        v, t = world_mesh(p, explode)
        if len(v) == 0 or len(t) == 0:
            continue
        out.append((p.name, v, t, p.rgb()))
    return out


# ---------------------------------------------------------------------------
# view definitions
# ---------------------------------------------------------------------------
VIEWS = {
    #        camera direction (world -> camera position dir),  up
    "front":  ((+1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "rear":   ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "left":   ((0.0, +1.0, 0.0), (0.0, 0.0, 1.0)),
    "right":  ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
    "top":    ((0.0, 0.0, +1.0), (1.0, 0.0, 0.0)),
    "bottom": ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0)),
    "iso":    ((0.78, 0.52, 0.35), (0.0, 0.0, 1.0)),
    "iso2":   ((-0.62, 0.66, 0.42), (0.0, 0.0, 1.0)),
    "iso_low": ((0.72, 0.48, -0.10), (0.0, 0.0, 1.0)),
}


def view_basis(view):
    d, up = VIEWS[view]
    d = np.array(d, float)
    d = d / np.linalg.norm(d)          # camera position direction from focal point
    f = -d                              # viewing direction
    up = np.array(up, float)
    right = np.cross(f, up)
    if np.linalg.norm(right) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(f, up)
    right /= np.linalg.norm(right)
    tup = np.cross(right, f)
    tup /= np.linalg.norm(tup)
    return d, f, right, tup


def project(pts, view, focal, ps, size):
    """World points -> pixel coords for a parallel-projection render."""
    d, f, right, tup = view_basis(view)
    W, Hh = size
    p = np.atleast_2d(np.asarray(pts, float)) - np.asarray(focal, float)
    sx = p @ right
    sy = p @ tup
    aspect = W / Hh
    px = (sx / (ps * aspect) * 0.5 + 0.5) * W
    py = (0.5 - sy / ps * 0.5) * Hh
    return np.stack([px, py], axis=1)


def scene_bounds(meshes):
    lo = np.array([1e12] * 3)
    hi = np.array([-1e12] * 3)
    for _, v, _, _ in meshes:
        lo = np.minimum(lo, v.min(axis=0))
        hi = np.maximum(hi, v.max(axis=0))
    return lo, hi


def fit_parallel_scale(meshes, view, size, margin=1.06):
    """Tight fit computed from the actual projected geometry, not the AABB."""
    d, f, right, tup = view_basis(view)
    pts = np.vstack([v[::3] if len(v) > 900 else v for _, v, _, _ in meshes])
    sx = pts @ right
    sy = pts @ tup
    cx = (sx.min() + sx.max()) / 2.0
    cy = (sy.min() + sy.max()) / 2.0
    lo, hi = scene_bounds(meshes)
    depth_c = ((lo + hi) / 2.0) @ f
    focal = cx * right + cy * tup + depth_c * f
    ex = (sx.max() - sx.min()) / 2.0
    ey = (sy.max() - sy.min()) / 2.0
    W, Hh = size
    aspect = W / Hh
    ps = max(ey, ex / aspect) * margin
    return focal, ps


# ---------------------------------------------------------------------------
# VTK rendering
# ---------------------------------------------------------------------------
def _polydata(v, t):
    pts = vtk.vtkPoints()
    pts.SetDataTypeToFloat()
    arr = vtk.vtkFloatArray()
    arr.SetNumberOfComponents(3)
    arr.SetNumberOfTuples(len(v))
    for i, p in enumerate(v):
        arr.SetTuple3(i, float(p[0]), float(p[1]), float(p[2]))
    pts.SetData(arr)
    cells = vtk.vtkCellArray()
    ids = np.empty((len(t), 4), dtype=np.int64)
    ids[:, 0] = 3
    ids[:, 1:] = t
    ca = vtk.vtkIdTypeArray()
    ca.SetNumberOfValues(ids.size)
    flat = ids.ravel()
    for i in range(flat.size):
        ca.SetValue(i, int(flat[i]))
    cells.SetCells(len(t), ca)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    n = vtk.vtkPolyDataNormals()
    n.SetInputData(pd)
    n.SetFeatureAngle(38)
    n.SplittingOn()
    n.ConsistencyOn()
    n.Update()
    return n.GetOutput()


def render(meshes, view, out_png, size=(1800, 1350), ortho=True,
           bg=(0.965, 0.968, 0.975), bg2=(0.885, 0.895, 0.912),
           focal=None, ps=None, silhouette=True):
    ren = vtk.vtkRenderer()
    ren.SetBackground(*bg)
    ren.SetBackground2(*bg2)
    ren.GradientBackgroundOn()
    ren.SetUseDepthPeeling(0)

    for name, v, t, rgb in meshes:
        pd = _polydata(v, t)
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(pd)
        a = vtk.vtkActor()
        a.SetMapper(m)
        pr = a.GetProperty()
        pr.SetColor(*rgb)
        pr.SetAmbient(0.22)
        pr.SetDiffuse(0.78)
        lum = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
        pr.SetSpecular(0.34 if lum > 0.4 else 0.20)
        pr.SetSpecularPower(28)
        pr.SetInterpolationToPhong()
        ren.AddActor(a)

    if focal is None or ps is None:
        focal, ps = fit_parallel_scale(meshes, view, size)
    d, f, right, tup = view_basis(view)
    cam = ren.GetActiveCamera()
    dist = max(ps * 8.0, 4000.0)
    cam.SetFocalPoint(*focal)
    cam.SetPosition(*(np.array(focal) + d * dist))
    cam.SetViewUp(*tup)
    if ortho:
        cam.ParallelProjectionOn()
        cam.SetParallelScale(ps)
    else:
        cam.ParallelProjectionOff()
        cam.SetViewAngle(20.0)
        cam.SetPosition(*(np.array(focal) + d * ps / math.tan(math.radians(10.0))))
    cam.SetClippingRange(dist * 0.05, dist * 3.0)

    lk = vtk.vtkLightKit()
    lk.SetKeyLightIntensity(1.08)
    lk.SetKeyLightWarmth(0.56)
    lk.SetFillLightWarmth(0.46)
    lk.SetKeyToFillRatio(2.6)
    lk.SetKeyToHeadRatio(3.4)
    lk.SetKeyToBackRatio(3.0)
    lk.SetKeyLightElevation(46)
    lk.SetKeyLightAzimuth(28)
    lk.AddLightsToRenderer(ren)

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(ren)
    rw.SetSize(*size)
    rw.SetMultiSamples(8)
    rw.Render()

    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.SetScale(1)
    w2i.ReadFrontBufferOff()
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(out_png)
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    rw.Finalize()
    return focal, ps
