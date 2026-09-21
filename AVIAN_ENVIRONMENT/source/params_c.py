"""REV-C constants.

Anything REV-C-specific lives here rather than in params.py, which stays a
REV-A/REV-B module REV-C runs unchanged.

Every value below that is an assumption rather than a measurement says so
explicitly, per working rule 5.
"""
from __future__ import annotations

import params as P  # noqa: F401  -- REV-C constants are read alongside these


# ===========================================================================
# SWEPT CORPUS PARAMETER -- concrete surface weathering
# ===========================================================================
# This is NOT an aesthetic setting. It is a corpus parameter in the same
# sense as the paper's BOND_VIS sweep (Table 7: BOND_VIS 0.10 -> 0.60 moves
# certified area 10.1% -> 20.3% while validity does not move). Surface
# weathering is the visual analogue of that knob: it changes how hard the
# defects are to see without changing where they are or whether the scene is
# valid, so detection performance can be reported as a function of it
# instead of against one arbitrary backdrop.
#
# 0.0 = clean concrete, defect contrast at its most generous
# 1.0 = fully realistic coastal weathering -- the default, because
#       "measure what the sensor can actually resolve" is the project's
#       thesis, and keeping the concrete artificially clean would flatter a
#       detector that has not been written yet.
#
# Sweep it; do not tune it. Reported alongside every contrast result.
CONCRETE_WEATHER_STRENGTH = 1.0

# The sweep points used when reporting reclassification. 0.0 and 1.0 are the
# endpoints the Stage 1b gate reports; intermediate points exist so a later
# detection study has a grid to work on rather than two corners.
CONCRETE_WEATHER_SWEEP = (0.0, 0.25, 0.5, 0.75, 1.0)


# ===========================================================================
# CONTRAST MEASUREMENT
# ===========================================================================
# ASSUMPTION. Michelson contrast below which a defect is treated as not
# detectable by RGB regardless of how well it is resolved. There is no
# single right number: 0.01-0.02 is near the human threshold for a large
# uniform patch, but a small feature on mottled, streaked concrete needs
# considerably more to separate from the background's own variation. 0.05 is
# chosen as a deliberately conservative automated-detector threshold and is
# swept-adjacent -- if a detection study later measures its own threshold,
# this should be replaced by that measurement, not re-tuned by eye.
CONTRAST_MIN_MICHELSON = 0.05

# Albedo-pass render settings for the contrast sampler. The pass is diffuse
# COLOUR -- surface reflectance with no lighting in it -- so one sample is
# exact, not an approximation, and the metric stays valid across every
# lighting scenario in scenarios.py. Rendered in EEVEE, which never touches
# Embree; see contrast_c._setup_albedo_render for why that matters here.
CONTRAST_RES_PX = 64
CONTRAST_SAMPLES = 1

# Fraction of the frame the defect's own extent is framed to fill. The
# centre disc inside this radius is sampled as the defect, the annulus
# outside CONTRAST_ANNULUS_R0 as the host surface behind it.
CONTRAST_DEFECT_FILL = 0.40
CONTRAST_DISC_R = 0.20      # centre disc radius, fraction of frame half-width
CONTRAST_ANNULUS_R0 = 0.60  # annulus inner radius
CONTRAST_ANNULUS_R1 = 0.95  # annulus outer radius


# ===========================================================================
# ENVIRONMENT (recorded assumptions, see master brief S1.3)
# ===========================================================================
# This machine's system Blender is 4.0.2, not the 4.5 LTS the brief names:
# no pip bpy wheel exists for its Python 3.12 (bpy ships cp311 only). REV-B
# reproduced at 0.00 mm ground-truth drift under 4.0.2, so it is proven
# adequate for REV-B; it is NOT proven for REV-C's new mesh and material
# work. If a REV-C feature needs a 4.5-only API, stop rather than work
# around it.
BLENDER_VERSION_ASSUMED = "4.0.2"

# This Blender is built without OpenImageDenoiser, and rendering in the same
# process that just built the scene segfaults inside libembree4 4.3.0
# (measured: the same scenes render fine under EMBREE from a saved file in a
# fresh process, REV-B's included). Both are environment properties, not
# REV-C ones, and both affect build_scene_b.py --render equally.
RENDER_FORCE_BVH2 = True
RENDER_DISABLE_DENOISE = True
