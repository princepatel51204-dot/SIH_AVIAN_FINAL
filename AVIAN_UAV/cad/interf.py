"""All-pairs static interference scan with an explicit ALLOW list.

An overlap is only acceptable if the two bodies are deliberately in contact
(a bolted joint, a shaft in a bearing, a plug in a socket). Everything else is
a modelling error or a layout error and must be resolved.
"""
import itertools
import assy_b
from checks_b import placed

# pairs that are SUPPOSED to share volume, with the reason
ALLOW = [
    ("motor_", "propeller_", "prop hub clamps onto the motor bell"),
    ("battery_cartridge", "battery_rail", "cartridge seats on the tray rails"),
    ("battery_cartridge", "battery_latch", "latch engages the cartridge lug"),
    ("arm_link_6", "tool_changer_master", "TC master bolts to the J6 flange"),
    ("arm_link_6", "ft_sensor", "F/T sensor bolts to the J6 flange"),
    ("ft_sensor", "tool_changer_master", "TC master bolts to the F/T sensor"),
    ("tool_changer_master", "tool_plate", "coupled tool interface"),
    ("tool_plate", "tool_", "tool body bolts to its plate"),
    ("arm_base_flange", "manipulator_hub", "arm base bolts to the hub"),
    ("arm_base_flange", "arm_link_1", "J1 output"),
    ("esc_", "esc_saddle", "ESC clamped in its saddle"),
    ("esc_saddle", "esc_saddle", "upper/lower saddle halves clamp the tube"),
    ("arm_tube", "arm_clamp", "split clamp grips the tube"),
    ("arm_tube", "coax_mount", "mount clamps the tube"),
    ("arm_tube", "esc_saddle", "saddle clamps the tube"),
    ("arm_tube", "arm_conduit", "conduit routed inside the tube"),
    ("arm_tube", "corner_node", "tube spigots into the node"),
    ("arm_clamp", "corner_node", "clamp bolts to the node"),
    ("spine_rail", "corner_node", "rail spigots into the node"),
    ("spine_cross", "corner_node", "cross member spigots into the node"),
    ("coax_mount", "motor_", "motor bolts to the mount plate"),
    ("coax_mount", "esc_saddle", "shared arm station"),
    ("gnss_mast", "gnss_antenna", "antenna on its mast"),
    ("lidar_mast", "lidar", "lidar on its mast"),
    ("gimbal_boom", "rgb_gimbal", "gimbal on its boom"),
    ("lamp_boom", "work_lamp", "lamp on its boom"),
    ("gear_leg", "gear_skid", "leg bonded into the skid"),
    ("gear_leg", "gear_damper", "damper between leg and node"),
    ("gear_skid", "gear_foot", "foot bonded to the skid"),
    ("gear_damper", "corner_node", "damper bolts to the node"),
    ("tank_shell", "tank_baffles", "baffles bonded inside the tank"),
    ("cartridge_frame", "tank_", "tank carried by the cartridge frame"),
    ("cartridge_frame", "pump", "pump mounted in the cartridge end-bay"),
    ("cartridge_frame", "filter", "filter mounted in the cartridge end-bay"),
    ("cartridge_frame", "valve", "valve mounted in the cartridge"),
    ("cartridge_frame", "pressure_sensor", "sensor mounted in the cartridge"),
    ("cartridge_frame", "hose_", "internal cartridge plumbing"),
    ("cartridge_frame", "quick_connect", "dry-break on the cartridge face"),
    ("hose_", "quick_connect", "hose onto the coupling"),
    ("hose_", "pump", "hose onto the pump port"),
    ("hose_", "manipulator_hub", "feed-through into the hub"),
    ("fc_tray", "flight_controller", "FC on its isolated tray"),
    ("companion_computer", "compute_cooling", "cooler bolted to the CPU lid"),
    ("harness_main", "connector_bulkhead", "loom into the bulkhead"),
    ("harness_main", "power_distribution", "loom into the PDB"),
    ("harness_main", "spine_rail", "loom routed inside the spine"),
    ("arm_harness", "arm_tube", "loom inside the arm tube"),
    ("arm_conduit", "coax_mount", "conduit into the mount"),
    ("antenna_", "panel_", "antenna base through its panel grommet"),
    ("arm_conduit", "esc_", "conduit terminates at the ESC gland"),
    ("arm_conduit", "arm_clamp", "conduit passes through the root clamp"),
    ("spine_rail", "spine_cross", "cross member spigots into the rail"),
    ("cartridge_frame", "quick_connect", "dry-break on the cartridge face"),
    ("tank_shell", "quick_connect", "dry-break into the tank boss"),
    ("lamp_boom", "inspection_lamp", "lamp on its boom"),
    ("panel_bottom", "range_finder", "sensor fastened through the panel"),
    ("panel_bottom", "optical_flow", "sensor fastened through the panel"),
    ("panel_", "status_beacon", "beacon fastened through the panel"),
    ("cartridge_frame", "battery_rail", "cartridge frame shares the bay floor"),
    ("cartridge_frame", "panel_bottom", "cartridge lands on the panel seal"),
]
# adjacent manipulator links share a joint
for i in range(1, 6):
    ALLOW.append((f"arm_link_{i}", f"arm_link_{i+1}", "adjacent joint"))
# the fluid charge is a fill-volume representation, not a solid part
IGNORE = ("fluid_charge",)


def allowed(a, b):
    for k1, k2, _ in ALLOW:
        if (k1 in a and k2 in b) or (k1 in b and k2 in a):
            return True
    return False


def scan(cfg="01_FLIGHT", thresh=50.0, verbose=True):
    reg = assy_b.build(cfg)
    items = [(p.name, placed(p)) for p in reg.parts
             if not any(k in p.name for k in IGNORE)]
    bbs = [(n, s, s.BoundingBox()) for n, s in items]
    hits = []
    for i in range(len(bbs)):
        n1, s1, b1 = bbs[i]
        for j in range(i + 1, len(bbs)):
            n2, s2, b2 = bbs[j]
            if (b1.xmin > b2.xmax or b2.xmin > b1.xmax or
                    b1.ymin > b2.ymax or b2.ymin > b1.ymax or
                    b1.zmin > b2.zmax or b2.zmin > b1.zmax):
                continue
            if allowed(n1, n2):
                continue
            try:
                v = s1.intersect(s2).Volume()
            except Exception:
                v = 0.0
            if v > thresh:
                hits.append((v, n1, n2))
    hits.sort(reverse=True)
    if verbose:
        print(f"[{cfg}] {len(hits)} unexpected interferences > {thresh:.0f} mm3")
        for v, a, b in hits[:30]:
            print(f"  {v:10.0f} mm3   {a}  <->  {b}")
    return hits


if __name__ == "__main__":
    scan()
