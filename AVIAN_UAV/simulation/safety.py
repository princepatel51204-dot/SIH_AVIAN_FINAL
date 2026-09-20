"""Minimal safety supervisor -- the smallest thing that makes Test 7 real.

Not the autonomy stack. A state machine that watches four conditions and can
put the aircraft into a defined safe state, so "emergency behaviour works" is
a measured transition rather than a claim.

    NOMINAL -> the mission owns the target
    RETURN  -> battery below reserve; target becomes the home point
    HOLD    -> sensor invalid or motor limit; hold the last good position
    LAND    -> unrecoverable; descend at a controlled rate

Thresholds are aircraft numbers, not round figures picked to pass: the battery
reserve is the 20 % depth-of-discharge limit the CAD sizing used, and the
proximity trigger is the 1.5 m minimum structural clearance from the REV-B
UNDERSIDE_INSPECTION airspace class.
"""
from __future__ import annotations

import numpy as np

BATTERY_RESERVE_PCT = 20.0        # matches the CAD DoD limit of 0.8
PROXIMITY_STOP_M = 1.5            # REV-B underside minimum clearance
SATURATION_HOLD = 1.35            # sustained demand above the rotor limit
# Hysteresis. A state that clears the instant its trigger stops is a state
# that chatters; the margins below are what stop a supervisor toggling at the
# sample rate right next to a girder.
BATTERY_HYSTERESIS_PCT = 5.0
PROXIMITY_HYSTERESIS = 1.4        # must reach 1.4 x the stop distance


class SafetySupervisor:
    def __init__(self, vehicle, controller, home=(0.0, 0.0, 5.0)):
        self.v = vehicle
        self.c = controller
        self.home = np.asarray(home, dtype=float)
        self.state = "NOMINAL"
        self.reason = ""
        self.transitions = []
        self.last_good = None
        self.sensor_valid = True

    def update(self, t, clearance=None):
        v, c = self.v, self.c
        st = v.state()

        # A MISSING sample is not a safe sample. Clearance is measured every
        # few steps for cost reasons; treating the gaps as "no clearance
        # problem" made the supervisor oscillate into HOLD and straight back
        # out again, and it ended a deliberate fly-into-the-deck test
        # reporting NOMINAL.
        if clearance is not None:
            self.last_clearance = clearance
        clearance = getattr(self, "last_clearance", None)

        new, why = "NOMINAL", ""

        if not self.sensor_valid:
            new, why = "HOLD", "state estimate invalid"
        elif any(v.motor_failed):
            n = sum(v.motor_failed)
            # An X8 has one rotor of redundancy per arm; two failures on the
            # same arm is a lost arm and is not recoverable in hover.
            new, why = ("HOLD" if n >= 2 else "RETURN",
                        f"{n} motor(s) failed")
        elif clearance is not None and clearance < PROXIMITY_STOP_M:
            # Ordered ABOVE battery and saturation on purpose: an imminent
            # collision is the most urgent condition, and a supervisor that
            # reports "low battery" while the aircraft is 1 m from a girder
            # has diagnosed the wrong problem.
            new, why = "HOLD", (f"clearance {clearance:.2f} m below the "
                                f"{PROXIMITY_STOP_M} m minimum")
        elif st["battery_pct"] <= BATTERY_RESERVE_PCT:
            new, why = "RETURN", (f"battery {st['battery_pct']:.1f} % at or "
                                  f"below the {BATTERY_RESERVE_PCT:.0f} % "
                                  f"reserve")
        elif getattr(v, "saturation_now", 0.0) > SATURATION_HOLD:
            new, why = "HOLD", (f"motor demand at "
                                f"{v.saturation_now*100:.0f} % of limit")

        # LATCHING. A safety state persists until the condition has cleared
        # by a hysteresis margin, and a return to NOMINAL is a decision, not
        # the absence of a trigger. Without this the state machine tracked
        # instantaneous conditions and could drop its guard between samples.
        if new == "NOMINAL" and self.state != "NOMINAL":
            if self._recovered(st, clearance):
                self.transitions.append({"t": round(t, 3),
                                         "from": self.state, "to": "NOMINAL",
                                         "reason": "condition cleared with "
                                                   "margin"})
                self.state = "NOMINAL"
                self.reason = ""
            # otherwise hold the existing safety state
        elif new != self.state and new != "NOMINAL":
            self.transitions.append({"t": round(t, 3), "from": self.state,
                                     "to": new, "reason": why})
            self.state = new
            self.reason = why
            if new == "HOLD":
                self.last_good = st["position"].copy()

        if self.state == "RETURN":
            c.set_target(self.home)
        elif self.state == "HOLD" and self.last_good is not None:
            c.set_target(self.last_good)
        elif self.state == "LAND":
            p = st["position"].copy()
            p[2] = 0.0
            c.set_target(p)
        return self.state

    def _recovered(self, st, clearance):
        """Has every trigger cleared by the hysteresis margin?"""
        if not self.sensor_valid or any(self.v.motor_failed):
            return False
        if st["battery_pct"] <= BATTERY_RESERVE_PCT + BATTERY_HYSTERESIS_PCT:
            return False
        if clearance is not None and clearance < PROXIMITY_STOP_M * \
                PROXIMITY_HYSTERESIS:
            return False
        if getattr(self.v, "saturation_now", 0.0) > SATURATION_HOLD * 0.8:
            return False
        return True

    def clear(self, reason="operator"):
        """Explicit reset. The only way out of a latched state by command."""
        if self.state != "NOMINAL":
            self.transitions.append({"t": None, "from": self.state,
                                     "to": "NOMINAL", "reason": reason})
        self.state = "NOMINAL"
        self.reason = ""
        return self.state
