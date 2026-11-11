"""BluePilot: launch bias (optional, off by default).

A small extra acceleration for the first few mph when pulling away from a stop under openpilot
longitudinal in Experimental Mode. Measured on a 2023 Expedition, openpilot reaches 10 mph about half a
second later than Ford's own ACC because the driving model's request ramps up gently.

The bias is added to the MODEL's acceleration request only, before the planner takes the minimum of that
and the lead/cruise MPC. So:
  - lead following always has the last word: the result can never exceed what the MPC allows
  - it has no effect in chill mode, or whenever the MPC (not the model) is the limit

It is zero unless ALL of these hold:
  - the setting is on and openpilot is enabled
  - neither the planner nor the model is asking to stop, and the model's own request is positive
  - the model is not anticipating a brake press (same brake-probability gate upstream's accel boost uses)
  - speed is under ~5 mph (full below ~2 mph, fading to nothing by ~5 mph)
  - there is no lead, or the lead is moving, not being closed on, and far enough ahead

It is slew-limited so it comes in and out smoothly, and the planner's max-acceleration clip still applies.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

LAUNCH_BIAS_LEVELS = (0.0, 0.2, 0.35, 0.5)   # off, low, medium, high: m/s^2
FADE_SPEED = (1.0, 2.2)                      # m/s: full below ~2 mph, gone by ~5 mph
MIN_MODEL_ACCEL = 0.0                        # m/s^2: the model's own request must already be positive
BRAKE_PROB = (0.1, 0.3)                      # model's predicted brake press: full bias below 10%, none from 30%
LEAD_GAP = (5.0, 8.0)                        # m: no bias under 5 m to the lead, full from 8 m
LEAD_MIN_SPEED = 0.5                         # m/s: the lead must actually be moving
LEAD_MAX_CLOSING = -0.25                     # m/s: no bias while closing on the lead faster than this
BUILD, RELEASE = 1.5, 3.0                    # m/s^3


def level_to_accel(level) -> float:
  try:
    return LAUNCH_BIAS_LEVELS[int(np.clip(int(level or 0), 0, len(LAUNCH_BIAS_LEVELS) - 1))]
  except (TypeError, ValueError):
    return 0.0


class LaunchBias:
  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.amount = 0.0
    self.bias = 0.0

  def update_params(self) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.amount = level_to_accel(self.params.get("BPLaunchBias"))

  def wanted(self, sm, a_target: float, should_stop: bool) -> float:
    if self.amount <= 0.0 or not sm['selfdriveState'].enabled:
      return 0.0
    model = sm['modelV2']
    if should_stop or model.action.shouldStop or a_target <= MIN_MODEL_ACCEL:
      return 0.0
    brake_probs = model.meta.disengagePredictions.brakePressProbs
    if len(brake_probs) < 2:
      return 0.0   # no prediction to check against: stay out
    scale = float(np.interp(sm['carState'].vEgo, FADE_SPEED, [1.0, 0.0]))
    scale *= float(np.interp(brake_probs[1], BRAKE_PROB, [1.0, 0.0]))
    lead = sm['radarState'].leadOne
    if lead.status:
      if lead.vLead < LEAD_MIN_SPEED or lead.vRel < LEAD_MAX_CLOSING:
        return 0.0
      scale *= float(np.interp(lead.dRel, LEAD_GAP, [0.0, 1.0]))
    return self.amount * scale

  def update(self, sm, a_target: float, should_stop: bool) -> float:
    if self.amount <= 0.0 and self.bias == 0.0:
      return a_target
    target = self.wanted(sm, a_target, should_stop)
    if should_stop or not sm['selfdriveState'].enabled:
      self.bias = 0.0   # no tail when stopping or disengaged
    else:
      self.bias = float(np.clip(target, self.bias - RELEASE * DT_MDL, self.bias + BUILD * DT_MDL))
    return a_target + self.bias
