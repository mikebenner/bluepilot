"""BluePilot: steady the stop/go decision when pulling away from a stop.

At a standstill the driving model's stop flag is simply "requested acceleration below 0.1 m/s^2", so when
its request hovers around that value the flag flips between stop and go several times a second. Each flip
back to "stop" re-applies the hold brake and drops the gas request, then releases again a few frames later.
Measured on a 2023 Expedition: up to 11 flips in the two seconds before the truck moved, with the false
"go"s lasting 0.02-0.16 s.

Once the plan has said "go" from a standstill, a return to "stop" has to persist for STOP_CONFIRM_TIME
before it is passed on. Nothing else is delayed:
  - "go" is never delayed
  - a stop while moving (above STANDSTILL_SPEED) passes straight through, as does an ordinary arrival at a stop
  - a stop that comes with a real braking request (below FIRM_BRAKE) passes straight through
"""
from openpilot.common.realtime import DT_MDL

STOP_CONFIRM_TIME = 0.3   # s
STANDSTILL_SPEED = 0.3    # m/s, the same threshold the model's stop flag uses
FIRM_BRAKE = -0.3         # m/s^2


class GoDebounce:
  def __init__(self):
    self.launching = False     # the plan said "go" from a standstill and the car hasn't got moving yet
    self.prev_stop = True
    self.stop_time = 0.0

  def update(self, should_stop: bool, v_ego: float, a_target: float) -> bool:
    standstill = v_ego < STANDSTILL_SPEED
    if not standstill:
      self.launching = False            # it's moving: from here on stops are passed straight through
    if not should_stop:
      if self.prev_stop and standstill:
        self.launching = True
      self.stop_time = 0.0
      self.prev_stop = False
      return False

    if self.launching and standstill and a_target > FIRM_BRAKE:
      self.stop_time += DT_MDL
      if self.stop_time < STOP_CONFIRM_TIME:
        return False                    # too brief to act on: keep going
    self.launching = False
    self.stop_time = 0.0
    self.prev_stop = True
    return True
