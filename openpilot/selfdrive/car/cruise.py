import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
PCM_SETTLE_FRAMES = 20  # BluePilot: see pcm_owns_target; 0.2 s, as BluePilot 7.1's cluster follower


# BluePilot
def mph_on_grid(mph: int) -> float:
  """A whole mph as a target in km/h: on openpilot's 1.6 km/h step grid, at the point that reads back as the same
  whole mph (ICBM and the HUD round to whole mph). At 86 mph, the only one where no point both reads back and steps
  both ways, the point a Set- reads right from."""
  reads = lambda x: round(min(x, V_CRUISE_MAX) * CV.KPH_TO_MPH)  # noqa: E731
  n = round(mph * CV.MPH_TO_KPH / IMPERIAL_INCREMENT)
  candidates = [c for c in (min(k * IMPERIAL_INCREMENT, V_CRUISE_MAX) for k in (n, n - 1, n + 1)) if reads(c) == mph]
  candidates.sort(key=lambda c: reads(c - IMPERIAL_INCREMENT) != mph - 1)
  return float(candidates[0]) if candidates else float(min(mph * CV.MPH_TO_KPH, V_CRUISE_MAX))


def is_pcm_code(CS) -> bool:
  """Ford's display signal sends 254 (Unknown) and 255 (Fault) as codes, in the car's own unit; 254 km/h, the smaller,
  is above any set speed either unit shows."""
  return CS.cruiseState.speedCluster * CV.MS_TO_KPH >= 253.5
# End BluePilot
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    # BluePilot: see pcm_owns_target
    self.pcm_target = False
    self.pcm_settle = 0
    self.pcm_handed_back = False
    self.pcm_last_press = None
    # End BluePilot

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    self.get_minimum_set_speed(is_metric)

    _enabled = self.update_enabled_state(CS, enabled)

    if CS.cruiseState.available:
      if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
        # if stock cruise is completely disabled, then we can use our own set speed logic
        # BluePilot: on a car whose PCM holds the set speed, the target can be the PCM's for a while (see pcm_owns_target)
        if not (self.CP.pcmCruise and self.pcm_owns_target(CS, is_metric)):
          before, self.pcm_last_press = self.v_cruise_kph, None
          self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
          if self.CP.pcmCruise:
            self.tap_moves_a_whole_mph(before, CS, is_metric)
        if self.update_speed_limit_assist_v_cruise_non_pcm() and self.pcm_target:
          self.hand_back_pcm_target()
        # End BluePilot
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
        # BluePilot: with ICBM (pcmCruise, not pcmCruiseSpeed) this target is the one openpilot keeps once engaged: the
        # same value an engage would take (pcm_set_speed), and no target or shown set speed for a code such as Ford's
        # Unknown/Fault
        elif self.CP.pcmCruise and not self.CP_SP.pcmCruiseSpeed:
          set_speed = self.pcm_set_speed(CS, is_metric)
          self.v_cruise_kph = set_speed if set_speed is not None else V_CRUISE_UNSET
          if is_pcm_code(CS):
            self.v_cruise_cluster_kph = V_CRUISE_UNSET
        if self.CP.pcmCruise and not self.CP_SP.pcmCruiseSpeed:
          self.pcm_press_while_disengaged()
        # End BluePilot
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      self.pcm_target, self.pcm_settle = False, 0  # BluePilot: see pcm_owns_target

    if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
      self.update_button_timers(CS, enabled)
      # BluePilot: see hand_back_pcm_target; after the timers, so a press that began on this frame is included
      if self.pcm_handed_back:
        self.pcm_handed_back = False
        for button, timer in self.button_timers.items():
          if timer > 0:
            self.button_change_states[button]["enabled"] = False
      # End BluePilot

  # BluePilot
  def pcm_owns_target(self, CS, is_metric) -> bool:
    """ICBM on a car whose PCM holds the set speed (Ford with openpilot longitudinal): whether the target is the PCM's
    own set speed this frame, in which case openpilot's button arithmetic does not run.

    Nothing gives the target a value at an engage: initialize_v_cruise leaves a PCM-cruise car alone, and the PCM's
    set speed can show a few frames after carControl.enabled, when the target is still unset (255 km/h). Set- taps then
    counted down from V_CRUISE_MAX: two taps made it 143 km/h, under ICBM's 90 mph "unset" hold, and ICBM pressed Set+
    toward it. So the PCM owns the target while it is unset (or out of range), and takes the PCM's set speed as soon
    as that is a real one (pcm_set_speed). A Set+/Set- press that began in that time is the PCM's to apply: the
    ownership lasts while it is held and PCM_SETTLE_FRAMES after its release, since the display can update a frame or
    two after the button. A Speed Limit Assist write ends it (hand_back_pcm_target). Otherwise openpilot's own logic
    runs, exactly as before."""
    held = self.pcm_button_held()
    unset = self.v_cruise_kph == V_CRUISE_UNSET or self.v_cruise_kph > V_CRUISE_MAX
    settling = self.pcm_target and self.pcm_settle > 0
    if self.pcm_target and held:
      self.pcm_settle = PCM_SETTLE_FRAMES
    elif self.pcm_settle > 0:
      self.pcm_settle -= 1
    if not (unset or (self.pcm_target and (held or settling))):
      self.pcm_target = False
      return False

    self.pcm_target = True
    set_speed = self.pcm_set_speed(CS, is_metric)
    if set_speed is not None:
      self.v_cruise_kph = set_speed
    elif unset:
      self.v_cruise_kph = V_CRUISE_UNSET
    return True

  def pcm_set_speed(self, CS, is_metric) -> float | None:
    """The PCM's displayed set speed as a target in km/h, or None when it shows none. Only a real one: Ford's display
    signal sends 254 (Unknown) and 255 (Fault) as codes. On openpilot's own step grid (1 km/h, or IMPERIAL_INCREMENT
    for a mph display), at the grid point that reads back as the same whole number in ICBM, so a long press steps from
    it like from any other target and 86 to 90 mph do not read back one low. Not below openpilot's minimum set speed,
    as everywhere else in openpilot: ICBM then brings the PCM up to it."""
    kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    if not 0 < kph <= V_CRUISE_MAX + 0.5:   # + 0.5: the signal is a Float32, and 145 km/h reads 145.000003
      return None
    if is_metric:
      grid = float(min(round(kph), V_CRUISE_MAX))
    else:
      grid = mph_on_grid(round(CS.cruiseState.speedCluster * CV.MS_TO_MPH))
    return float(max(grid, self.v_cruise_min))

  def pcm_press_while_disengaged(self) -> None:
    """openpilot not (effectively) engaged, the PCM's cruise on: a Set+/Set- held now is the PCM's to apply, and
    openpilot only becomes effectively engaged once it is released (update_enabled_state), so the ownership and its
    settle window carry into the engagement. A release just before the engage keeps counting the window down; without
    a press, nothing from an earlier engagement is kept."""
    if self.pcm_button_held():
      self.pcm_target, self.pcm_settle = True, PCM_SETTLE_FRAMES
    elif self.pcm_target and self.pcm_settle > 0:
      self.pcm_settle -= 1   # a release just before the engage: the window keeps counting, as it would engaged
    else:
      self.pcm_target, self.pcm_settle = False, 0

  def pcm_button_held(self) -> bool:
    """A cruise button is down: Set+/Set- (button_timers) or Set/Resume (enable_button_timers). Ford reports its
    Set- as Set when the PCM was not yet engaged at the press, and keeps that type through the release."""
    return any(timer > 0 for timer in self.button_timers.values()) or any(timer > 0 for timer in self.enable_button_timers.values())

  def hand_back_pcm_target(self) -> None:
    """Speed Limit Assist wrote the target: it is openpilot's again. A press held now, including one that began on
    this frame, was the PCM's to apply, so its release is not counted on top of the limit (marked in update_v_cruise
    once this frame's button events are registered)."""
    self.pcm_target, self.pcm_settle = False, 0
    self.pcm_handed_back = True

  def tap_moves_a_whole_mph(self, before: float, CS, is_metric: bool) -> None:
    """An imperial ICBM car: a tap the handler applied (pcm_last_press, its own tap / long-press decision) whose result
    still reads the same whole mph moves one more 1.6 km/h step the same way. openpilot's 1.6 km/h step drifts half a
    mph from whole mph above 85 mph, where a tap could otherwise leave the target on the mph the PCM just left, and ICBM
    would press it back. Long presses keep openpilot's own result, and so does a Set- with the gas down (the target is
    the current speed then)."""
    press = self.pcm_last_press
    if is_metric or press is None or press[1] or V_CRUISE_UNSET in (before, self.v_cruise_kph) or before > V_CRUISE_MAX:
      return
    button = press[0]
    if CS.gasPressed and button != ButtonType.accelCruise:
      return
    if round(self.v_cruise_kph * CV.KPH_TO_MPH) == round(before * CV.KPH_TO_MPH):
      step = IMPERIAL_INCREMENT * CRUISE_INTERVAL_SIGN[button]
      self.v_cruise_kph = float(np.clip(round(self.v_cruise_kph + step, 1), self.v_cruise_min, V_CRUISE_MAX))
  # End BluePilot

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # Speed Limit Assist for Non PCM long cars.
    # True: Disallow set speed changes when user confirmed the target set speed during preActive state
    # False: Allow set speed changes as SLA is not requesting user confirmation
    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    self.pcm_last_press = (button_type, long_press)  # BluePilot: see tap_moves_a_whole_mph
    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, initial_speed: int) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(max(CS.vEgo * CV.MS_TO_KPH, initial_speed), V_CRUISE_INITIAL, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
