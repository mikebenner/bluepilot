"""ICBM on a car whose PCM holds the set speed (Ford: pcmCruise, not pcmCruiseSpeed): while openpilot has no target,
the target is the PCM's own set speed (VCruiseHelper.pcm_owns_target).

Without that, the target stayed unset after an engage where the PCM's set speed arrived a few frames late, and with
no speed limit for Speed Limit Assist to write, two Set- taps counted down from the 145 km/h maximum: the target became
143 km/h (89 mph) and ICBM pressed Set+ toward it. The real VCruiseHelper, Speed Limit Assist write and ICBM controller
are stepped here at 100 Hz; the planner is stood in for by min(set speed, 145 km/h), as when cruise is its limiting
source. Speed Limit Assist's own state machine is stood in for by setting its state and limit directly.
"""
import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MAX, V_CRUISE_UNSET, PCM_SETTLE_FRAMES
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
LATE = 4  # frames after the engage before the PCM's set speed shows


class Truck:
  """Display values (`shown`) are in the display unit: mph, or km/h with metric=True."""
  def __init__(self, pcm_cruise=True, pcm_cruise_speed=False, op_long=True, metric=False, real_sla=False):
    self.CP = structs.CarParams(brand="ford", pcmCruise=pcm_cruise, openpilotLongitudinalControl=op_long).as_reader()
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed, intelligentCruiseButtonManagementAvailable=True).as_reader()
    self.metric = metric
    self.unit = CV.KPH_TO_MS if metric else CV.MPH_TO_MS
    self.helper = VCruiseHelper(self.CP, self.CP_SP)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP)
    self.helper.custom_acc_enabled = False   # ordinary 1 mph / 5 mph steps, whatever this machine's settings say
    # both start from the module's shared timer dictionary; a test that ends with a button held must not leak
    self.helper.enable_button_timers = dict.fromkeys(CRUISE_BUTTON_TIMER, 0)
    self.icbm.cruise_button_timers = dict.fromkeys(CRUISE_BUTTON_TIMER, 0)
    self.frame = 0
    self.limit = 0.   # Speed Limit Assist active with this limit (display unit) when non-zero
    self.enabled_prev = False
    self.CS_prev = None
    self.sent = []
    self.icbm_targets = []
    self.ego = 15.6   # m/s
    # real_sla: the real Speed Limit Assist state machine (auto-confirm) decides when the limit is written, sampled at
    # 20 Hz with its plan reaching the helper on the next frame, instead of being active whenever a limit is set
    self.sla = None
    if real_sla:
      self.sla = SpeedLimitAssist(self.CP, self.CP_SP)
      self.sla.update_params = lambda: None   # Assist mode and the unit as given, whatever this machine's settings say
      self.sla.enabled = True
      self.sla.is_metric = metric
    self.plan_next = None
    self.plan = custom.LongitudinalPlanSP().as_reader()

  def step(self, shown: float, button=None, pressed=False, late=LATE, enabled=True, unit=None, gas=False) -> None:
    """One 10 ms frame, openpilot engaged unless enabled=False (the PCM's cruise stays on). The display reads 0 for the
    first `late` frames; `unit` overrides the display unit (a car whose display unit differs from openpilot's)."""
    speed = 0.0 if self.frame < late else float(shown) * (unit or self.unit)
    events = [{"type": button, "pressed": pressed}] if button is not None else []
    CS = car.CarState(vEgo=self.ego, gasPressed=gas, buttonEvents=events,
                      cruiseState={"available": True, "enabled": True, "speed": speed, "speedCluster": speed}).as_reader()
    if self.sla is not None:
      if self.plan_next is not None:
        self.plan, self.plan_next = self.plan_next, None
      if self.frame % 5 == 0:
        limit = self.limit * self.unit if self.limit else 0.
        self.sla.update(enabled, False, self.ego, 0., min(self.helper.v_cruise_cluster_kph, V_CRUISE_MAX) * CV.KPH_TO_MS,
                        limit, limit, bool(self.limit), 0., EventsSP())
        self.sla.enabled = True
        plan = custom.LongitudinalPlanSP()
        plan.speedLimit.resolver.speedLimitValid = bool(self.limit)
        plan.speedLimit.resolver.speedLimitFinalLast = limit
        plan.speedLimit.assist.state = self.sla.state
        # what ICBM gets in production: the planner's target, the lower of the cruise target and the limit, as Float32
        plan.vTarget = float(min(min(self.helper.v_cruise_kph, V_CRUISE_MAX) * CV.KPH_TO_MS, self.sla.output_v_target))
        self.plan_next = plan.as_reader()
    else:
      plan = custom.LongitudinalPlanSP()
      if self.limit:
        plan.speedLimit.resolver.speedLimitValid = True
        plan.speedLimit.resolver.speedLimitFinalLast = self.limit * self.unit
        plan.speedLimit.assist.state = "active"
      self.plan = plan.as_reader()
    self.helper.update_speed_limit_assist(self.metric, self.plan)
    self.helper.update_v_cruise(CS, enabled, self.metric)
    if enabled and not self.enabled_prev:
      self.helper.initialize_v_cruise(self.CS_prev if self.CS_prev is not None else CS, 0)
    self.enabled_prev, self.CS_prev = enabled, CS
    if self.sla is not None:
      icbm_plan = self.plan
    else:
      icbm_plan = custom.LongitudinalPlanSP(vTarget=float(min(self.helper.v_cruise_kph, V_CRUISE_MAX)) * CV.KPH_TO_MS).as_reader()
    self.icbm.run(CS, car.CarControl(enabled=enabled).as_reader(), icbm_plan, self.metric)
    self.sent.append(self.icbm.cruise_button)
    self.icbm_targets.append(self.icbm.v_target)
    self.frame += 1

  def run(self, shown: float, frames: int, **kwargs) -> None:
    for _ in range(frames):
      self.step(shown, **kwargs)

  def tap(self, button, after: float, before: float) -> None:
    self.step(before, button, True)
    self.run(before, 9)
    self.step(after, button, False)   # the PCM moves its own set speed one step per tap

  def run_pcm(self, shown: float, frames: int, **kwargs) -> float:
    """Like run, with the PCM taking one ICBM press (a step every 10 frames, Set+ / Set-) as one display unit; returns
    the display at the end."""
    for f in range(frames):
      self.step(shown, **kwargs)
      if f % 10 == 9:
        if self.icbm.cruise_button == SendButtonState.increase:
          shown += 1
        elif self.icbm.cruise_button == SendButtonState.decrease:
          shown -= 1
    return shown

  @property
  def target(self) -> int:
    """The target in the display unit, rounded as ICBM rounds it."""
    return round(float(self.helper.v_cruise_kph) * (1. if self.metric else CV.KPH_TO_MPH))

  @property
  def increased(self) -> bool:
    return SendButtonState.increase in self.sent

  @property
  def decreased(self) -> bool:
    return SendButtonState.decrease in self.sent


class TestUnsetTargetSeed(unittest.TestCase):
  def test_a_late_cluster_speed_becomes_the_target(self):
    truck = Truck()
    truck.run(35, LATE)
    self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)   # nothing to take yet
    truck.step(35)
    self.assertEqual(truck.target, 35)
    truck.run(35, 400)
    self.assertEqual(truck.target, 35)
    self.assertFalse(truck.increased or truck.decreased)

  def test_set_minus_taps_lower_the_target_and_icbm_never_presses_up(self):
    truck = Truck()
    truck.run(35, 400)
    truck.tap(ButtonType.decelCruise, 34, 35)
    truck.run(34, 40)
    truck.tap(ButtonType.decelCruise, 33, 34)
    truck.run(33, 500)   # past the driver-press inhibit and the engage guard
    self.assertEqual(truck.target, 33)
    self.assertEqual(truck.icbm.v_target, 33)
    self.assertFalse(truck.increased)

  def test_a_target_that_has_a_value_is_left_alone(self):
    truck = Truck()
    truck.run(35, 100)
    truck.helper.v_cruise_kph = 80.5   # the driver's own setting, say
    truck.run(35, 300)
    self.assertEqual(truck.helper.v_cruise_kph, 80.5)

  def test_every_set_speed_reads_back_as_itself(self):
    # ICBM rounds the target to whole mph: a seed must come back as the PCM's own number, up to the top
    for mph in list(range(20, 91)):
      with self.subTest(mph=mph):
        truck = Truck()
        truck.run(mph, 400)
        self.assertEqual(truck.icbm.v_target, mph)
        self.assertFalse(truck.increased or truck.decreased)

  def test_a_set_speed_below_the_minimum_is_the_minimum(self):
    # 16 mph is on record on this truck's cluster; openpilot's minimum is 20 mph, and ICBM brings the PCM up to it
    truck = Truck()
    truck.run(16, 500)
    self.assertEqual(truck.target, 20)
    self.assertEqual(truck.icbm.v_target, 20)
    self.assertFalse(truck.decreased)
    self.assertLessEqual(max(truck.icbm_targets), 20)

  def test_the_minimum_holds_from_above(self):
    # 21 -> 20 -> 19 (the PCM taking the tap) or 20 (refusing it): the target stays at the 20 mph minimum
    for last in (19, 20):
      with self.subTest(last=last):
        truck = Truck()
        truck.run(21, 400)
        truck.tap(ButtonType.decelCruise, 20, 21)
        truck.run(20, 500)
        truck.tap(ButtonType.decelCruise, last, 20)
        truck.run(last, 500)
        self.assertEqual(truck.target, 20)
        self.assertLessEqual(max(truck.icbm_targets), 21)

  def test_a_tap_released_before_the_cluster_shows(self):
    # press and release while the PCM still reads 0: the PCM applies the tap; the target takes its result
    truck = Truck()
    truck.step(35)                                    # engage
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(35, ButtonType.decelCruise, False)     # still before the cluster shows (LATE frames)
    truck.run(34, 400)                                # the PCM shows its result, 34
    self.assertEqual(truck.target, 34)
    truck.tap(ButtonType.decelCruise, 33, 34)
    truck.run(33, 500)
    self.assertEqual(truck.target, 33)
    self.assertFalse(truck.increased)

  def test_a_tap_released_on_the_frame_the_cluster_shows(self):
    truck = Truck()
    truck.run(35, LATE - 1)
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(34, ButtonType.decelCruise, False)     # the first non-zero reading is the tap's result
    truck.run(34, 500)
    self.assertEqual(truck.target, 34)                # counted once
    self.assertFalse(truck.increased or truck.decreased)

  def test_a_press_held_across_the_first_reading(self):
    # held from before the cluster shows until well after: the PCM steps down while it is held, the target follows
    truck = Truck()
    truck.step(35)
    truck.step(35, ButtonType.decelCruise, True)
    for mph in (35, 34, 33, 32, 31, 30):
      truck.run(mph, 50)
    truck.step(30, ButtonType.decelCruise, False)
    truck.run(30, 500)
    self.assertEqual(truck.target, 30)
    self.assertFalse(truck.increased)

  def test_the_display_updating_after_the_release_within_the_settle_window(self):
    # the PCM's button and its displayed set speed come in different messages: the last frame of the window counts
    truck = Truck()
    truck.step(35)
    truck.step(35, ButtonType.decelCruise, True)
    truck.run(35, 8)
    truck.step(35, ButtonType.decelCruise, False)     # released; the display still shows 35
    truck.run(35, PCM_SETTLE_FRAMES - 1)
    truck.run(34, 500)                                # and shows 34 on the window's last frame
    self.assertEqual(truck.target, 34)
    self.assertFalse(truck.increased)

  def test_unknown_and_fault_codes_are_not_a_set_speed(self):
    # the display signal's 254 (Unknown) and 255 (Fault), decoded as mph
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.run(code, 20, late=0)
        self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
        truck.run(35, 400, late=0)
        self.assertEqual(truck.target, 35)
        truck.tap(ButtonType.decelCruise, 34, 35)
        truck.run(34, 40)
        truck.tap(ButtonType.decelCruise, 33, 34)
        truck.run(33, 500)
        self.assertEqual(truck.target, 33)
        self.assertFalse(truck.increased)

  def test_a_fault_reading_after_the_seed_presses_nothing(self):
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.run(35, 400)
        sent = len(truck.sent)
        truck.run(code, 120, late=0)
        self.assertEqual(truck.target, 35)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])
        self.assertNotIn(SendButtonState.decrease, truck.sent[sent:])
        truck.run(35, 200, late=0)                    # the display recovers
        self.assertEqual(truck.target, 35)
        self.assertFalse(truck.increased or truck.decreased)

  def test_a_90_mph_set_speed_still_follows_a_lower_target(self):
    truck = Truck()
    truck.run(90, 400)
    self.assertEqual(truck.icbm.v_target, 90)
    truck.limit = 45
    truck.run(90, 200)
    self.assertEqual(truck.target, 45)
    self.assertTrue(truck.decreased)
    self.assertFalse(truck.increased)

  def test_a_limit_written_during_a_held_press_stays(self):
    # Speed Limit Assist becomes active while a press from the PCM-owned window is held: its limit is the target, and
    # the rest of the press goes on from it with openpilot's own arithmetic, as on the base (here its long-press tick:
    # 38 -> 40, 55 -> 60 on the 8 km/h grid)
    for limit, want in ((38, 40), (55, 60)):
      with self.subTest(limit=limit):
        truck = Truck()
        truck.step(20)
        truck.step(20, ButtonType.accelCruise, True)
        truck.run(20, 20)
        truck.limit = limit
        truck.run(21, 30)
        truck.step(21, ButtonType.accelCruise, False)
        truck.run(21, 300)
        self.assertEqual(truck.target, want)

  def test_a_limit_equal_to_the_display_ends_the_ownership(self):
    # metric: a 90 km/h display and a 90 km/h limit give the same target value; the write itself hands it back, and the
    # tap released after it is counted on top of it (91), as on the base
    truck = Truck(metric=True)
    truck.step(90)
    truck.step(90, ButtonType.accelCruise, True)
    truck.run(90, 28)
    truck.limit = 90
    truck.run(90, 2)
    truck.run(95, 8)
    truck.step(95, ButtonType.accelCruise, False)
    truck.run(95, 320)
    self.assertEqual(truck.target, 91)

  def test_a_low_limit_stays_the_target(self):
    # a limit under openpilot's 20 mph minimum is written as the minimum; it is not replaced by the display, and a
    # later Set- does not send the set speed back up
    for limit in (15, 19):
      with self.subTest(limit=limit):
        truck = Truck()
        truck.run(55, 400)
        truck.limit = limit
        truck.run(55, 300)
        self.assertEqual(truck.target, 20)
        truck.tap(ButtonType.decelCruise, 19, 20)
        truck.run(19, 500)
        self.assertEqual(truck.target, 20)
        self.assertLessEqual(max(truck.icbm_targets[400:]), 20)

  def test_a_display_dropout_stops_a_raise_under_way(self):
    # a 16 mph engage: ICBM raises the PCM toward the 20 mph minimum; the display reading 0 stops it at once
    truck = Truck()
    truck.run(16, 96)
    raising = len(truck.sent)
    truck.run(16, 300, late=0)
    self.assertTrue(SendButtonState.increase in truck.sent[raising:])
    dropout = len(truck.sent)
    truck.run(0, 200, late=0)
    self.assertNotIn(SendButtonState.increase, truck.sent[dropout:])
    self.assertNotIn(SendButtonState.decrease, truck.sent[dropout:])
    truck.run(20, 100, late=0)
    self.assertEqual(truck.target, 20)

  def test_a_tap_down_from_86_mph(self):
    # above 85 mph the 1.6 km/h grid is half a mph off whole mph: the seed is the point a Set- also reads right from
    truck = Truck()
    truck.run(86, 400)
    truck.tap(ButtonType.decelCruise, 85, 86)
    truck.run(85, 400)
    self.assertEqual(truck.target, 85)
    self.assertFalse(truck.increased)

  def test_a_set_speed_taken_while_disengaged_is_the_same_value(self):
    # openpilot off with the PCM's cruise on, then on: the target is what an engage would have taken
    for shown, want in ((35, 35), (16, 20)):
      with self.subTest(shown=shown):
        truck = Truck()
        truck.run(shown, 10, late=0, enabled=False)
        truck.run(shown, 400, late=0)
        self.assertEqual(truck.target, want)
    # and a tap after it is openpilot's one step
    truck = Truck()
    truck.run(35, 10, late=0, enabled=False)
    truck.run(35, 400, late=0)
    truck.tap(ButtonType.decelCruise, 34, 35)
    truck.run(34, 400, late=0)
    self.assertEqual(truck.target, 34)
    self.assertFalse(truck.increased)

  def test_a_code_while_disengaged_is_no_target(self):
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.step(code, late=0, enabled=False)
        self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
        truck.step(code, late=0)                      # the engage frame
        self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)

  def test_a_code_in_the_cars_own_unit_holds(self):
    # a car whose display is in km/h while openpilot shows mph: 254 km/h reads 158 mph
    truck = Truck()
    truck.run(35, 400)
    sent = len(truck.sent)
    truck.run(254, 120, late=0, unit=CV.KPH_TO_MS)
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])
    self.assertNotIn(SendButtonState.decrease, truck.sent[sent:])
    self.assertEqual(truck.target, 35)

  def test_145_km_h_is_a_set_speed(self):
    truck = Truck(metric=True)
    truck.run(145, 400)
    self.assertEqual(truck.helper.v_cruise_kph, 145.)

  def test_a_limit_written_on_the_frame_a_press_begins(self):
    # the limit (45) is the target, and the Set- released after it is counted on top of it, as on the base
    truck = Truck()
    truck.run(35, 60, late=60)                        # no display yet
    truck.limit = 45
    truck.step(35, ButtonType.decelCruise, True, late=60)
    truck.run(35, 3, late=60)
    truck.run(35, 6, late=0)
    truck.step(35, ButtonType.decelCruise, False, late=0)
    truck.run(35, 300, late=0)
    self.assertEqual(truck.target, 44)

  def test_a_second_tap_after_the_target_has_a_value_is_openpilots(self):
    # a press that began before the target existed, then another one once it has a value: openpilot counts the second
    # one (taken or refused by the PCM), inside the 0.2 s after the first release or after it, as on the base
    for gap, pcm_after, want in ((PCM_SETTLE_FRAMES - 1, 34, 33), (PCM_SETTLE_FRAMES - 1, 33, 33), (PCM_SETTLE_FRAMES + 5, 33, 33)):
      with self.subTest(gap=gap, pcm_after=pcm_after):
        truck = Truck()
        truck.step(35)
        truck.step(35, ButtonType.decelCruise, True)
        truck.step(34, ButtonType.decelCruise, False, late=2)
        truck.run(34, gap, late=0)
        truck.tap(ButtonType.decelCruise, pcm_after, 34)
        truck.run(pcm_after, 500, late=0)
        self.assertEqual(truck.target, want)
        self.assertFalse(truck.increased)

  def test_a_settle_window_does_not_outlive_its_engagement(self):
    # a press from the unset window, released at 34, then cancel: a minute later, re-engaged at 34, the first Set- is
    # openpilot's to count again
    truck = Truck()
    truck.step(35)
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(34, ButtonType.decelCruise, False, late=2)
    truck.run(34, 6000, late=0, enabled=False)
    truck.run(34, 2, late=0)
    truck.tap(ButtonType.decelCruise, 34, 34)          # the PCM refuses; openpilot counts it, ICBM walks the PCM down
    truck.run(34, 400, late=0)
    self.assertEqual(truck.target, 33)
    self.assertFalse(truck.increased)

  def test_a_press_held_through_the_engage(self):
    # Set- already down on the engage: openpilot is effectively engaged only after the release, and the press is the
    # PCM's; a long press (to 30) and a tap (to 34, shown just after the release)
    for hold, after in ((65, 30), (5, 34)):
      with self.subTest(hold=hold, after=after):
        truck = Truck()
        truck.step(35, ButtonType.decelCruise, True)
        truck.run(35, hold)
        truck.step(35, ButtonType.decelCruise, False)
        truck.run(after, 400, late=0)
        self.assertEqual(truck.target, after)
        self.assertFalse(truck.increased)

  def test_a_real_set_speed_above_90_still_follows_a_lower_target(self):
    truck = Truck()
    truck.run(35, 400)
    truck.limit = 45
    truck.run(35, 50)
    sent = len(truck.sent)
    truck.run(91, 400, late=0)
    self.assertEqual(truck.target, 45)
    self.assertIn(SendButtonState.decrease, truck.sent[sent:])
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_taps_at_the_top_and_a_long_press_read_as_whole_mph(self):
    # (seed, button, long press, the PCM's result, expected target). Known, not covered: a long press at 85-90 mph
    # follows openpilot's own 8 km/h grid, which is a mph off there (90 -> 89, 85 -> 89).
    for seed, button, long, pcm, want in ((86, ButtonType.accelCruise, False, 87, 87), (86, ButtonType.decelCruise, False, 85, 85),
                                          (88, ButtonType.accelCruise, False, 89, 89), (89, ButtonType.decelCruise, False, 88, 88),
                                          (72, ButtonType.decelCruise, True, 70, 70), (35, ButtonType.accelCruise, True, 40, 40)):
      with self.subTest(seed=seed, button=button, long=long):
        truck = Truck()
        truck.run(seed, 400)
        truck.step(seed, button, True)
        truck.run(seed, 65 if long else 9)
        truck.step(pcm, button, False)
        truck.run(pcm, 400)
        self.assertEqual(truck.target, want)
        self.assertFalse(truck.increased and truck.decreased)
        self.assertNotIn(SendButtonState.decrease if button == ButtonType.accelCruise else SendButtonState.increase, truck.sent[400:])

  def test_a_release_on_the_long_press_boundary(self):
    # the handler decides: released with its timer at 49 or 50 it is a tap (one mph), at 51 the long-press tick at 50
    # has fired (Set+ from 35: openpilot's 8 km/h grid, 40); ICBM never presses against the driver. (A long Set- tick
    # from a whole mph can stay on the same mph on that grid, as on the base: not this fix.)
    for button, sign in ((ButtonType.accelCruise, 1), (ButtonType.decelCruise, -1)):
      for frames_held, want in ((48, 35 + sign), (49, 35 + sign), (50, 35 + 5 * sign)):
        if frames_held == 50 and sign < 0:
          continue
        with self.subTest(button=button, frames_held=frames_held):
          truck = Truck()
          truck.run(35, 400)
          truck.step(35, button, True)
          truck.run(35, frames_held)
          truck.step(35 + sign, button, False)
          truck.run(35 + sign, 400)
          self.assertEqual(truck.target, want)
          self.assertNotIn(SendButtonState.decrease if sign > 0 else SendButtonState.increase, truck.sent)

  def test_a_release_just_before_the_engage(self):
    # the press and its release happen while openpilot is not engaged; it engages on the next frame and the PCM's
    # display shows the press's result a frame later. Ford reports a Set- pressed before the PCM engaged as Set.
    for button in (ButtonType.decelCruise, ButtonType.setCruise):
      with self.subTest(button=button):
        truck = Truck()
        truck.run(35, 5, late=0, enabled=False)
        truck.step(35, button, True, late=0, enabled=False)
        truck.run(35, 64, late=0, enabled=False)
        truck.step(35, button, False, late=0, enabled=False)
        truck.step(35, late=0)                          # engaged
        truck.run(30, 400, late=0)
        self.assertEqual(truck.target, 30)
        self.assertFalse(truck.increased)

  def test_a_tap_up_from_86_with_custom_increments_or_the_gas(self):
    for custom_steps, gas in ((True, False), (False, True)):
      with self.subTest(custom_steps=custom_steps, gas=gas):
        truck = Truck()
        if custom_steps:
          truck.helper.custom_acc_enabled, truck.helper.short_increment, truck.helper.long_increment = True, 1, 5
        truck.run(86, 400)
        truck.step(86, ButtonType.accelCruise, True, gas=gas)
        truck.run(86, 9, gas=gas)
        truck.step(87, ButtonType.accelCruise, False, gas=gas)
        truck.run(87, 400)
        self.assertEqual(truck.target, 87)
        self.assertFalse(truck.decreased)

  def test_a_press_that_begins_after_the_seed_is_openpilots(self):
    # the display first shows 35 on frame 4; a Set- from frame 5 is counted by openpilot (the PCM refusing it here)
    for frames, want in ((9, 34),):
      with self.subTest(frames=frames):
        truck = Truck()
        truck.run(35, LATE + 1)
        truck.step(35, ButtonType.decelCruise, True)
        truck.run(35, frames)
        truck.step(35, ButtonType.decelCruise, False)
        truck.run(35, 400)
        self.assertEqual(truck.target, want)
        self.assertFalse(truck.increased)

  def test_the_last_frame_of_the_window_for_either_button_name(self):
    # pressed and released while openpilot is not engaged, engaged on the next frame, the display showing the result
    # on the window's last frame; Ford names the same physical Set- Set or Set- depending on the PCM at the press
    for button in (ButtonType.decelCruise, ButtonType.setCruise):
      with self.subTest(button=button):
        truck = Truck()
        truck.run(35, 5, late=0, enabled=False)
        truck.step(35, button, True, late=0, enabled=False)
        truck.run(35, 64, late=0, enabled=False)
        truck.step(35, button, False, late=0, enabled=False)
        truck.run(35, PCM_SETTLE_FRAMES - 1, late=0)
        truck.run(30, 400, late=0)
        self.assertEqual(truck.target, 30)
        self.assertFalse(truck.increased)

  def test_gas_held_taps_down_from_87(self):
    # each tap with the gas down; the current speed (35 mph) is below the target, so it sets no floor
    truck = Truck()
    truck.run(87, 400)
    for after, before in ((86, 87), (85, 86)):
      truck.step(before, ButtonType.decelCruise, True, gas=True)
      truck.run(before, 9, gas=True)
      truck.step(after, ButtonType.decelCruise, False, gas=True)
      truck.run(after, 40)
    truck.run(85, 400)
    self.assertEqual(truck.target, 85)
    self.assertFalse(truck.increased)

  def test_a_long_set_plus_stays_under_icbms_hold_line(self):
    # held Set+ from 65 mph for five long-press ticks, the PCM taking only one mph per tick (to 70): openpilot's target
    # stops at 89 (not 90, which ICBM would take as no target and hold), ICBM brings the PCM there, and a later Set-
    # is a step down from it with no press back up
    truck = Truck()
    truck.run(65, 500)
    truck.step(65, ButtonType.accelCruise, True)
    for mph in (65, 66, 67, 68, 69):
      truck.run(mph, 50)
    truck.step(70, ButtonType.accelCruise, False)
    self.assertEqual(truck.target, 89)
    shown = truck.run_pcm(70, 800)
    self.assertEqual(shown, 89)
    sent = len(truck.sent)
    truck.tap(ButtonType.decelCruise, 88, 89)
    shown = truck.run_pcm(88, 800)
    self.assertEqual((truck.target, shown), (88, 88))
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_fresh_press_once_the_target_has_a_value_is_openpilots(self):
    # a press from the unset window released as 35 first shows; a fresh Set- (the PCM refusing it here) is counted by
    # openpilot, as on the base, inside the window's 20 frames as well as after them, and ICBM applies it
    for idle, want in ((PCM_SETTLE_FRAMES - 1, 34), (PCM_SETTLE_FRAMES, 34), (PCM_SETTLE_FRAMES + 1, 34)):
      with self.subTest(idle=idle):
        truck = Truck()
        truck.step(35)
        truck.step(35, ButtonType.decelCruise, True)
        truck.step(35, ButtonType.decelCruise, False, late=2)
        truck.run(35, idle, late=0)
        truck.tap(ButtonType.decelCruise, 35, 35)
        truck.run(35, 400, late=0)
        self.assertEqual(truck.target, want)
        self.assertFalse(truck.increased)

  def test_cancel_and_main_do_not_extend_the_window(self):
    truck = Truck()
    truck.step(35)
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(35, ButtonType.decelCruise, False, late=2)
    truck.step(35, ButtonType.cancel, True, late=0)
    truck.run(35, PCM_SETTLE_FRAMES + 5, late=0)
    self.assertFalse(truck.helper.pcm_target)

  def test_a_limit_of_90_mph_or_more_still_holds_the_pcm(self):
    # this truck's map limits carry a 10% offset: an 85 mph limit becomes 93.5. As on the installed base, ICBM takes a
    # target of 90 mph or more as no target and holds the PCM; 88 is followed
    for limit, end in ((93.5, 35), (90, 35), (88, 88)):
      with self.subTest(limit=limit):
        truck = Truck()
        truck.run(35, 300)
        truck.limit = limit
        self.assertEqual(truck.run_pcm(35, 1200), end)
        self.assertEqual(truck.target, end)   # 90 or more is not written at all

  def test_set_plus_at_the_top_is_not_pressed_back(self):
    # a 90 mph PCM tapped up (it stays at 90), and 89 tapped up to 90: ICBM presses nothing back down
    for seed, after in ((90, 90), (89, 90)):
      with self.subTest(seed=seed):
        truck = Truck()
        truck.run(seed, 400)
        truck.tap(ButtonType.accelCruise, after, seed)
        self.assertEqual(truck.run_pcm(after, 600), after)
        self.assertFalse(truck.decreased)

  def test_the_window_ends_on_its_last_frame_across_the_engage(self):
    # pressed and released while not engaged, the window's last frame is the engage frame (still not effectively
    # engaged); a tap one frame later is openpilot's to count (the PCM refusing it here)
    truck = Truck()
    truck.step(35, late=0, enabled=False)
    truck.step(35, ButtonType.decelCruise, True, late=0, enabled=False)
    truck.step(35, ButtonType.decelCruise, False, late=0, enabled=False)
    truck.run(35, PCM_SETTLE_FRAMES - 1, late=0, enabled=False)
    truck.step(35, late=0)                              # engaged, effectively from the next frame
    truck.tap(ButtonType.decelCruise, 35, 35)
    truck.run(35, 400, late=0)
    self.assertEqual(truck.target, 34)

  def test_custom_increments_from_a_limit_are_as_before(self):
    # custom 5 mph taps from this truck's recorded 55:11 limit (49.5 mph): a Set- the PCM takes as 45 stays at 45
    truck = Truck()
    truck.helper.custom_acc_enabled, truck.helper.short_increment, truck.helper.long_increment = True, 5, 5
    truck.run(35, 100)
    truck.limit = 22.128480911254883 / CV.MPH_TO_MS
    truck.run(35, 300)
    shown = truck.run_pcm(35, 400)
    truck.tap(ButtonType.decelCruise, shown - 5, shown)
    sent = len(truck.sent)
    self.assertEqual(truck.run_pcm(shown - 5, 600), shown - 5)
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_set_minus_after_a_long_set_plus_near_the_top_presses_nothing_up(self):
    # an 88 mph limit (an 80 mph map limit with this truck's 10% offset) while the PCM is at 65; a Set+ tap, then a held
    # Set+ the PCM takes by only 1 or 5 mph: openpilot's target stops at 89 (not the hold) and ICBM brings the PCM up
    # after the release, in the driver's direction. A Set- the PCM takes is then a step down with no press back up.
    for manual in (1, 5):
      with self.subTest(manual=manual):
        truck = Truck()
        truck.limit = 65
        shown = truck.run_pcm(65, 800)
        truck.limit = 88
        shown = truck.run_pcm(shown, 25)
        truck.tap(ButtonType.accelCruise, shown + 1, shown)
        shown = truck.run_pcm(shown + 1, 30)
        truck.step(shown, ButtonType.accelCruise, True)
        truck.run(shown, 49)
        truck.run(shown + manual, 5)
        truck.step(shown + manual, ButtonType.accelCruise, False)
        self.assertEqual(truck.target, 89)
        shown = truck.run_pcm(shown + manual, 900)
        self.assertEqual(shown, 89)
        sent = len(truck.sent)
        truck.tap(ButtonType.decelCruise, shown - 1, shown)
        self.assertEqual((truck.run_pcm(shown - 1, 900), truck.target), (88, 88))
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_held_set_plus_to_90_with_a_late_display_is_not_pressed_back(self):
    # an 88 mph limit, the PCM at 80; Set+ held for two ticks, the PCM showing 85 and then 90 a frame (or three) after
    # each tick: it ends at 90 with nothing pressed back down
    for delay in (1, 3):
      with self.subTest(delay=delay):
        truck = Truck()
        truck.limit = 80
        truck.run_pcm(80, 800)
        truck.limit = 88
        truck.run(80, 100)
        truck.step(80, ButtonType.accelCruise, True)
        truck.run(80, 49 + delay)
        truck.run(85, 50)
        truck.run(90, 6 - delay)
        truck.step(90, ButtonType.accelCruise, False)
        self.assertEqual(truck.run_pcm(90, 600), 90)
        self.assertFalse(truck.decreased)

  def test_a_held_set_minus_after_a_resume_engage_goes_on_from_the_limit(self):
    # engaged with Resume (released on the engage frame), a Set- held from 0.12 s later, and Speed Limit Assist writing
    # 55 mph during the hold: the hold's later ticks count from 55 (to 35), as on the base. They were ignored for the
    # rest of the hold, and ICBM pressed the PCM back up to 55 after the release.
    for per in (1, 5):
      with self.subTest(per=per):
        truck = Truck()
        truck.run(55, 11, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, True, late=0, enabled=False)
        truck.run(55, 13, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, False, late=0)
        truck.run(55, 11, late=0)
        shown = 55
        truck.step(shown, ButtonType.decelCruise, True, late=0)
        for f in range(269):
          if f == 39:
            truck.limit = 55
          if f % 50 == 48:
            shown -= per
          truck.step(shown, late=0)
        truck.step(shown, ButtonType.decelCruise, False, late=0)
        self.assertEqual(truck.target, 35)
        self.assertEqual(truck.run_pcm(shown, 1200, late=0), 35)

  def test_a_held_set_minus_after_a_resume_engage_with_speed_limit_assist_deciding(self):
    # the real Speed Limit Assist decides when to write (posted 50 + 10%: 55). Engaged with Resume, a Set- held from
    # 0.12 s later, the PCM taking a mph 30 ms after the press and then 1 (or 5) per tick: Speed Limit Assist writes 55
    # during the hold and the rest of it counts from there, ending as on the base (35; a shorter hold, 50). The display
    # moving under the PCM-owned press restarted its activation guard, and its first write came after the release,
    # pressing the PCM back up to 55.
    for per, frames, want in ((1, 270, 35), (5, 270, 35), (1, 115, 50)):
      with self.subTest(per=per, frames=frames):
        truck = Truck(real_sla=True)
        truck.ego = 55 * CV.MPH_TO_MS
        truck.limit = 55
        truck.run(55, 11, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, True, late=0, enabled=False)
        truck.run(55, 13, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, False, late=0)
        truck.run(55, 11, late=0)
        shown = 55
        truck.step(shown, ButtonType.decelCruise, True, late=0)
        for f in range(frames - 1):
          if f == 2:
            shown -= 1
          elif f % 50 == 48:
            shown -= per
          truck.step(shown, late=0)
        truck.step(shown, ButtonType.decelCruise, False, late=0)
        self.assertEqual(truck.target, want)
        self.assertEqual(truck.run_pcm(shown, 1200, late=0), want)

  def test_a_press_just_after_speed_limit_assists_write_holds(self):
    # engaged at 66 under a 67.9 mph final limit; Speed Limit Assist writes it, and a held Set- ticks to 65 just after:
    # the driver's 65 holds. Speed Limit Assist read its own write as a manual change, went inactive for a sample,
    # came back on the next and wrote the limit again over the tick (68).
    truck = Truck(real_sla=True)
    truck.ego = 66 * CV.MPH_TO_MS
    truck.limit = 67.85
    truck.run(66, 20, late=0, enabled=False)
    truck.run(66, 10, late=0)
    truck.step(66, ButtonType.decelCruise, True, late=0)
    truck.run(66, 92, late=0)
    truck.step(66, ButtonType.decelCruise, False, late=0)
    self.assertEqual(truck.target, 65)
    self.assertEqual(truck.run_pcm(66, 1200, late=0), 65)

  def test_the_first_set_speed_keeps_speed_limit_assists_timing(self):
    # the target taking the PCM's set speed after an engage is not a manual change to Speed Limit Assist: its first
    # write comes 0.5 s after the engage either way, the display there before it (taken while disengaged) or arriving
    # 4 frames after (on the base the target stayed unset; here it restarted the 0.5 s guard, a sample later)
    for before in (True, False):
      with self.subTest(display_before_the_engage=before):
        truck = Truck(real_sla=True)
        truck.ego = 40 * CV.MPH_TO_MS
        truck.limit = 55
        if before:
          truck.run(40, 20, late=0, enabled=False)
        start, written = truck.frame, None
        for _ in range(200):
          truck.step(40, late=start + (0 if before else 4))
          if truck.target == 55:
            written = truck.frame - 1 - start
            break
        self.assertIsNotNone(written)
        self.assertLessEqual(written, 52)

  def test_a_tap_then_a_hold_after_a_resume_engage_presses_nothing_back(self):
    # 55 before the engage, a 60 mph posted limit (66 final) throughout. Resume released at the engage, a Set- tap at
    # +0.14..0.24 s (the PCM showing 54 from +0.17 s), then a Set- held from +0.29 s to +2.44 s, the PCM taking a mph per
    # repeat: nothing is pressed after the release (the target took the tap a frame late, which moved Speed Limit
    # Assist's first write past the hold's first repeat; ICBM then pressed the PCM back up)
    truck = Truck(real_sla=True)
    truck.ego = 55 * CV.MPH_TO_MS
    truck.limit = 66
    truck.run(55, 11, late=0, enabled=False)
    truck.step(55, ButtonType.resumeCruise, True, late=0, enabled=False)
    truck.run(55, 13, late=0, enabled=False)
    truck.step(55, ButtonType.resumeCruise, False, late=0)
    truck.run(55, 13, late=0)
    shown = 55
    truck.step(shown, ButtonType.decelCruise, True, late=0)
    for f in range(9):
      if f == 2:
        shown = 54
      truck.step(shown, late=0)
    truck.step(shown, ButtonType.decelCruise, False, late=0)
    truck.run(shown, 4, late=0)
    truck.step(shown, ButtonType.decelCruise, True, late=0)
    for f in range(214):
      if f % 50 == 48:
        shown -= 1
      truck.step(shown, late=0)
    truck.step(shown, ButtonType.decelCruise, False, late=0)
    sent = len(truck.sent)
    end = truck.run_pcm(shown, 1200, late=0)
    self.assertEqual(set(truck.sent[sent:]) - {SendButtonState.none, SendButtonState.decrease}, set())
    self.assertLessEqual(end, shown)

  def test_a_pcm_owned_hold_follows_the_pcm(self):
    # no limit; a Set- held from before the display shows (so the PCM's), the PCM taking a mph per repeat: the target
    # is the PCM's set speed all through the hold, so the planner follows the driver's reductions, and after it;
    # nothing is pressed after the release
    truck = Truck()
    truck.step(55, late=8)
    truck.step(55, ButtonType.decelCruise, True, late=8)
    shown = 55
    for f in range(269):
      if f % 50 == 46:
        shown -= 1
      truck.step(shown, late=8)
      if f > 10:
        self.assertEqual(truck.target, shown)
    truck.step(shown, ButtonType.decelCruise, False, late=8)
    self.assertEqual(truck.target, shown)
    sent = len(truck.sent)
    self.assertEqual(truck.run_pcm(shown, 600, late=8), shown)
    self.assertEqual(set(truck.sent[sent:]) - {SendButtonState.none}, set())

  def test_a_late_display_from_the_engage_press_does_not_move_the_limit(self):
    # 55 showing, posted 50 (55 final). A Set- from 10 ms before the engage to +0.22 s, which the PCM shows (54) only at
    # +0.30 s; then a Set- held from +0.32 s to +3.02 s, the PCM taking a mph per repeat. Speed Limit Assist writes 55
    # half a second after the engage, in every planner phase, and the hold counts from there to 35, as on the base.
    # (The 54 arriving late restarted its guard, each repeat restarted it again, and its write came after the release:
    # ICBM pressed the PCM from 49 back up to 55.)
    for phase in range(5):
      with self.subTest(phase=phase):
        truck = Truck(real_sla=True)
        truck.ego = 55 * CV.MPH_TO_MS
        truck.limit = 55
        truck.run(55, 19 + phase, late=0, enabled=False)
        truck.step(55, ButtonType.decelCruise, True, late=0, enabled=False)
        truck.run(55, 22, late=0)
        truck.step(55, ButtonType.decelCruise, False, late=0)
        truck.run(55, 7, late=0)
        shown = 54
        truck.run(shown, 2, late=0)
        truck.step(shown, ButtonType.decelCruise, True, late=0)
        for f in range(269):
          if f % 50 == 48:
            shown -= 1
          truck.step(shown, late=0)
        truck.step(shown, ButtonType.decelCruise, False, late=0)
        sent = len(truck.sent)
        self.assertEqual(truck.run_pcm(shown, 1500, late=0), 35)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_taps_after_a_late_display_count_from_the_limit(self):
    # the display 4 frames late at 57, posted 50 (55 final); Set- taps released at +0.22, +0.70 and +0.95 s, the PCM
    # taking each (54): Speed Limit Assist writes 55 at +0.5 s and the two later taps count from it (53), in every
    # planner phase, as on the base. (Each tap restarted its guard and the 55 came after them: one Set+ back to 55.)
    for phase in range(5):
      with self.subTest(phase=phase):
        truck = Truck(real_sla=True)
        truck.ego = 57 * CV.MPH_TO_MS
        truck.limit = 55
        late = phase + 4
        truck.run(57, phase, late=late, enabled=False)
        truck.run(57, 12, late=late)
        truck.tap(ButtonType.decelCruise, 56, 57)
        truck.run(56, 37, late=late)
        truck.tap(ButtonType.decelCruise, 55, 56)
        truck.run(55, 14, late=late)
        truck.tap(ButtonType.decelCruise, 54, 55)
        sent = len(truck.sent)
        self.assertEqual(truck.run_pcm(54, 1500, late=late), 53)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_hold_begun_before_the_first_display_lets_the_limit_in(self):
    # posted 50 (55 final) throughout, the display 40 ms late, a Set- held from +10 ms to +2.71 s with the PCM taking a
    # mph per repeat: Speed Limit Assist writes 55 during the hold and the rest of it counts from there (to 40), in
    # every planner phase, as on the base. (Each change of the target under the hold restarted Speed Limit Assist's
    # guard: it wrote after the release and ICBM pressed the PCM back up to 55.)
    for phase in range(5):
      with self.subTest(phase=phase):
        truck = Truck(real_sla=True)
        truck.ego = 55 * CV.MPH_TO_MS
        truck.limit = 55
        late = phase + 4
        truck.run(55, phase, late=late, enabled=False)
        truck.step(55, late=late)
        truck.step(55, ButtonType.decelCruise, True, late=late)
        shown = 55
        for f in range(269):
          if f % 50 == 48:
            shown -= 1
          truck.step(shown, late=late)
        truck.step(shown, ButtonType.decelCruise, False, late=late)
        self.assertEqual(truck.run_pcm(shown, 1500, late=late), 40)

  def test_a_lower_limit_arriving_on_a_set_minus_repeat_applies(self):
    # 62 before the engage under a 75 mph sign (82.5 final, written while the PCM is still at 62); a Set- held from
    # +0.59 s whose first repeat takes the target to 80, and a 50 mph sign (55 final) on that planner sample: the new,
    # lower limit is applied and ICBM brings the PCM down to 55. (The repeat read as a manual change, the sign was
    # marked as seen, and ICBM raised the PCM from 61 to 80. The base does that with a limit of exactly 82.5, which
    # its rounding matches to its own write; a hair under, it does not match, stays inactive and takes the sign.)
    for first in (82.5, 82.499):
      with self.subTest(first=first):
        truck = Truck(real_sla=True)
        truck.ego = 62 * CV.MPH_TO_MS
        truck.limit = first
        truck.run(62, 20, late=0, enabled=False)
        truck.run(62, 59, late=0)
        truck.step(62, ButtonType.decelCruise, True, late=0)
        truck.run(62, 48, late=0)
        truck.run(61, 2, late=0)
        truck.limit = 55
        truck.run(61, 9, late=0)
        truck.step(61, ButtonType.decelCruise, False, late=0)
        self.assertEqual((truck.run_pcm(61, 1500, late=0), truck.target), (55, 55))

  def test_a_tap_whose_display_comes_after_the_release_presses_nothing_back(self):
    # the tap-then-hold case with the PCM's 54 showing one frame after the tap's release (or after a 100 ms gap): the
    # tap is openpilot's, counted on its release frame as on the base, and nothing is pressed back after the hold
    for lag, gap in ((1, 4), (8, 9)):
      with self.subTest(lag=lag, gap=gap):
        truck = Truck(real_sla=True)
        truck.ego = 55 * CV.MPH_TO_MS
        truck.limit = 66
        truck.run(55, 11, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, True, late=0, enabled=False)
        truck.run(55, 13, late=0, enabled=False)
        truck.step(55, ButtonType.resumeCruise, False, late=0)
        truck.run(55, 13, late=0)
        truck.tap(ButtonType.decelCruise, 55, 55)
        truck.run(55, lag - 1, late=0) if lag > 1 else None
        shown = 54
        truck.run(shown, gap, late=0)
        truck.step(shown, ButtonType.decelCruise, True, late=0)
        for f in range(214):
          if f % 50 == 48:
            shown -= 1
          truck.step(shown, late=0)
        truck.step(shown, ButtonType.decelCruise, False, late=0)
        sent = len(truck.sent)
        truck.run_pcm(shown, 1200, late=0)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_an_unrelated_button_during_a_held_set_minus_changes_nothing(self):
    # Speed Limit Assist (posted 50, final 55) and a Set- held from +0.12 s, with the distance button pressed and
    # released during it: the same as without it (Speed Limit Assist writes 55 during the hold and the rest counts from
    # there to 35); its release was taken as the Set-'s, which moved the write past the hold
    ends = []
    for gap in (False, True):
      truck = Truck(real_sla=True)
      truck.ego = 55 * CV.MPH_TO_MS
      truck.limit = 55
      truck.run(55, 11, late=0, enabled=False)
      truck.step(55, ButtonType.resumeCruise, True, late=0, enabled=False)
      truck.run(55, 13, late=0, enabled=False)
      truck.step(55, ButtonType.resumeCruise, False, late=0)
      truck.run(55, 11, late=0)
      shown = 55
      truck.step(shown, ButtonType.decelCruise, True, late=0)
      for f in range(269):
        if f == 2:
          shown -= 1
        elif f % 50 == 48:
          shown -= 1
        if gap and f in (7, 22):
          truck.step(shown, ButtonType.gapAdjustCruise, f == 7, late=0)
        else:
          truck.step(shown, late=0)
      truck.step(shown, ButtonType.decelCruise, False, late=0)
      ends.append((truck.target, truck.run_pcm(shown, 1200, late=0)))
    self.assertEqual(ends[0], ends[1])
    self.assertEqual(ends[0], (35, 35))

  def test_a_limit_icbm_takes_as_no_target_holds_the_pcm_where_it_is(self):
    # 93.5 mph (an 85 mph sign plus this truck's 10% offset) with the PCM at 35 or 65: the target is the PCM's own set
    # speed and the PCM is held, as before. A Set- the PCM takes, a Set- then a Set+, or a held Set- then step from the
    # speed the driver sees, and ICBM presses nothing back up. (The limit was written as 145 km/h; one Set- made that
    # 89 mph and ICBM walked the PCM up to it.)
    for seed, presses in ((35, "tap"), (65, "tap"), (35, "pair"), (65, "hold")):
      with self.subTest(seed=seed, presses=presses):
        truck = Truck()
        truck.limit = seed
        truck.run_pcm(seed, 800)
        truck.limit = 93.5
        self.assertEqual((truck.run_pcm(seed, 800), truck.target), (seed, seed))
        sent, shown = len(truck.sent), seed
        if presses == "hold":
          truck.step(shown, ButtonType.decelCruise, True)
          for _ in range(3):
            truck.run(shown, 50)
            shown -= 1   # the PCM a mph per repeat; openpilot's own repeats go further and ICBM follows down
          truck.step(shown, ButtonType.decelCruise, False)
        else:
          truck.tap(ButtonType.decelCruise, shown - 1, shown)
          shown -= 1
          if presses == "pair":
            truck.run(shown, 15)
            truck.tap(ButtonType.accelCruise, shown + 1, shown)
            shown += 1
        end = truck.run_pcm(shown, 1200)
        self.assertLessEqual(end, shown)
        self.assertEqual(truck.target, end)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_limit_icbm_takes_as_no_target_stops_a_walk_to_an_earlier_one(self):
    # the real Speed Limit Assist. 50 under a posted 60 (66 final), a Set- tap at +0.3 s (49); the 66 is written at
    # +0.5 s and ICBM starts toward it; an 85 mph sign (93.5) at +0.85 s holds the PCM where it has got to, as on the
    # base, instead of letting the walk to 66 run on (17 Set+ after the driver's Set-)
    truck = Truck(real_sla=True)
    truck.ego = 50 * CV.MPH_TO_MS
    truck.limit = 66
    truck.run(50, 20, late=0, enabled=False)
    truck.run(50, 30, late=0)
    truck.tap(ButtonType.decelCruise, 49, 50)
    shown = truck.run_pcm(49, 44, late=0)
    truck.limit = 93.5
    held = truck.run_pcm(shown, 10, late=0)
    end = truck.run_pcm(held, 1500, late=0)
    self.assertLessEqual(end, 52)
    self.assertEqual((end, truck.target), (held, held))

  def test_a_tap_after_a_limit_icbm_takes_as_no_target_steps_from_the_pcm(self):
    # 66, then a posted 35 (38.5) at +5.0 s and, with the PCM still on its way down, an 85 mph sign at +5.5 s: the PCM
    # is held where it is, and a Set+ at +6.1 s is one step up from there with nothing pressed after it (the 38.5 was
    # left in force: the tap made it 40 and ICBM took the PCM down to it, 16 Set- after a Set+)
    truck = Truck(real_sla=True)
    truck.ego = 66 * CV.MPH_TO_MS
    truck.limit = 66
    truck.run(66, 20, late=0, enabled=False)
    shown = truck.run_pcm(66, 500, late=0)
    truck.limit = 38.5
    shown = truck.run_pcm(shown, 50, late=0)
    truck.limit = 93.5
    shown = truck.run_pcm(shown, 59, late=0)
    self.assertGreater(shown, 55)
    truck.tap(ButtonType.accelCruise, shown + 1, shown)
    sent = len(truck.sent)
    self.assertEqual((truck.run_pcm(shown + 1, 1200, late=0), truck.target), (shown + 1, shown + 1))
    self.assertEqual(set(truck.sent[sent:]) - {SendButtonState.none}, set())

  def test_a_half_mph_limit_shows_and_steps_as_before(self):
    # a 65 or 75 mph sign plus 10% is 71.5 / 82.5 mph: stored in tenths of a km/h it shows 72 / 83, and the limit
    # itself, as the plan's Float32, reads 71.49999 / 82.49999, which ICBM rounds to 71 / 82. Speed Limit Assist's
    # target is the stored value, so the PCM is brought to the speed shown and a Set+ is one step (the PCM sat a mph
    # under the set speed shown, and a Set+ moved it two)
    for limit, shows in ((71.4999963, 72), (82.4999963, 83)):
      with self.subTest(limit=limit):
        truck = Truck(real_sla=True)
        truck.ego = (shows - 6) * CV.MPH_TO_MS
        truck.limit = limit
        truck.run(shows - 6, 20, late=0, enabled=False)
        self.assertEqual((truck.run_pcm(shows - 6, 700, late=0), truck.target), (shows, shows))
        truck.tap(ButtonType.accelCruise, shows + 1, shows)
        sent = len(truck.sent)
        self.assertEqual(truck.run_pcm(shows + 1, 600, late=0), shows + 1)
        self.assertEqual(set(truck.sent[sent:]) - {SendButtonState.none}, set())

  def test_the_no_target_line_is_read_as_icbm_reads_it(self):
    # metric: a limit stored as 144.5 km/h reaches ICBM as 144.50000 (Float32 m/s), which it rounds to its 145 km/h
    # "no target"; it is treated as one (the PCM is held at 80 and a Set- is a step from there)
    truck = Truck(metric=True, real_sla=True)
    truck.ego = 80 * CV.KPH_TO_MS
    truck.limit = 144.51
    truck.run(80, 20, late=0, enabled=False)
    self.assertEqual((truck.run_pcm(80, 700, late=0), truck.target), (80, 80))
    truck.tap(ButtonType.decelCruise, 79, 80)
    self.assertEqual((truck.run_pcm(79, 1500, late=0), truck.target), (79, 79))
    self.assertFalse(truck.increased)

  def test_a_lower_limit_after_one_icbm_takes_as_no_target_applies(self):
    # the real Speed Limit Assist: an 85 mph sign (93.5) is not written, the 35 mph one after it (38.5) is
    truck = Truck(real_sla=True)
    truck.ego = 50 * CV.MPH_TO_MS
    truck.limit = 93.5
    truck.run(50, 20, late=0, enabled=False)
    self.assertEqual((truck.run_pcm(50, 400, late=0), truck.target), (50, 50))
    truck.limit = 38.5
    end = truck.run_pcm(50, 1500, late=0)
    self.assertEqual(truck.target, end)
    self.assertIn(end, (38, 39))

  def test_a_held_set_minus_from_a_driver_set_90_counts_down(self):
    # 66 under a posted 60 (66 final). Set+ held 3 s takes openpilot's target to the top while the PCM takes a mph per
    # repeat (72); then Set- held 4 s, the PCM a mph per repeat (64): openpilot's repeats count down from the top to 55,
    # as on the base, and ICBM follows down, also when the display reads nothing for three frames around a repeat.
    # (With presses left to the PCM under a 90+ target, the target stayed at 90 and a curve target, or the frame with
    # no display, sent the PCM up instead.)
    for dropout in (False, True):
      with self.subTest(dropout=dropout):
        truck = Truck()
        truck.limit = 66
        truck.run(66, 400)
        shown = 66
        truck.step(shown, ButtonType.accelCruise, True)
        for f in range(305):
          if f % 50 == 48:
            shown += 1
          truck.step(shown)
        truck.step(shown, ButtonType.accelCruise, False)
        self.assertGreaterEqual(truck.target, 90)
        truck.run(shown, 14)
        truck.step(shown, ButtonType.decelCruise, True)
        for f in range(405):
          if f % 50 == 48:
            shown -= 1
          truck.step(0 if dropout and 397 <= f <= 399 else shown)
        truck.step(shown, ButtonType.decelCruise, False)
        self.assertEqual(truck.target, 55)
        sent = len(truck.sent)
        self.assertEqual(truck.run_pcm(shown, 1500), 55)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_set_minus_at_the_top_with_the_pcm_there_is_openpilots(self):
    # the PCM itself at 90: a Set- it takes is 89 for openpilot too, as on the base
    truck = Truck()
    truck.run(90, 400)
    truck.tap(ButtonType.decelCruise, 89, 90)
    self.assertEqual((truck.run_pcm(89, 600), truck.target), (89, 89))
    self.assertFalse(truck.increased)

  def test_a_press_with_the_gas_down_keeps_openpilots_own_result(self):
    # Set- while overriding with the gas: openpilot sets the target to the current speed (15.6 m/s, 35 mph)
    truck = Truck()
    truck.run(30, 400)
    truck.step(30, ButtonType.decelCruise, True, gas=True)
    truck.run(30, 9, gas=True)
    truck.step(30, ButtonType.decelCruise, False, gas=True)
    self.assertEqual(truck.target, 35)

  def test_a_code_while_disengaged_shows_no_set_speed(self):
    truck = Truck()
    truck.step(254, late=0, enabled=False)
    self.assertEqual(truck.helper.v_cruise_cluster_kph, V_CRUISE_UNSET)

  def test_stock_set_speed_handling_is_unchanged(self):
    # the PCM owns the set speed (no ICBM): the target is the PCM's, unset while it reads 0
    truck = Truck(pcm_cruise_speed=True)
    truck.run(35, LATE)
    self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
    truck.run(35, 100)
    self.assertAlmostEqual(float(truck.helper.v_cruise_kph) * CV.KPH_TO_MPH, 35, places=1)

  def test_a_car_without_pcm_cruise_is_unchanged(self):
    # openpilot's own set speed: initialize_v_cruise gives it a value from the car's speed, not from a cluster
    truck = Truck(pcm_cruise=False)
    truck.run(70, 100)
    self.assertAlmostEqual(truck.helper.v_cruise_kph, round(15.6 * CV.MS_TO_KPH), places=3)


if __name__ == "__main__":
  unittest.main()
