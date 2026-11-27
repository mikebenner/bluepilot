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

ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
LATE = 4  # frames after the engage before the PCM's set speed shows


class Truck:
  """Display values (`shown`) are in the display unit: mph, or km/h with metric=True."""
  def __init__(self, pcm_cruise=True, pcm_cruise_speed=False, op_long=True, metric=False):
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

  def step(self, shown: float, button=None, pressed=False, late=LATE, enabled=True, unit=None, gas=False) -> None:
    """One 10 ms frame, openpilot engaged unless enabled=False (the PCM's cruise stays on). The display reads 0 for the
    first `late` frames; `unit` overrides the display unit (a car whose display unit differs from openpilot's)."""
    speed = 0.0 if self.frame < late else float(shown) * (unit or self.unit)
    events = [{"type": button, "pressed": pressed}] if button is not None else []
    CS = car.CarState(vEgo=self.ego, gasPressed=gas, buttonEvents=events,
                      cruiseState={"available": True, "enabled": True, "speed": speed, "speedCluster": speed}).as_reader()
    plan = custom.LongitudinalPlanSP()
    if self.limit:
      plan.speedLimit.resolver.speedLimitValid = True
      plan.speedLimit.resolver.speedLimitFinalLast = self.limit * self.unit
      plan.speedLimit.assist.state = "active"
    self.helper.update_speed_limit_assist(self.metric, plan.as_reader())
    self.helper.update_v_cruise(CS, enabled, self.metric)
    if enabled and not self.enabled_prev:
      self.helper.initialize_v_cruise(self.CS_prev if self.CS_prev is not None else CS, 0)
    self.enabled_prev, self.CS_prev = enabled, CS
    target = float(min(self.helper.v_cruise_kph, V_CRUISE_MAX)) * CV.KPH_TO_MS
    self.icbm.run(CS, car.CarControl(enabled=enabled).as_reader(), custom.LongitudinalPlanSP(vTarget=target).as_reader(), self.metric)
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

  def test_a_long_press_from_a_seed_steps_5_mph_from_its_first_tick(self):
    truck = Truck()
    truck.run(35, 400)
    truck.step(35, ButtonType.decelCruise, True)
    truck.run(35, 50)                                 # the first long-press tick
    self.assertEqual(truck.target, 30)
    truck.run(35, 50)                                 # the second
    self.assertEqual(truck.target, 25)

  def test_a_long_press_released_between_ticks(self):
    # released after the first tick: the target is the step it took; ICBM never presses back up
    for pcm in (34, 30):
      with self.subTest(pcm=pcm):
        truck = Truck()
        truck.run(35, 400)
        truck.step(35, ButtonType.decelCruise, True)
        truck.run(35, 65)
        truck.step(pcm, ButtonType.decelCruise, False)
        truck.run(pcm, 400)
        self.assertEqual(truck.target, 30)
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
    # the press is not counted on top of it at the release
    for limit in (38, 55):
      with self.subTest(limit=limit):
        truck = Truck()
        truck.step(20)
        truck.step(20, ButtonType.accelCruise, True)
        truck.run(20, 20)
        truck.limit = limit
        truck.run(21, 30)
        truck.step(21, ButtonType.accelCruise, False)
        truck.run(21, 300)
        self.assertEqual(truck.target, limit)

  def test_a_limit_equal_to_the_display_ends_the_ownership(self):
    # metric: a 90 km/h display and a 90 km/h limit give the same target value; the write itself hands it back
    truck = Truck(metric=True)
    truck.step(90)
    truck.step(90, ButtonType.accelCruise, True)
    truck.run(90, 28)
    truck.limit = 90
    truck.run(90, 2)
    truck.run(95, 8)
    truck.step(95, ButtonType.accelCruise, False)
    truck.run(95, 320)
    self.assertEqual(truck.target, 90)

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
    # and a long press released between ticks still steps a whole 5 mph
    truck = Truck()
    truck.run(35, 10, late=0, enabled=False)
    truck.run(35, 400, late=0)
    truck.step(35, ButtonType.decelCruise, True, late=0)
    truck.run(35, 65, late=0)
    truck.step(34, ButtonType.decelCruise, False, late=0)
    truck.run(34, 400, late=0)
    self.assertEqual(truck.target, 30)
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
    truck = Truck()
    truck.run(35, 60, late=60)                        # no display yet
    truck.limit = 45
    truck.step(35, ButtonType.decelCruise, True, late=60)
    truck.run(35, 3, late=60)
    truck.run(35, 6, late=0)
    truck.step(35, ButtonType.decelCruise, False, late=0)
    truck.run(35, 300, late=0)
    self.assertEqual(truck.target, 45)

  def test_a_second_tap_within_the_settle_window_is_the_pcms(self):
    # a press that began before the target existed, then another one: inside the 0.2 s after the first release the
    # PCM applies it and the target follows the PCM (taken or refused); after it, openpilot counts it as usual
    for gap, pcm_after, want in ((PCM_SETTLE_FRAMES - 1, 34, 34), (PCM_SETTLE_FRAMES - 1, 33, 33), (PCM_SETTLE_FRAMES + 5, 33, 33)):
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
    # has fired (five mph); ICBM never presses against the driver
    for button, sign in ((ButtonType.accelCruise, 1), (ButtonType.decelCruise, -1)):
      for frames_held, want in ((48, 35 + sign), (49, 35 + sign), (50, 35 + 5 * sign)):
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

  def test_a_long_press_from_a_limit_moves_a_whole_5_mph(self):
    # targets Speed Limit Assist wrote, from this truck's recorded limits (54:5: 44 mph, 55:11: 50 mph): off openpilot's
    # 8 km/h long-press grid; a long press still moves the shown mph to the next multiple of 5 and ICBM follows it
    for limit_ms, tap, hold, pcm, want in ((19.66975975036621, ButtonType.accelCruise, ButtonType.decelCruise, 40, 40),
                                           (22.128480911254883, None, ButtonType.accelCruise, 55, 55)):
      with self.subTest(limit_ms=limit_ms):
        truck = Truck()
        truck.run(35, 100)
        truck.limit = limit_ms / CV.MPH_TO_MS
        truck.run(35, 300)
        start = truck.target
        if tap is not None:
          truck.tap(tap, start + 1, start)
          truck.run(start + 1, 100)
          start += 1
        truck.step(start, hold, True)
        truck.run(start, 65)
        truck.step(pcm, hold, False)
        sent = len(truck.sent)
        truck.run(pcm, 400)
        self.assertEqual(truck.target, want)
        self.assertNotIn(SendButtonState.increase if hold == ButtonType.decelCruise else SendButtonState.decrease, truck.sent[sent:])

  def test_a_press_that_begins_after_the_seed_is_openpilots(self):
    # the display first shows 35 on frame 4; a Set- from frame 5 is counted by openpilot (the PCM refusing it here)
    for frames, want in ((65, 30), (9, 34)):
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

  def test_taps_from_a_limit_move_a_whole_mph(self):
    # this truck's recorded 58:1 limit (17.211 m/s) is written as 62.0 km/h, shown 39 mph; five Set+ taps and two
    # gas-held Set- taps each move the shown speed one mph, the PCM taking each, and ICBM never presses back
    truck = Truck()
    truck.run(35, 100)
    truck.limit = 17.211040496826172 / CV.MPH_TO_MS
    truck.run(35, 300)
    shown = truck.target
    for _ in range(5):
      truck.tap(ButtonType.accelCruise, shown + 1, shown)
      truck.run(shown + 1, 150)
      shown += 1
      self.assertEqual(truck.target, shown)
    for _ in range(2):
      truck.step(shown, ButtonType.decelCruise, True, gas=True)
      truck.run(shown, 9, gas=True)
      truck.step(shown - 1, ButtonType.decelCruise, False, gas=True)
      truck.run(shown - 1, 150)
      shown -= 1
      self.assertEqual(truck.target, shown)
    self.assertNotIn(SendButtonState.decrease, truck.sent[400:400 + 5 * 160])

  def test_a_press_just_after_the_settle_window_is_openpilots(self):
    # a press from the unset window released as 35 first shows; a fresh Set- after the window's 20 frames is counted
    # by openpilot (the PCM refusing it here), one inside it is the PCM's
    for idle, want in ((PCM_SETTLE_FRAMES - 1, 35), (PCM_SETTLE_FRAMES, 34), (PCM_SETTLE_FRAMES + 1, 34)):
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

  def test_a_held_set_plus_keeps_a_high_limit_hold(self):
    # a map limit with this truck's 10% offset (93.5 mph) holds the PCM at 35. A held Set+ (one long-press tick) that
    # the PCM takes to 40 (or 36) keeps the hold, as on the base: no automated increases. With the PCM already at 90,
    # a held Set+ is not pressed back to 89.
    for seed, after in ((35, 40), (35, 36), (90, 90)):
      with self.subTest(seed=seed, after=after):
        truck = Truck()
        truck.run(seed, 300)
        truck.limit = 93.5
        self.assertEqual(truck.run_pcm(seed, 800), seed)
        sent = len(truck.sent)
        truck.step(seed, ButtonType.accelCruise, True)
        truck.run(seed, 54)
        truck.step(after, ButtonType.accelCruise, False)
        self.assertEqual(truck.run_pcm(after, 1000), after)
        self.assertGreaterEqual(truck.target, 90)
        self.assertEqual(set(truck.sent[sent:]) - {SendButtonState.none}, set())

  def test_a_long_set_plus_from_89_reaches_the_hold(self):
    # from 89 with the PCM at 89, a held Set+ is 90 and ICBM's hold: the PCM at 90 from the driver's own hold stays
    truck = Truck()
    truck.run(89, 400)
    truck.step(89, ButtonType.accelCruise, True)
    truck.run(89, 54)
    truck.step(90, ButtonType.accelCruise, False)
    self.assertEqual(truck.run_pcm(90, 600), 90)
    self.assertFalse(truck.decreased)

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

  def test_a_long_set_plus_from_a_recorded_limit_into_the_top(self):
    # this truck's recorded 55:11 limit (49.5 mph); Set+ held for nine ticks, the PCM taking one mph per tick: the
    # target stops at 89, the PCM follows, and a Set- is a step down with no press back up
    truck = Truck()
    truck.run(50, 100)
    truck.limit = 22.128480911254883 / CV.MPH_TO_MS
    shown = truck.run_pcm(50, 400)
    truck.step(shown, ButtonType.accelCruise, True)
    for _ in range(9):
      truck.run(shown, 50)
      shown += 1
    truck.step(shown, ButtonType.accelCruise, False)
    self.assertEqual(truck.target, 89)
    shown = truck.run_pcm(shown, 900)
    sent = len(truck.sent)
    truck.tap(ButtonType.decelCruise, shown - 1, shown)
    self.assertEqual(truck.run_pcm(shown - 1, 900), 88)
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

  def test_a_tap_after_the_gas_floor_moves_a_whole_mph(self):
    # Set- with the gas down at 70.0 km/h (43.5 mph) from 40: the target is the 43 the PCM shows. With the gas released,
    # a Set- the PCM takes to 42 stays at 42 (it was 68.4 km/h, still 43, and ICBM pressed the PCM back)
    truck = Truck()
    truck.run(40, 400)
    truck.ego = 70.0 / 3.6
    truck.step(40, ButtonType.decelCruise, True, gas=True)
    truck.run(40, 9, gas=True)
    truck.step(43, ButtonType.decelCruise, False, gas=True)
    truck.run(43, 150, gas=True)
    self.assertEqual(truck.target, 43)
    self.assertAlmostEqual(float(truck.helper.v_cruise_kph), 70.0, places=3)   # the floor at the current speed, as written
    truck.run(43, 150)
    sent = len(truck.sent)
    truck.tap(ButtonType.decelCruise, 42, 43)
    self.assertEqual((truck.run_pcm(42, 600), truck.target), (42, 42))
    self.assertNotIn(SendButtonState.increase, truck.sent[sent:])

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
