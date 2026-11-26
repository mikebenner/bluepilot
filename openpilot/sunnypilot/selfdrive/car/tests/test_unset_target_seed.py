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

  def step(self, shown: float, button=None, pressed=False, late=LATE) -> None:
    """One 10 ms frame, engaged. The display reads 0 for the first `late` frames."""
    speed = 0.0 if self.frame < late else float(shown) * self.unit
    events = [{"type": button, "pressed": pressed}] if button is not None else []
    CS = car.CarState(vEgo=15.6, cruiseState={"available": True, "enabled": True, "speed": speed, "speedCluster": speed},
                      buttonEvents=events).as_reader()
    plan = custom.LongitudinalPlanSP()
    if self.limit:
      plan.speedLimit.resolver.speedLimitValid = True
      plan.speedLimit.resolver.speedLimitFinalLast = self.limit * self.unit
      plan.speedLimit.assist.state = "active"
    self.helper.update_speed_limit_assist(self.metric, plan.as_reader())
    self.helper.update_v_cruise(CS, True, self.metric)
    if not self.enabled_prev:
      self.helper.initialize_v_cruise(self.CS_prev if self.CS_prev is not None else CS, 0)
    self.enabled_prev, self.CS_prev = True, CS
    target = float(min(self.helper.v_cruise_kph, V_CRUISE_MAX)) * CV.KPH_TO_MS
    self.icbm.run(CS, car.CarControl(enabled=True).as_reader(), custom.LongitudinalPlanSP(vTarget=target).as_reader(), self.metric)
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
