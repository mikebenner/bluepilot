"""ICBM on a car whose PCM holds the set speed (Ford: pcmCruise, not pcmCruiseSpeed): while openpilot has no usable
target, the target is the PCM's own set speed (VCruiseHelper.pcm_owns_target).

Without that, the target stayed unset after an engage where the PCM's set speed arrived a few frames late, and with
no speed limit for Speed Limit Assist to write, two Set- taps counted down from the 145 km/h maximum: the target became
143 km/h (89 mph) and ICBM pressed Set+ toward it. The real VCruiseHelper and ICBM controller are stepped here at
100 Hz; the planner is stood in for by min(set speed, 145 km/h), as when cruise is its limiting source. Every scenario
checks the one property that matters: ICBM never presses opposite to what the driver asked for.
"""
import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
LATE = 4  # frames after the engage before the PCM's set speed shows


class Truck:
  def __init__(self, pcm_cruise=True, pcm_cruise_speed=False, op_long=True):
    self.CP = structs.CarParams(brand="ford", pcmCruise=pcm_cruise, openpilotLongitudinalControl=op_long).as_reader()
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed, intelligentCruiseButtonManagementAvailable=True).as_reader()
    self.helper = VCruiseHelper(self.CP, self.CP_SP)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP)
    self.helper.custom_acc_enabled = False   # ordinary 1 mph / 5 mph steps, whatever this machine's settings say
    # both start from the module's shared timer dictionary; a test that ends with a button held must not leak into
    # the next one
    self.helper.enable_button_timers = dict.fromkeys(CRUISE_BUTTON_TIMER, 0)
    self.icbm.cruise_button_timers = dict.fromkeys(CRUISE_BUTTON_TIMER, 0)
    self.frame = 0
    self.limit_mph = 0.   # Speed Limit Assist: active with this limit when non-zero
    self.enabled_prev = False
    self.CS_prev = None
    self.sent = []

  def step(self, cluster_mph: float, button=None, pressed=False, late=LATE) -> None:
    """One 10 ms frame, engaged. The cluster set speed reads 0 for the first `late` frames."""
    shown = 0.0 if self.frame < late else float(cluster_mph) * CV.MPH_TO_MS
    events = [{"type": button, "pressed": pressed}] if button is not None else []
    CS = car.CarState(vEgo=15.6, cruiseState={"available": True, "enabled": True, "speed": shown, "speedCluster": shown},
                      buttonEvents=events).as_reader()
    plan = custom.LongitudinalPlanSP()
    if self.limit_mph:
      plan.speedLimit.resolver.speedLimitValid = True
      plan.speedLimit.resolver.speedLimitFinalLast = self.limit_mph * CV.MPH_TO_MS
      plan.speedLimit.assist.state = "active"
    self.helper.update_speed_limit_assist(False, plan.as_reader())
    self.helper.update_v_cruise(CS, True, False)
    if not self.enabled_prev:
      self.helper.initialize_v_cruise(self.CS_prev if self.CS_prev is not None else CS, 0)
    self.enabled_prev, self.CS_prev = True, CS
    target = float(min(self.helper.v_cruise_kph, V_CRUISE_MAX)) * CV.KPH_TO_MS
    self.icbm.run(CS, car.CarControl(enabled=True).as_reader(), custom.LongitudinalPlanSP(vTarget=target).as_reader(), False)
    self.sent.append(self.icbm.cruise_button)
    self.frame += 1

  def run(self, cluster_mph: float, frames: int, **kwargs) -> None:
    for _ in range(frames):
      self.step(cluster_mph, **kwargs)

  def tap(self, button, cluster_after_mph: float, cluster_before_mph: float) -> None:
    self.step(cluster_before_mph, button, True)
    self.run(cluster_before_mph, 9)
    self.step(cluster_after_mph, button, False)   # the PCM moves its own set speed one step per tap

  @property
  def target_mph(self) -> float:
    return float(self.helper.v_cruise_kph) * CV.KPH_TO_MPH

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
    self.assertEqual(round(truck.target_mph), 35)
    truck.run(35, 400)
    self.assertEqual(round(truck.target_mph), 35)

  def test_set_minus_taps_lower_the_target_and_icbm_never_presses_up(self):
    truck = Truck()
    truck.run(35, 400)
    truck.tap(ButtonType.decelCruise, 34, 35)
    truck.run(34, 40)
    truck.tap(ButtonType.decelCruise, 33, 34)
    truck.run(33, 500)   # past the driver-press inhibit and the engage guard
    self.assertEqual(round(truck.target_mph), 33)
    self.assertEqual(truck.icbm.v_target, 33)
    self.assertNotIn(SendButtonState.increase, truck.sent)

  def test_a_target_that_has_a_value_is_left_alone(self):
    truck = Truck()
    truck.run(35, 100)
    truck.helper.v_cruise_kph = 80.5   # as Speed Limit Assist writes a limit, or the driver's own setting
    truck.run(35, 300)
    self.assertEqual(truck.helper.v_cruise_kph, 80.5)

  def test_the_cluster_speed_is_taken_as_the_pcm_shows_it(self):
    # 16 mph is below openpilot's 20 mph minimum but recorded on this truck's cluster: the target matches it, so ICBM
    # has nothing to press
    truck = Truck()
    truck.run(16, 400)
    self.assertEqual(round(truck.target_mph), 16)
    self.assertFalse(truck.increased or truck.decreased)

  def test_below_the_minimum_a_tap_never_lifts_the_target(self):
    # the PCM either takes the tap (16 -> 15 / 16 -> 17) or holds; the target follows it either way
    for button, after in ((ButtonType.decelCruise, 15), (ButtonType.decelCruise, 16), (ButtonType.accelCruise, 17)):
      with self.subTest(button=button, after=after):
        truck = Truck()
        truck.run(16, 400)
        truck.tap(button, after, 16)
        truck.run(after, 500)
        self.assertEqual(round(truck.target_mph), after)
        self.assertFalse(truck.increased or truck.decreased)

  def test_a_tap_released_before_the_cluster_shows(self):
    # press and release while the PCM still reads 0: the PCM applies the tap; the target takes its result
    truck = Truck()
    truck.step(35)                                    # engage
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(35, ButtonType.decelCruise, False)     # still before the cluster shows (LATE frames)
    truck.run(34, 400)                                # the PCM shows its result, 34
    self.assertEqual(round(truck.target_mph), 34)
    truck.tap(ButtonType.decelCruise, 33, 34)
    truck.run(33, 500)
    self.assertEqual(round(truck.target_mph), 33)
    self.assertFalse(truck.increased)

  def test_a_tap_released_on_the_frame_the_cluster_shows(self):
    truck = Truck()
    truck.run(35, LATE - 1)
    truck.step(35, ButtonType.decelCruise, True)
    truck.step(34, ButtonType.decelCruise, False)     # the first non-zero reading is the tap's result
    truck.run(34, 500)
    self.assertEqual(round(truck.target_mph), 34)    # counted once
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
    self.assertEqual(round(truck.target_mph), 30)
    self.assertFalse(truck.increased)

  def test_unknown_and_fault_codes_are_not_a_set_speed(self):
    # the display signal's 254 (Unknown) and 255 (Fault), decoded as mph
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.run(code, 20, late=0)
        self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
        truck.run(35, 400, late=0)
        self.assertEqual(round(truck.target_mph), 35)
        truck.tap(ButtonType.decelCruise, 34, 35)
        truck.run(34, 40)
        truck.tap(ButtonType.decelCruise, 33, 34)
        truck.run(33, 500)
        self.assertEqual(round(truck.target_mph), 33)
        self.assertFalse(truck.increased)

  def test_a_code_shown_through_the_engage_is_not_a_set_speed(self):
    # while openpilot is not engaged the PCM mirror copies whatever the PCM shows, code included; on the first
    # engaged frame that is no target, and the first real reading becomes one
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.run(code, 10, late=0)
        self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
        truck.run(35, 400, late=0)
        self.assertEqual(round(truck.target_mph), 35)
        self.assertFalse(truck.increased or truck.decreased)

  def test_a_long_press_from_the_seed_goes_down_in_5_mph_steps(self):
    truck = Truck()
    truck.run(35, 400)
    truck.step(35, ButtonType.decelCruise, True)
    truck.run(35, 100)                                # two long-press ticks: 35 -> 34.8 (no visible change), -> 30
    self.assertEqual(round(truck.target_mph), 30)
    truck.step(30, ButtonType.decelCruise, False)
    truck.run(30, 300)
    self.assertFalse(truck.increased)

  def test_a_fault_reading_after_the_seed_presses_nothing(self):
    for code in (254, 255):
      with self.subTest(code=code):
        truck = Truck()
        truck.run(35, 400)
        sent = len(truck.sent)
        truck.run(code, 120, late=0)
        self.assertEqual(round(truck.target_mph), 35)
        self.assertNotIn(SendButtonState.increase, truck.sent[sent:])
        self.assertNotIn(SendButtonState.decrease, truck.sent[sent:])
        truck.run(35, 200, late=0)                    # the display recovers
        self.assertEqual(round(truck.target_mph), 35)
        self.assertFalse(truck.increased or truck.decreased)

  def test_a_seed_reads_back_as_the_same_mph_across_the_range(self):
    # ICBM rounds the target to whole mph: a seed must come back as the PCM's own number, up to the top
    for mph in (16, 20, 35, 70, 85, 86, 87, 88, 89, 90):
      with self.subTest(mph=mph):
        truck = Truck()
        truck.run(mph, 500)
        self.assertEqual(truck.icbm.v_target, mph)
        self.assertFalse(truck.increased or truck.decreased)

  def test_the_display_updating_a_frame_after_the_release(self):
    # the PCM's button and its displayed set speed come in different messages
    truck = Truck()
    truck.step(35)
    truck.step(35, ButtonType.decelCruise, True)
    truck.run(35, 8)
    truck.step(35, ButtonType.decelCruise, False)     # released; the display still shows 35
    truck.run(34, 500)                                # and shows 34 from the next frame
    self.assertEqual(round(truck.target_mph), 34)
    self.assertFalse(truck.increased)

  def test_a_limit_written_during_a_held_press_stays(self):
    # Speed Limit Assist becomes active while a press from the PCM-owned window is held: its limit is the target, and
    # the press is not counted on top of it at the release
    for limit in (38, 55):
      with self.subTest(limit=limit):
        truck = Truck()
        truck.step(16)
        truck.step(16, ButtonType.accelCruise, True)
        truck.run(16, 20)
        truck.limit_mph = limit
        truck.run(17, 30)
        truck.step(17, ButtonType.accelCruise, False)
        truck.run(17, 300)
        self.assertEqual(round(truck.target_mph), limit)

  def test_down_across_the_minimum_from_above(self):
    # 21 -> 20 -> 19 (or the PCM refusing at 20): the target follows; ICBM never presses Set+
    for last in (19, 20):
      with self.subTest(last=last):
        truck = Truck()
        truck.run(21, 400)
        truck.tap(ButtonType.decelCruise, 20, 21)
        truck.run(20, 500)
        truck.tap(ButtonType.decelCruise, last, 20)
        truck.run(last, 500)
        self.assertEqual(round(truck.target_mph), last)
        self.assertFalse(truck.increased)

  def test_stock_set_speed_handling_is_unchanged(self):
    # the PCM owns the set speed (no ICBM): the target is the PCM's, unset while it reads 0
    truck = Truck(pcm_cruise_speed=True)
    truck.run(35, LATE)
    self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)
    truck.run(35, 100)
    self.assertAlmostEqual(truck.target_mph, 35, places=1)

  def test_a_car_without_pcm_cruise_is_unchanged(self):
    # openpilot's own set speed: initialize_v_cruise gives it a value from the car's speed, not from a cluster
    truck = Truck(pcm_cruise=False)
    truck.run(70, 100)
    self.assertAlmostEqual(truck.helper.v_cruise_kph, round(15.6 * CV.MS_TO_KPH), places=3)


if __name__ == "__main__":
  unittest.main()
