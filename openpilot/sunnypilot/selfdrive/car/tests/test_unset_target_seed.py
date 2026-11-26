"""ICBM on a car whose PCM holds the set speed (Ford: pcmCruise, not pcmCruiseSpeed): an unset target takes the
cluster set speed once it shows.

Without that, the target stayed unset after an engage where the PCM's set speed arrived a few frames late, and with
no speed limit for Speed Limit Assist to write, two Set- taps counted down from the 145 km/h maximum: the target became
143 km/h (89 mph) and ICBM pressed Set+ toward it. The real VCruiseHelper and ICBM controller are stepped here at
100 Hz; the planner is stood in for by min(set speed, 145 km/h), as when cruise is its limiting source.
"""
import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MAX, V_CRUISE_UNSET
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
    self.frame = 0
    self.enabled_prev = False
    self.CS_prev = None
    self.sent = []

  def step(self, cluster_mph: float, button=None, pressed=False, late=LATE) -> None:
    """One 10 ms frame, engaged. The cluster set speed reads 0 for the first `late` frames."""
    shown = 0.0 if self.frame < late else cluster_mph * CV.MPH_TO_MS
    events = [{"type": button, "pressed": pressed}] if button is not None else []
    CS = car.CarState(vEgo=15.6, cruiseState={"available": True, "enabled": True, "speed": shown, "speedCluster": shown},
                      buttonEvents=events).as_reader()
    self.helper.update_speed_limit_assist(False, custom.LongitudinalPlanSP().as_reader())
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


class TestUnsetTargetSeed(unittest.TestCase):
  def test_a_late_cluster_speed_becomes_the_target(self):
    truck = Truck()
    truck.run(35, LATE)
    self.assertEqual(truck.helper.v_cruise_kph, V_CRUISE_UNSET)   # nothing to take yet
    truck.step(35)
    self.assertAlmostEqual(truck.target_mph, 35, places=1)
    truck.run(35, 400)
    self.assertAlmostEqual(truck.target_mph, 35, places=1)

  def test_set_minus_taps_lower_the_target_and_icbm_never_presses_up(self):
    truck = Truck()
    truck.run(35, 400)
    truck.tap(ButtonType.decelCruise, 34, 35)
    truck.run(34, 40)
    truck.tap(ButtonType.decelCruise, 33, 34)
    truck.run(33, 500)   # past the driver-press inhibit and the engage guard
    self.assertAlmostEqual(truck.target_mph, 33, places=1)
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
    self.assertAlmostEqual(truck.target_mph, 16, places=1)
    self.assertNotIn(SendButtonState.increase, truck.sent)
    self.assertNotIn(SendButtonState.decrease, truck.sent)

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
