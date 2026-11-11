"""Launch bias: only ever adds to a plan that already wants to go, only for the first few mph."""
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from bluepilot.selfdrive.controls.launch_bias import LaunchBias, LAUNCH_BIAS_LEVELS, level_to_accel, BUILD

MPH = 0.44704


def make_sm(v_mph=0.0, enabled=True, model_stop=False, lead=None):
  lead = lead or dict(status=False, dRel=0.0, vRel=0.0, vLead=0.0)
  return {'carState': SimpleNamespace(vEgo=v_mph * MPH), 'selfdriveState': SimpleNamespace(enabled=enabled),
          'modelV2': SimpleNamespace(action=SimpleNamespace(shouldStop=model_stop)),
          'radarState': SimpleNamespace(leadOne=SimpleNamespace(**lead))}


def on(level=2):
  lb = LaunchBias()
  lb.amount = level_to_accel(level)
  return lb


def settle(lb, sm, a_target=0.6, should_stop=False, frames=40):
  out = a_target
  for _ in range(frames):
    out = lb.update(sm, a_target, should_stop)
  return out - a_target


LEAD_AWAY = dict(status=True, dRel=12.0, vRel=1.0, vLead=1.5)


class TestLaunchBias:
  def teardown_method(self, method):
    Params().remove("BPLaunchBias")

  def test_off_by_default_and_a_passthrough(self):
    lb = LaunchBias()
    lb.update_params()
    assert lb.amount == 0.0
    for a in (-1.0, 0.0, 0.7):
      assert lb.update(make_sm(), a, False) == a

  @pytest.mark.parametrize("level", [1, 2, 3])
  def test_full_bias_from_a_stop_with_a_clear_road(self, level):
    assert settle(on(level), make_sm(v_mph=0.0)) == pytest.approx(LAUNCH_BIAS_LEVELS[level])

  @pytest.mark.parametrize("v_mph, frac", [(0, 1.0), (2, 1.0), (3.5, 0.53), (5, 0.0), (10, 0.0), (40, 0.0)])
  def test_fades_out_by_5_mph(self, v_mph, frac):
    assert settle(on(3), make_sm(v_mph=v_mph)) == pytest.approx(frac * LAUNCH_BIAS_LEVELS[3], abs=0.02)

  @pytest.mark.parametrize("a_target", [-2.0, -0.3, 0.0, 0.05])
  def test_never_added_unless_the_plan_already_wants_to_go(self, a_target):
    assert settle(on(3), make_sm(), a_target=a_target) == 0.0

  def test_never_added_when_the_planner_or_the_model_says_stop(self):
    assert settle(on(3), make_sm(), should_stop=True) == 0.0
    assert settle(on(3), make_sm(model_stop=True)) == 0.0

  def test_not_added_when_disengaged(self):
    assert settle(on(3), make_sm(enabled=False)) == 0.0

  def test_lead_pulling_away_with_room_gets_full_bias(self):
    assert settle(on(2), make_sm(lead=LEAD_AWAY)) == pytest.approx(LAUNCH_BIAS_LEVELS[2])

  @pytest.mark.parametrize("lead", [
    dict(status=True, dRel=12.0, vRel=0.0, vLead=0.0),     # lead still stopped
    dict(status=True, dRel=12.0, vRel=-1.0, vLead=1.0),    # closing on the lead
    dict(status=True, dRel=4.0, vRel=1.0, vLead=1.5),      # too close
  ])
  def test_no_bias_into_a_lead(self, lead):
    assert settle(on(3), make_sm(lead=lead)) == 0.0

  def test_bias_scales_with_gap(self):
    lead = dict(LEAD_AWAY, dRel=6.5)
    assert settle(on(2), make_sm(lead=lead)) == pytest.approx(0.5 * LAUNCH_BIAS_LEVELS[2])

  def test_builds_gradually_and_drops_at_once_on_stop(self):
    lb = on(3)
    first = lb.update(make_sm(), 0.6, False) - 0.6
    assert first == pytest.approx(BUILD * DT_MDL)
    settle(lb, make_sm())
    assert lb.update(make_sm(), 0.6, True) == 0.6 and lb.bias == 0.0

  def test_bad_or_out_of_range_levels_are_safe(self):
    assert level_to_accel(None) == 0.0 and level_to_accel("x") == 0.0
    assert level_to_accel(-3) == 0.0 and level_to_accel(99) == LAUNCH_BIAS_LEVELS[-1]

  def test_reads_the_setting(self):
    params = Params()
    params.put("BPLaunchBias", 2, block=True)
    lb = LaunchBias()
    lb.update_params()
    assert lb.amount == LAUNCH_BIAS_LEVELS[2]
    params.put("BPLaunchBias", 0, block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL)):
      lb.update_params()
    assert lb.amount == 0.0
