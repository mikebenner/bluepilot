"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Upstream's tests for the predictive accel boost (sunnypilot/sunnypilot#2089), adapted to the backport:
update() takes only the model's acceleration, and the boost is off until the param turns it on.
"""
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, ACCEL_BOOST_MAX


def make_sm(brake_prob=0.05, gas_prob=0.3, lat_accel=0.0, v_ego=30.0, personality=0, enabled=True, should_stop=False):
  meta = SimpleNamespace(disengagePredictions=SimpleNamespace(brakePressProbs=[0.0, brake_prob, 0.0, 0.0, 0.0, 0.0],
                                                              gasPressProbs=[0.0, gas_prob, 0.0, 0.0, 0.0, 0.0]))
  action = SimpleNamespace(shouldStop=should_stop, desiredCurvature=lat_accel / v_ego ** 2)
  return {'modelV2': SimpleNamespace(meta=meta, action=action), 'carState': SimpleNamespace(vEgo=v_ego),
          'selfdriveState': SimpleNamespace(enabled=enabled, personality=personality)}


def on():
  boost = AccelBoostSP()
  boost.enabled = True
  return boost


def settle(sm, frames=100, a_e2e=0.0):
  boost = on()
  for _ in range(frames):
    boost.update(sm, a_e2e)
  return boost.total_boost


class TestAccelBoostSP:
  def teardown_method(self, method):
    Params().remove("AccelBoostSP")

  @pytest.mark.parametrize("personality", [0, 1, 2])
  def test_personality_max(self, personality):
    assert abs(settle(make_sm(personality=personality)) - ACCEL_BOOST_MAX[personality]) < 1e-6

  def test_brake_gate_closes(self):
    assert settle(make_sm(brake_prob=0.5)) == 0.0

  def test_gas_reopens(self):
    assert abs(settle(make_sm(brake_prob=0.5, gas_prob=0.9)) - ACCEL_BOOST_MAX[0]) < 1e-6

  def test_should_stop(self):
    assert settle(make_sm(gas_prob=0.9, should_stop=True)) == 0.0

  @pytest.mark.parametrize("lat_accel, scale", [(1.5, 1.0), (2.0, 0.5), (2.5, 0.0), (-2.0, 0.5)])
  def test_curve_fade(self, lat_accel, scale):
    assert abs(settle(make_sm(lat_accel=lat_accel)) - scale * ACCEL_BOOST_MAX[0]) < 1e-6

  def test_low_speed(self):
    assert settle(make_sm(v_ego=0.5)) == 0.0

  @pytest.mark.parametrize("a_e2e, scale", [(-0.7, 0.0), (-1.5, 0.0), (-0.45, 0.5), (-0.2, 1.0), (1.0, 1.0)])
  def test_fades_out_while_the_model_is_slowing(self, a_e2e, scale):
    assert abs(settle(make_sm(), a_e2e=a_e2e) - scale * ACCEL_BOOST_MAX[0]) < 1e-6

  def test_disabled_resets(self):
    boost = on()
    for _ in range(100):
      boost.update(make_sm(), 0.0)
    assert boost.update(make_sm(enabled=False), 0.0) == 0.0
    assert boost.boost == 0.0 and boost.total_boost == 0.0

  def test_build_and_release_rates(self):
    boost = on()
    boost.update(make_sm(), 0.0)
    assert abs(boost.total_boost - 0.01) < 1e-6
    for _ in range(100):
      boost.update(make_sm(), 0.0)
    boost.update(make_sm(brake_prob=0.5), 0.0)
    assert abs(boost.total_boost - (ACCEL_BOOST_MAX[0] - 0.05)) < 1e-6

  def test_missing_model_predictions_give_no_boost(self):
    sm = make_sm()
    sm['modelV2'].meta.disengagePredictions.gasPressProbs = []
    assert settle(sm) == 0.0

  # backport-specific: off by default, and off means the model's value passes through untouched
  @pytest.mark.parametrize("a_e2e", [-2.0, 0.0, 0.7])
  def test_off_is_a_passthrough(self, a_e2e):
    boost = AccelBoostSP()
    assert not boost.enabled
    for _ in range(100):
      assert boost.update(make_sm(gas_prob=0.9), a_e2e) == a_e2e
    assert boost.total_boost == 0.0

  def test_param_switch(self):
    params = Params()
    boost = AccelBoostSP()
    assert not boost.update_params() and not boost.enabled      # default off
    params.put_bool("AccelBoostSP", True, block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL) - 1):
      assert not boost.update_params()
    assert boost.update_params() and boost.enabled
    for _ in range(100):
      boost.update(make_sm(), 0.0)
    params.put_bool("AccelBoostSP", False, block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL) - 1):
      assert not boost.update_params()
    assert boost.update_params() and not boost.enabled
    assert boost.boost == 0.0 and boost.total_boost == 0.0
