"""Stop/go debounce: only a brief return to "stop" right after a "go" from standstill is held back."""
from itertools import pairwise

import pytest

from openpilot.common.realtime import DT_MDL
from bluepilot.selfdrive.controls.go_debounce import GoDebounce, STOP_CONFIRM_TIME

N = round(STOP_CONFIRM_TIME / DT_MDL)   # planner frames a stop must persist


def run(seq, v=0.0, a=0.09):
  d = GoDebounce()
  return [d.update(s, v, a) for s in seq]


class TestGoDebounce:
  def test_holding_at_a_stop_is_untouched(self):
    assert run([True] * 50) == [True] * 50

  def test_go_is_never_delayed(self):
    assert run([True] * 10 + [False])[-1] is False

  @pytest.mark.parametrize("flicker", [1, 2, N - 1])
  def test_brief_stop_after_go_is_ignored(self, flicker):
    out = run([True] * 10 + [False] * 3 + [True] * flicker + [False] * 5)
    assert out[10:] == [False] * (3 + flicker + 5)

  def test_stop_that_persists_is_honoured_after_the_confirm_time(self):
    out = run([True] * 10 + [False] * 3 + [True] * 20)
    assert out[13:13 + N - 1] == [False] * (N - 1)
    assert all(out[13 + N - 1:])

  def test_flapping_like_the_recorded_launch_becomes_one_go(self):
    # planner frames: go 3, stop 1, go 2, stop 1, go 1, stop 1, go 2, stop 2, go ...
    seq = [True] * 10 + [False] * 3 + [True] + [False] * 2 + [True] + [False] + [True] + [False] * 2 + [True] * 2 + [False] * 10
    out = run(seq)
    assert sum(1 for a, b in pairwise(seq) if a != b) == 9
    assert sum(1 for a, b in pairwise(out) if a != b) == 1

  def test_stop_with_a_real_braking_request_is_immediate(self):
    d = GoDebounce()
    for s in [True] * 5 + [False] * 3:
      d.update(s, 0.0, 0.09)
    assert d.update(True, 0.0, -1.0) is True

  def test_stop_while_moving_is_immediate(self):
    d = GoDebounce()
    for s in [True] * 5 + [False] * 3:
      d.update(s, 0.0, 0.09)
    assert d.update(True, 1.5, 0.09) is True

  def test_ordinary_arrival_at_a_stop_is_immediate(self):
    # driving along (never "go from standstill"), slowing to a halt: the stop is not delayed
    d = GoDebounce()
    for v in (10.0, 5.0, 1.0, 0.5):
      assert d.update(False, v, -0.5) is False
    assert d.update(True, 0.2, -0.1) is True

  def test_once_rolling_then_stopping_again_is_immediate(self):
    d = GoDebounce()
    for s in [True] * 5 + [False] * 3:
      d.update(s, 0.0, 0.09)
    d.update(False, 0.8, 0.5)             # got moving
    assert d.update(True, 0.2, 0.05) is True
