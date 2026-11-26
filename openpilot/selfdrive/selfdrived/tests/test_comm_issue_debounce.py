"""The generic commIssue check in SelfdriveD.update_events is debounced for COMM_ISSUE_DEBOUNCE_FRAMES frames.

The check's own statements, from the line that decides whether there is a fault to the end of the block that raises
the event, are taken from selfdrived.py and run here against a stand-in SubMaster and event list, so the test follows
the code as written without starting the process. It exists because a port of the debounce once left a second
`self.comm_issue_counter = 0` after the check: the counter then never passed 1 and the event could not fire at all.
"""
import ast
import inspect
import textwrap
import types
import unittest

from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.selfdrived import COMM_ISSUE_DEBOUNCE_FRAMES, SelfdriveD

EventName = selfdrived.EventName


def comm_issue_statements() -> str:
  """update_events from `raw_comm_issue = ...` on: that assignment and every later statement that uses the counter."""
  body = ast.parse(textwrap.dedent(inspect.getsource(SelfdriveD.update_events))).body[0].body
  start = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "raw_comm_issue")
  return "\n".join(ast.unparse(n) for i, n in enumerate(body[start:]) if i == 0 or "comm_issue_counter" in ast.unparse(n))


class FakeSubMaster:
  """One covered service, with SubMaster's aggregate checks."""
  def __init__(self):
    self.valid, self.alive, self.freq_ok = {"lateralDelay": True}, {"lateralDelay": True}, {"lateralDelay": True}
    self.frame = 0

  def all_alive(self):
    return all(self.alive.values())

  def all_freq_ok(self):
    return all(self.freq_ok.values())

  def all_valid(self):
    return all(self.valid.values())

  def all_checks(self):
    return self.all_alive() and self.all_freq_ok() and self.all_valid()


class TestCommIssueDebounce(unittest.TestCase):
  def setUp(self):
    self.code = compile(comm_issue_statements(), "update_events (commIssue check)", "exec")
    self.events = []
    self.sm = FakeSubMaster()
    self.sd = types.SimpleNamespace(comm_issue_counter=0, logged_comm_issue=None, sm=self.sm,
                                    events=types.SimpleNamespace(add=self.events.append))
    self.env = dict(vars(selfdrived))
    self.env.update(self=self.sd, cloudlog=types.SimpleNamespace(event=lambda *args, **kwargs: None),
                    no_system_errors=True, big_model_settling=False)

  def frame(self) -> list:
    """One pass of the check; returns the events it added."""
    before = len(self.events)
    self.sm.frame += 1
    exec(self.code, self.env)
    return self.events[before:]

  def test_a_healthy_system_raises_nothing(self):
    for _ in range(10 * COMM_ISSUE_DEBOUNCE_FRAMES):
      self.assertEqual(self.frame(), [])
      self.assertEqual(self.sd.comm_issue_counter, 0)

  def test_each_kind_of_fault_fires_once_it_has_outlasted_the_debounce(self):
    for part, event in (("valid", EventName.commIssue), ("alive", EventName.commIssue), ("freq_ok", EventName.commIssueAvgFreq)):
      with self.subTest(part=part):
        self.setUp()
        getattr(self.sm, part)["lateralDelay"] = False
        added = [self.frame() for _ in range(COMM_ISSUE_DEBOUNCE_FRAMES + 10)]
        self.assertEqual(added[:COMM_ISSUE_DEBOUNCE_FRAMES], [[]] * COMM_ISSUE_DEBOUNCE_FRAMES)
        self.assertEqual(added[COMM_ISSUE_DEBOUNCE_FRAMES:], [[event]] * 10)

  def test_the_counter_counts_consecutive_faulty_frames_and_clears_on_recovery(self):
    self.sm.valid["lateralDelay"] = False
    for n in range(1, COMM_ISSUE_DEBOUNCE_FRAMES + 5):
      self.frame()
      self.assertEqual(self.sd.comm_issue_counter, n)
    self.sm.valid["lateralDelay"] = True
    self.assertEqual(self.frame(), [])
    self.assertEqual(self.sd.comm_issue_counter, 0)

  def test_a_fault_shorter_than_the_debounce_does_not_fire(self):
    for _ in range(5):
      self.sm.valid["lateralDelay"] = False
      self.assertEqual(sum(len(self.frame()) for _ in range(COMM_ISSUE_DEBOUNCE_FRAMES)), 0)
      self.sm.valid["lateralDelay"] = True
      self.assertEqual(self.frame(), [])

  def test_the_two_conditions_that_suppress_the_check_still_do(self):
    self.sm.valid["lateralDelay"] = False
    for name, value in (("no_system_errors", False), ("big_model_settling", True)):
      with self.subTest(name=name):
        self.env.update(no_system_errors=True, big_model_settling=False)
        self.env[name] = value
        self.sd.comm_issue_counter = 0
        self.assertEqual(sum(len(self.frame()) for _ in range(5 * COMM_ISSUE_DEBOUNCE_FRAMES)), 0)


if __name__ == "__main__":
  unittest.main()
