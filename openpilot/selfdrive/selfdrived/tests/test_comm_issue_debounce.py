"""The generic commIssue check in SelfdriveD.update_events is debounced for COMM_ISSUE_DEBOUNCE_FRAMES frames.

The check's own statements are taken from selfdrived.py and run here with stand-ins for the SubMaster and the event
list, so the test follows the code as written. It exists because a port of the debounce once left a second
`self.comm_issue_counter = 0` after the check: the counter then never passed 1 and the event could not fire at all.
"""
import ast
import inspect
import textwrap
import types
import unittest

from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.selfdrived import COMM_ISSUE_DEBOUNCE_FRAMES, SelfdriveD


def comm_issue_statements() -> str:
  """update_events' statements from the counter update on that read or write comm_issue_counter."""
  source = textwrap.dedent(inspect.getsource(SelfdriveD.update_events))
  body = ast.parse(source).body[0].body
  touching = [n for n in body if "comm_issue_counter" in ast.unparse(n)]
  assert touching and isinstance(touching[0], ast.Assign), "the counter update is no longer the first statement that uses it"
  return "\n".join(ast.unparse(n) for n in touching)


class FakeSubMaster:
  def __init__(self):
    self.valid, self.alive, self.freq_ok = {"modelV2": False}, {"modelV2": True}, {"modelV2": True}
    self.frame = 0

  def all_alive(self):
    return True

  def all_freq_ok(self):
    return True


class TestCommIssueDebounce(unittest.TestCase):
  def setUp(self):
    self.code = compile(comm_issue_statements(), "update_events (commIssue check)", "exec")
    self.events = []
    self.sd = types.SimpleNamespace(comm_issue_counter=0, logged_comm_issue=None, sm=FakeSubMaster(),
                                    events=types.SimpleNamespace(add=self.events.append))
    self.env = dict(vars(selfdrived))
    self.env.update(self=self.sd, cloudlog=types.SimpleNamespace(event=lambda *args, **kwargs: None))

  def frame(self, fault: bool) -> int:
    """One pass of the check; returns how many events it added."""
    before = len(self.events)
    self.env["raw_comm_issue"] = fault
    exec(self.code, self.env)
    return len(self.events) - before

  def test_fires_once_the_fault_has_lasted_longer_than_the_debounce(self):
    added = [self.frame(True) for _ in range(COMM_ISSUE_DEBOUNCE_FRAMES + 10)]
    self.assertEqual(added[:COMM_ISSUE_DEBOUNCE_FRAMES], [0] * COMM_ISSUE_DEBOUNCE_FRAMES)
    self.assertEqual(added[COMM_ISSUE_DEBOUNCE_FRAMES:], [1] * 10)
    self.assertEqual(set(self.events), {selfdrived.EventName.commIssue})

  def test_a_fault_shorter_than_the_debounce_does_not_fire(self):
    for _ in range(5):
      self.assertEqual(sum(self.frame(True) for _ in range(COMM_ISSUE_DEBOUNCE_FRAMES)), 0)
      self.assertEqual(self.frame(False), 0)
      self.assertEqual(self.sd.comm_issue_counter, 0)

  def test_the_counter_counts_consecutive_faulty_frames(self):
    for n in range(1, COMM_ISSUE_DEBOUNCE_FRAMES + 5):
      self.frame(True)
      self.assertEqual(self.sd.comm_issue_counter, n)

  def test_only_the_update_writes_the_counter(self):
    source = textwrap.dedent(inspect.getsource(SelfdriveD.update_events))
    writes = [n for n in ast.walk(ast.parse(source)) if isinstance(n, (ast.Assign, ast.AugAssign))
              and "comm_issue_counter" in ast.unparse(n.targets[0] if isinstance(n, ast.Assign) else n.target)]
    self.assertEqual(len(writes), 1, [ast.unparse(n) for n in writes])


if __name__ == "__main__":
  unittest.main()
