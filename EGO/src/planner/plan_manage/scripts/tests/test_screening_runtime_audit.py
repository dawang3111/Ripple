#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from screening_runtime_audit import audit_runtime


class RuntimeAuditTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        (self.run / "result.txt").write_text("fatal_runtime_count=1\nprocess_crash_count=0\n")

    def test_marker_keeps_teardown_errors_out_of_runtime(self):
        (self.run / "run.log").write_text(
            "auto-starting new master\nnormal operation\n__RUN_HEADLESS_SHUTDOWN_BEGIN__ ts=100\n"
            "terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'\n")
        self.assertEqual(audit_runtime(self.run)["status"], "shutdown_only_errors")

    def test_genuine_failure_before_marker_remains_invalid(self):
        (self.run / "run.log").write_text(
            "Segmentation fault\n__RUN_HEADLESS_SHUTDOWN_BEGIN__ ts=100\n")
        self.assertEqual(audit_runtime(self.run)["status"], "runtime_failure")

    def legacy(self, before="", after=""):
        launch = self.run / "roslaunch-test.log"
        launch.write_text(before + "[roslaunch.pmon][INFO] ProcessMonitor.shutdown <ProcessMonitor>\n" + after)
        (self.run / "run.log").write_text(
            "... logging to {}\nauto-starting new master\n"
            "terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'\n".format(launch))

    def test_overwritten_legacy_marker_recovers_only_verified_sigint_abort(self):
        self.legacy(after="process[renderer]: SIGINT killed with return value -6\n")
        audit = audit_runtime(self.run)
        self.assertEqual(audit["status"], "shutdown_only_errors")
        self.assertEqual(audit["lock_error_count"], 1)
        self.assertEqual(len(audit["shutdown_abort_lines"]), 1)

    def test_process_crashing_before_shutdown_is_not_rehabilitated(self):
        self.legacy(before="process[planner] has died [pid 123, exit code -6]\n",
                    after="process[renderer]: SIGINT killed with return value -6\n")
        self.assertEqual(audit_runtime(self.run)["status"], "runtime_failure")

    def test_missing_or_mismatched_evidence_stays_unresolved(self):
        self.legacy()
        self.assertEqual(audit_runtime(self.run)["status"], "unresolved")
        (self.run / "roslaunch-test.log").unlink()
        self.assertEqual(audit_runtime(self.run)["status"], "unresolved")

    def test_clean_exit_zero_is_not_a_runtime_crash(self):
        self.legacy(before="process[helper] has died [pid 123, exit code 0]\n",
                    after="process[renderer]: SIGINT killed with return value -6\n")
        self.assertEqual(audit_runtime(self.run)["status"], "shutdown_only_errors")


if __name__ == "__main__":
    unittest.main()
