#!/usr/bin/env python3
"""Separate experiment-time failures from verified SIGINT teardown failures."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List


FATAL = re.compile(
    r"Eigen::.*Assertion|Assertion .+ failed|boost::wrapexcept<boost::lock_error>|"
    r"terminate called after throwing|Segmentation fault|core dumped|"
    r"process monitor has been shut down|RLException: cannot add process",
    re.IGNORECASE,
)
SHUTDOWN_MARK = "__RUN_HEADLESS_SHUTDOWN_BEGIN__"
LOCK_ERROR = "boost::wrapexcept<boost::lock_error>"


def nonzero_exits(lines: List[str]) -> List[str]:
    output = []
    for line in lines:
        if "has died" not in line and "process died" not in line and "exit code" not in line and "return value" not in line:
            continue
        match = re.search(r"(?:exit code|return value)\s+(-?\d+)", line)
        if match is not None and int(match.group(1)) != 0:
            output.append(line)
        elif match is None and ("has died" in line or "process died" in line):
            output.append(line)
    return output


def audit_runtime(run: Path) -> Dict[str, Any]:
    result = run / "result.txt"
    if not result.is_file():
        return {"status": "not_reported", "source": "", "runtime_errors": 0}
    values = dict(line.split("=", 1) for line in result.read_text(errors="replace").splitlines() if "=" in line)
    flagged = False
    for key in ("fatal_runtime_count", "process_crash_count"):
        try:
            flagged = flagged or float(values.get(key, "0")) > 0
        except ValueError:
            return {"status": "unresolved", "source": str(result), "runtime_errors": None}
    if not flagged:
        return {"status": "clean_runtime", "source": str(result), "runtime_errors": 0}

    raw_path = run / "run.log"
    if not raw_path.is_file():
        return {"status": "runtime_failure", "source": str(result), "runtime_errors": None}
    raw = raw_path.read_text(errors="replace")
    if SHUTDOWN_MARK in raw:
        before = raw.split(SHUTDOWN_MARK, 1)[0].splitlines()
        failures = [line for line in before if FATAL.search(line)] + nonzero_exits(before)
        return {"status": "runtime_failure" if failures else "shutdown_only_errors", "source": str(raw_path),
                "runtime_errors": len(failures), "runtime_failure_lines": failures}

    # Legacy roslaunch used a non-append descriptor and could overwrite the marker.
    # Its dedicated log preserves process lifetime order independently of buffered stdout.
    match = re.search(r"logging to ([^\r\n]+roslaunch-[^\r\n]+\.log)", raw[:8192])
    launch_path = Path(match.group(1).strip()) if match else None
    if launch_path is None or not launch_path.is_file():
        return {"status": "unresolved", "source": str(raw_path), "runtime_errors": None,
                "reason": "missing_shutdown_marker_and_authoritative_roslaunch_log"}
    lines = launch_path.read_text(errors="replace").splitlines()
    boundary = next((index for index, line in enumerate(lines) if "ProcessMonitor.shutdown <" in line), None)
    if boundary is None:
        return {"status": "unresolved", "source": str(launch_path), "runtime_errors": None,
                "reason": "missing_roslaunch_shutdown_event"}
    premature = nonzero_exits(lines[:boundary])
    raw_fatals = [line for line in raw.splitlines() if FATAL.search(line)]
    lock_errors = [line for line in raw_fatals if LOCK_ERROR in line]
    other_fatals = [line for line in raw_fatals if LOCK_ERROR not in line]
    shutdown_aborts = [line for line in lines[boundary:] if "SIGINT killed with return value -6" in line]
    evidence = {
        "source": str(launch_path), "shutdown_start_line": lines[boundary],
        "premature_exit_lines": premature, "lock_error_count": len(lock_errors),
        "other_fatal_lines": other_fatals, "shutdown_abort_lines": shutdown_aborts,
    }
    if premature:
        return dict(evidence, status="runtime_failure", runtime_errors=len(premature))
    if not other_fatals and len(lock_errors) == len(shutdown_aborts):
        return dict(evidence, status="shutdown_only_errors", runtime_errors=0)
    return dict(evidence, status="unresolved", runtime_errors=None,
                reason="fatal_output_not_fully_explained_by_recorded_SIGINT_teardown")
