"""The deep CAD audit is a report, not a gate - but it must keep working and must not find
anything NEW on its own.

`known_issues` below is the ledger of everything the audit reports about the 13 shipped STL
files at this commit. Fixing a listed item keeps this test green (the assertion is a subset
test); a fresh geometric defect - a bore that moves, a pocket that shrinks, a body that comes
apart - adds an entry and fails, loudly.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
AUDIT = ROOT / "src" / "phase2_mechanical" / "deep_cad_audit.py"

# (section, check name) as printed by the audit's failure tally.
known_issues = {
    ("clearance :: J3", "child never enters the parent through the travel"),
    ("clearance :: J4", "child never enters the parent through the travel"),
    ("fits :: M3 hex trap, base plate", "assembles at worst-case print error"),
    ("fits :: M3 hex trap, coupon", "assembles at worst-case print error"),
    ("fits :: passive pin bore, forearm", "assembles at worst-case print error"),
    ("fits :: passive pin bore, upper arm", "assembles at worst-case print error"),
    ("fits :: wheel hub bore, left", "assembles at worst-case print error"),
    ("fits :: wheel hub bore, right", "assembles at worst-case print error"),
    ("placement :: J1 MG996R in the base tower", "case still fits at worst-case print error"),
    ("placement :: J4 MG90S in the forearm", "case still fits at worst-case print error"),
    ("placement :: J5 MG90S in the wrist", "case 12.2 mm fits along Y"),
    ("placement :: J5 MG90S in the wrist", "case still fits at worst-case print error"),
    ("placement :: tray", "all three boards fit without stacking"),
}


def failing_checks(text: str):
    """[(section, check name)] from the audit's final FAIL tally."""
    out = []
    for line in text.splitlines():
        if not line.startswith("  FAIL "):
            continue
        parts = line[len("  FAIL "):].rsplit(" :: ", 2)
        if len(parts) == 3:
            out.append((parts[0], parts[1]))
    return set(out)


def test_deep_cad_audit_reports_only_known_issues():
    proc = subprocess.run(
        [sys.executable, str(AUDIT), "--quick"],
        cwd=ROOT, capture_output=True, text=True, timeout=900,
        check=False,
    )
    # exit 1 simply means "there are findings"; a crash would be a different code or a traceback.
    assert "Traceback" not in proc.stderr, proc.stderr
    assert "deep CAD checks passed" in proc.stdout, proc.stdout[-2000:]

    reported = failing_checks(proc.stdout)
    new = reported - known_issues
    assert not new, f"the deep audit found new geometry problems: {sorted(new)}"


def test_audit_checks_every_part():
    """Every shipped STL must appear in the audit - a new part cannot slip through unmeasured."""
    stl = sorted((ROOT / "cad" / "stl").glob("*.stl"))
    assert len(stl) == 13, [p.name for p in stl]
    text = AUDIT.read_text(encoding="utf-8")
    for path in stl:
        assert path.name in text, f"{path.name} is not covered by the deep audit"
