"""The documentation is executable here, so it has to stay true.

`docs/testing_guide.md` and `README.md` are full of copy-paste commands. This repository has
already shipped a walkthrough that named three scripts which did not exist; that is a bad failure
mode, because the reader cannot tell a typo from a missing feature. These tests resolve every
path the docs mention and fail if it is not in the repository.
"""

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUIDE = ROOT / "docs" / "testing_guide.md"
CHECKLIST = ROOT / "docs" / "testing_checklist.md"
README = ROOT / "README.md"
# Docs whose commands a reader is invited to type. Every path they mention must exist.
EXECUTABLE_DOCS = (GUIDE, CHECKLIST, README)

# `src/whatever/file.py` written with either slash, plus firmware sources.
PATH_PATTERNS = (
    re.compile(r"src[\\/][A-Za-z0-9_\\/]+\.py"),
    re.compile(r"firmware[\\/][A-Za-z0-9_\\/]+\.(?:ino|cpp|h)"),
    re.compile(r"tests[\\/][A-Za-z0-9_\\/]+\.(?:py|cpp|h)"),
    re.compile(r"docs[\\/][A-Za-z0-9_\\/]+\.md"),
)


def _paths_in(text):
    found = set()
    for pattern in PATH_PATTERNS:
        for match in pattern.findall(text):
            normalised = match.replace("\\", "/")
            if "*" in normalised:
                continue
            found.add(normalised)
    return found


class GuidePaths(unittest.TestCase):
    def test_the_guide_and_checklist_exist(self):
        self.assertTrue(GUIDE.exists(), "docs/testing_guide.md is missing")
        self.assertTrue(CHECKLIST.exists(), "docs/testing_checklist.md is missing")

    def test_every_path_the_docs_tell_you_to_run_exists(self):
        for document in EXECUTABLE_DOCS:
            with self.subTest(document=document.name):
                missing = sorted(path for path in _paths_in(document.read_text(encoding="utf-8"))
                                 if not (ROOT / path).exists())
                self.assertEqual(missing, [],
                                 f"{document.name} references files that do not exist: {missing}")

    def test_every_command_block_in_the_checklist_is_a_real_script(self):
        """The checklist is nothing but commands; a typo in it wastes a reader's evening."""
        text = CHECKLIST.read_text(encoding="utf-8")
        commands = re.findall(r"`([^`]+)`", text)
        scripts = [c for c in commands if re.search(r"\.(?:py|ino|cpp|exe)", c)]
        self.assertGreaterEqual(len(scripts), 15, "the checklist lost its commands")
        for script in scripts:
            paths = _paths_in(script)
            for path in paths:
                self.assertTrue((ROOT / path).exists(), f"checklist runs a missing file: {path}")

    def test_the_guide_covers_every_phase(self):
        text = GUIDE.read_text(encoding="utf-8")
        for phase in range(1, 9):
            self.assertIn(f"Phase {phase}", text, f"Phase {phase} is not in the testing guide")

    def test_the_checklist_covers_every_hardware_gate(self):
        text = CHECKLIST.read_text(encoding="utf-8")
        for gate in "ABCDEFGHIJ":
            self.assertIn(f"Gate {gate}", text, f"Gate {gate} is not in the checklist")

    def test_the_two_documents_point_at_each_other(self):
        """A reader who lands on either one must be able to find the other."""
        self.assertIn("docs/testing_guide.md", CHECKLIST.read_text(encoding="utf-8"))
        self.assertIn("testing_checklist.md", GUIDE.read_text(encoding="utf-8"))


class GuideHonesty(unittest.TestCase):
    """The two claims that must never quietly disappear from the guide."""

    def setUp(self):
        self.text = GUIDE.read_text(encoding="utf-8")

    def test_it_says_the_firmware_has_never_been_flashed(self):
        lowered = self.text.lower()
        self.assertIn("never been flashed", lowered)

    def test_it_says_nothing_here_is_evidence_about_hardware(self):
        lowered = self.text.lower()
        self.assertIn("nothing in tier 1 or 2 is evidence about tier 3", lowered)
        self.assertIn("the servo torque figures are datasheets", lowered)

    def test_it_tells_you_the_e_stop_is_the_only_real_stop(self):
        self.assertIn("the only thing that stops this robot", self.text.lower())

    def test_it_warns_about_the_unfitted_cliff_sensor_trap(self):
        # An INPUT_PULLUP pin with nothing on it reads HIGH = "cliff", so forward is refused.
        lowered = self.text.lower()
        self.assertIn("unplugged sensor reads as", lowered)
        self.assertIn("forward_refused", lowered)

    def test_it_explains_the_latched_e_stop(self):
        self.assertIn("will not re-enable itself", self.text.lower())


class SetupCheck(unittest.TestCase):
    """`check_setup.py` is the first thing a new user runs, so it has to work from anywhere.

    A reader whose shell is in the wrong folder sees `can't open file '...src...': No such file
    or directory`; this script is the answer to that, which means it must not itself depend on
    the working directory.
    """

    def test_it_runs_from_another_folder_and_names_every_test_module(self):
        import subprocess
        import sys
        script = ROOT / "check_setup.py"
        self.assertTrue(script.exists(), "check_setup.py is missing from the repo root")
        # cwd is deliberately OUTSIDE the repository: that is the case that breaks people.
        done = subprocess.run([sys.executable, str(script)], cwd=str(ROOT.parent),
                              capture_output=True, text=True, timeout=180)
        self.assertEqual(done.returncode, 0, done.stdout[-2000:] + done.stderr[-800:])
        output = done.stdout
        self.assertIn("B.U.D.-E SETUP CHECK", output)
        self.assertIn(str(ROOT), output, "the check must print the repository root")
        self.assertIn("run_phase_walkthrough.py", output)
        modules = sorted(path.stem for path in (ROOT / "tests").glob("test_*.py"))
        self.assertGreaterEqual(len(modules), 6)
        for module in modules:
            self.assertIn(f"tests.{module}", output,
                          f"the setup check does not offer to run {module}")


class FirmwarePolarityDocs(unittest.TestCase):
    """The one-line polarity fix must be documented where someone will find it."""

    def test_the_wiring_doc_names_the_polarity_constant(self):
        wiring = (ROOT / "docs" / "phase3_electronics_and_wiring.md").read_text(encoding="utf-8")
        self.assertIn("CLIFF_TRIPPED_IS_HIGH", wiring)
        self.assertIn("Gate D", wiring, "the wiring doc should point at the polarity procedure")


if __name__ == "__main__":
    unittest.main()
