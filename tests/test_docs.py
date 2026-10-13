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
README = ROOT / "README.md"

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
    def test_the_guide_exists(self):
        self.assertTrue(GUIDE.exists(), "docs/testing_guide.md is missing")

    def test_every_script_the_guide_tells_you_to_run_exists(self):
        missing = sorted(path for path in _paths_in(GUIDE.read_text(encoding="utf-8"))
                         if not (ROOT / path).exists())
        self.assertEqual(missing, [], f"the testing guide references files that do not exist: "
                                      f"{missing}")

    def test_every_script_the_readme_tells_you_to_run_exists(self):
        missing = sorted(path for path in _paths_in(README.read_text(encoding="utf-8"))
                         if not (ROOT / path).exists())
        self.assertEqual(missing, [], f"README.md references files that do not exist: {missing}")

    def test_the_guide_covers_every_phase(self):
        text = GUIDE.read_text(encoding="utf-8")
        for phase in range(1, 9):
            self.assertIn(f"Phase {phase}", text, f"Phase {phase} is not in the testing guide")


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


class FirmwarePolarityDocs(unittest.TestCase):
    """The one-line polarity fix must be documented where someone will find it."""

    def test_the_wiring_doc_names_the_polarity_constant(self):
        wiring = (ROOT / "docs" / "phase3_electronics_and_wiring.md").read_text(encoding="utf-8")
        self.assertIn("CLIFF_TRIPPED_IS_HIGH", wiring)
        self.assertIn("Gate D", wiring, "the wiring doc should point at the polarity procedure")


if __name__ == "__main__":
    unittest.main()
