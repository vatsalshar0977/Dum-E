"""B.U.D.-E setup check. Run this FIRST, and run it from anywhere.

    python check_setup.py
    python C:\\full\\path\\to\\check_setup.py

It answers the questions that are actually hard on a fresh machine: where is the repo, is my
Python new enough, do I have the optional tools, is this a git clone or a ZIP, and what exactly
do I type next. Every script in this repository locates itself from its own file path, so you can
run any of them by absolute path from any working directory - this check tells you the path to
copy.

Nothing here touches hardware, and nothing here is a test result: this is a diagnosis of your
environment. Exit code 0 means the essentials are in place; the optional tools are optional.
"""

from __future__ import annotations

import platform
import shutil
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parent
MIN_PYTHON = (3, 9)

# The files whose absence means "you are not in the repository, or your ZIP downloaded badly".
KEY_FILES: Sequence[str] = (
    "README.md",
    "src/phase8_safety_testing/run_phase_walkthrough.py",
    "src/phase8_safety_testing/run_full_acceptance_suite.py",
    "src/phase4_simulation/web_sim_server.py",
    "src/phase3_electronics/esp32_arm_driver.py",
    "firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino",
    "docs/testing_guide.md",
    "docs/testing_checklist.md",
    "tests/test_firmware.py",
    "config/robot_config.yaml",
    "urdf/bude_arm.urdf",
    "cad/stl/00_tolerance_test_coupon.stl",
)


def _has_module(name: str) -> bool:
    try:
        import importlib
        importlib.import_module(name)
        return True
    except Exception:
        return False


def _module_version(name: str) -> Optional[str]:
    try:
        import importlib
        module = importlib.import_module(name)
        return str(getattr(module, "__version__", "installed"))
    except Exception:
        return None


def _which(*names: str) -> Optional[str]:
    for name in names:
        found = shutil.which(name)
        if found:
            return found
    return None


def _row(label: str, value: str) -> None:
    print(f"  {label:<16}: {value}")


def test_modules() -> List[str]:
    """The stdlib test runner needs the module list spelled out, so build it, never hard-code."""
    return [f"tests.{p.stem}" for p in sorted((ROOT / "tests").glob("test_*.py"))]


def main() -> int:
    print("=" * 78)
    print("  B.U.D.-E SETUP CHECK")
    print("=" * 78)

    essentials_ok = True

    # ---- where things are ---------------------------------------------------------------
    cwd = Path.cwd()
    _row("repo root", str(ROOT))
    _row("you are in", str(cwd))
    if cwd != ROOT:
        print("                 (different folder - that is fine, every script locates itself,")
        print("                  but if you prefer to cd there, copy this line:)")
        print(f'                 cd "{ROOT}"')

    # ---- Python ------------------------------------------------------------------------
    version = sys.version_info
    version_text = f"{version.major}.{version.minor}.{version.micro}"
    if (version.major, version.minor) >= MIN_PYTHON:
        _row("python", f"{version_text}  OK")
    else:
        essentials_ok = False
        _row("python", f"{version_text}  TOO OLD - 3.9 or newer required")
    _row("platform", platform.platform())

    # ---- the files ---------------------------------------------------------------------
    missing = [name for name in KEY_FILES if not (ROOT / name).exists()]
    if missing:
        essentials_ok = False
        _row("key files", f"{len(KEY_FILES) - len(missing)}/{len(KEY_FILES)} present - MISSING:")
        for name in missing:
            print(f"                   {name}")
        print("                 If this folder is empty or half-populated, your download was")
        print("                 incomplete: re-download the branch ZIP and extract it fully.")
    else:
        _row("key files", f"{len(KEY_FILES)}/{len(KEY_FILES)} present  OK")

    # ---- optional: pytest --------------------------------------------------------------
    pytest_version = _module_version("pytest")
    if pytest_version:
        _row("pytest", f"{pytest_version}  OK - gives the nicer `python -m pytest -v` output")
    else:
        _row("pytest", "not installed (OPTIONAL)")
        print("                 install it with:  python -m pip install pytest")
        print("                 without it, use the stdlib runner printed at the bottom")

    # ---- optional: pyserial (hardware only) --------------------------------------------
    serial_version = _module_version("serial")
    if serial_version:
        _row("pyserial", f"{serial_version}  OK - needed only to talk to the real ESP32")
    else:
        _row("pyserial", "not installed (OPTIONAL - hardware only)")
        print("                 install it with:  python -m pip install pyserial")

    # ---- optional: a C++ compiler ------------------------------------------------------
    compiler = _which("g++", "clang++")
    if compiler:
        _row("C++ compiler", f"{compiler}  OK - the firmware is compiled and executed")
    else:
        _row("C++ compiler", "NOT FOUND (OPTIONAL)")
        print("                 Without it, 6 unit tests SKIP and 2 acceptance checks report")
        print("                 'skipped: no C++ compiler'. Everything else still runs.")
        print("                 To enable them, any one of:")
        print("                   winget install -e --id MSYS2.MSYS2      (then, inside MSYS2:")
        print("                       pacman -S mingw-w64-ucrt-x86_64-gcc)")
        print("                   winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT")
        print("                   macOS:  xcode-select --install")
        print("                   Linux:  sudo apt install g++")
        print("                 If winget says the id is unknown, run: winget search gcc")

    # ---- git clone or ZIP? -------------------------------------------------------------
    if (ROOT / ".git").exists():
        if _which("git"):
            _row("source", "git clone - `git pull` will fetch updates")
        else:
            _row("source", "git clone, but git is not on PATH - `git pull` will not work")
    else:
        _row("source", "ZIP copy (no .git)")
        print("                 `git pull` cannot work here: 'fatal: not a git repository' is")
        print("                 expected, not a fault. To get updates, either re-download the")
        print("                 branch ZIP, or switch to a real clone:")
        print("                   git clone -b arena/dc4957e8-dum-e "
              "https://github.com/vatsalshar0977/Dum-E.git bude")

    if "OneDrive" in str(ROOT):
        print()
        print("  NOTE: this folder is inside OneDrive. If a file you can see in Explorer is")
        print("  reported as missing, right-click the folder -> 'Always keep on this device'")
        print("  (Files On-Demand can leave cloud-only placeholders). Moving the repo out of")
        print("  OneDrive also avoids sync churn during long test runs.")

    # ---- next steps --------------------------------------------------------------------
    modules = " ".join(test_modules())
    print()
    print("-" * 78)
    print("  NEXT - run these, in this order")
    print("-" * 78)
    print("  1. python src/phase8_safety_testing/run_phase_walkthrough.py")
    print("       every phase entry point, in order; writes docs/phase_walkthrough_transcript.txt")
    if pytest_version:
        print("  2. python -m pytest -v")
        print(f"     (no pytest? use: python -m unittest {modules} -v)")
    else:
        print(f"  2. python -m unittest {modules} -v")
    print("  3. python src/phase8_safety_testing/run_full_acceptance_suite.py")
    print("  4. python src/phase4_simulation/web_sim_server.py   ->  http://127.0.0.1:8080/")
    print()
    if cwd != ROOT:
        print("  Running from a different folder? Use the absolute path, e.g.:")
        print(f'    python "{ROOT / "src" / "phase8_safety_testing" / "run_phase_walkthrough.py"}"')
        print("  (that works from anywhere - the scripts locate the repo themselves)")
    print("  Hardware steps start at Part 2 of docs/testing_guide.md, with the e-stop wired")
    print("  before any servo is connected.")
    print("=" * 78)
    return 0 if essentials_ok else 1


if __name__ == "__main__":
    sys.exit(main())
