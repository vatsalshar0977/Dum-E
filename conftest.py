"""Pytest bootstrap: make `import src...` work from ANY directory.

The test modules import the project like a package:

    from src.phase5_control.kinematics import forward, inverse, LIMITS

That only works when the REPOSITORY ROOT is on `sys.path`. `python -m pytest` puts the *current
working directory* there, so running it from the repo root works and running it from anywhere else
fails with:

    ModuleNotFoundError: No module named 'src'

...which says nothing about the real cause. This file fixes it once, for every invocation:
`python -m pytest`, `pytest`, `pytest tests/test_firmware.py`, and any of them run from another
folder with an absolute path.

It is deliberately kept to one job. `pytest.ini` sets `rootdir`, `testpaths` and `pythonpath` for
newer pytest; this works on older pytest too, which is why both exist.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
