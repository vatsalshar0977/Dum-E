# Testing B.U.D.-E — the complete step-by-step guide

> In a hurry, or want something to tick off? **`docs/testing_checklist.md`** is the same
> procedure condensed to checkboxes and pass criteria, with no explanations. This page is the
> long version: what each test proves, and how to read it when it fails.

**Read this page before you type anything.** It is written so that you can start at Step 1 with a
laptop and nothing else, and finish at Gate J with an arm that fetches a screwdriver on voice
command. Every command is copy-paste ready, every expected output is real output from this
repository, and every step says what it proves and what it does not.

Commands are written for **Windows PowerShell**. On macOS or Linux, replace `python` with
`python3` and the backslashes with forward slashes — nothing else changes.

---

## The three kinds of test in this repo, because they are not equal

| Tier | What it tells you | Where it lives | Time |
|---|---|---|---|
| **1. Does the code run?** | A script executes and its self-checks agree with themselves | `src/**`, `tests/**` | 30 min, laptop only |
| **2. Is the model self-consistent?** | The STLs, the URDF, the config, the wiring table and the firmware all describe *the same robot* | `*_verifier.py`, `verify_*.py`, the acceptance suite | included above |
| **3. Does the hardware work?** | A servo moved, a sensor saw a floor, a wheel turned | **you**, with a multimeter and a current-limited supply | an afternoon |

**Nothing in tier 1 or 2 is evidence about tier 3.** No STL has been printed, no servo has been
powered, no camera has seen a photograph, and the ESP32 sketch has never been flashed. When this
guide says PASS, it means "the arithmetic and the code agree", not "it works".

---

# PART 1 — On your laptop (about 30 minutes, no hardware)

## Step 0 — Find your folder (30 seconds, and it prevents most of the confusion)

Windows PowerShell starts in `C:\WINDOWS\system32`, not in your download folder. A relative path
like `python src\phase8_safety_testing\...` is resolved against **where the shell really is**, so
you get:

```
python: can't open file 'C:\WINDOWS\system32\src\phase8_safety_testing\run_phase_walkthrough.py'
```

That message means *the folder*, not the file, and not your download. You have two ways out, and
either is fine:

**Option A — do not `cd` at all.** Every script in this repository finds its own location from its
own file path, so an absolute path works from anywhere:

```powershell
python "C:\full\path\to\check_setup.py"
```

**Option B — `cd` to the folder first.** Open File Explorer, find the folder that directly contains
`README.md`, `src`, `docs`, `firmware` and `tests`, then copy its address bar:

```powershell
cd "C:\Users\you\Documents\Dum-E-arena-dc4957e8-dum-e"
python check_setup.py
```

> **About the folder name:** a ZIP of this branch extracts to `Dum-E-arena-dc4957e8-dum-e` (a
> ZIP of the other branch extracts to `Dum-E-main`). If you extracted twice you may have
> `...\Dum-E-arena-dc4957e8-dum-e\Dum-E-arena-dc4957e8-dum-e` — that is harmless as long as the
> **innermost** folder is the one with `README.md` in it. `check_setup.py` prints `repo root`,
> which is the definitive answer.

---

## Step 1 — Run the setup check

```powershell
python check_setup.py
```

If it is not there, your download predates it — re-download this branch's ZIP, or clone (Step 0b
below). Downloading `main` instead of this branch is a common mistake and gives the *same* error
message as being in the wrong folder, because `main` does not contain the Phase 5–8 work:

```powershell
git clone -b arena/dc4957e8-dum-e https://github.com/vatsalshar0977/Dum-E.git bude
```

It answers, in one shot, the five things that actually bite on a fresh machine: where is the
repo, is my Python new enough, do I have the optional tools, is this a git clone or a ZIP, and
what exactly do I type next.

Expected — everything that matters says `OK`:

```
  repo root       : C:\Users\you\Documents\Dum-E-arena-dc4957e8-dum-e
  you are in      : C:\WINDOWS\system32
  python          : 3.13.7  OK
  key files       : 12/12 present  OK
  pytest          : 9.1.1  OK
  C++ compiler    : NOT FOUND (OPTIONAL)
  source          : ZIP copy (no .git)
```

**You do not have to `cd` anywhere.** Every script in this repository works out its own location
from its own file path, so both of these work no matter where you are:

```powershell
python "C:\full\path\to\check_setup.py"
python "C:\full\path\to\src\phase8_safety_testing\run_phase_walkthrough.py"
```

That is the answer to `can't open file '...\src\...': No such file or directory` — the relative
path was resolved against whatever folder the shell was really in, which is not always the folder
its prompt claims. If you would rather `cd` first, copy the `cd "..."` line the check prints, and
then confirm with `dir` that you can see `README.md`, `src`, `docs`, `firmware`, `tests`.

**Two things it reports that are not faults:**

* **`source : ZIP copy (no .git)`** — a ZIP download has no `.git` directory, so `git fetch`,
  `git checkout` and `git pull` all fail with `fatal: not a git repository`. That is expected, not
  damage. Either re-download the branch ZIP when you want an update, or make a real clone once:

  ```powershell
  git clone -b arena/dc4957e8-dum-e https://github.com/vatsalshar0977/Dum-E.git bude
  ```

* **`C++ compiler : NOT FOUND (OPTIONAL)`** — six unit tests skip and two acceptance checks report
  `skipped: no C++ compiler`. Everything else runs. Step 2 has the install options.

**If it reports `content : WRONG BRANCH OR OLD SNAPSHOT`:** you have the right idea and the wrong
download. `main` does **not** contain the Phase 5–8 work, the firmware or this guide, so those
commands fail with `No such file or directory` — the *same* error you get from being in the wrong
folder. Two ways to fix it:

```powershell
# either: download this branch (the folder extracts as Dum-E-arena-dc4957e8-dum-e)
#   https://github.com/vatsalshar0977/Dum-E/archive/refs/heads/arena/dc4957e8-dum-e.zip

# or: clone once, then `git pull` for every future update
git clone -b arena/dc4957e8-dum-e https://github.com/vatsalshar0977/Dum-E.git bude
```

The ZIP filename tells you which branch you got: **`Dum-E-main.zip`** is the wrong one,
**`Dum-E-arena-dc4957e8-dum-e.zip`** is this one. A ZIP is also a snapshot of the branch tip at
the moment you downloaded it — if `content` reports *some but not all* markers, your snapshot is
simply older than the current tip; re-download or clone.

**If it reports `key files: N/12` with names listed:** the extraction was incomplete. Delete the
folder and extract the ZIP again from scratch rather than patching it up.

**If a file you can see in Explorer is reported as missing:** the folder is inside OneDrive and
the file is a cloud-only placeholder. Right-click the folder → **Always keep on this device**.

---

## Step 2 — Install anything the check flagged

All three are optional; install what you want.

```powershell
python -m pip install pytest pyserial
```

* **pytest** — nicer output (`python -m pytest -v`). Without it, use the stdlib command the setup
  check prints for you; it is the same tests.
* **pyserial** — needed only in Part 2, to talk to the real ESP32 over USB.

For the **firmware** tests you need a C++ compiler, because the firmware is *compiled and
executed*, not merely inspected:

```powershell
g++ --version
```

* Prints a version? Done.
* **Windows:** `winget install -e --id MSYS2.MSYS2`, then inside MSYS2 run
  `pacman -S mingw-w64-ucrt-x86_64-gcc`. Or `winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT`.
  If winget says the id is unknown, run `winget search gcc`. **Open a new terminal afterwards** —
  an existing one will not see the new PATH.
* **macOS:** `xcode-select --install`. **Linux:** `sudo apt install g++`.
* No compiler? Everything still runs. You just lose the evidence that the sketch's safety logic
  actually executes, and the six firmware tests report as skipped rather than passed.

---

## Step 3 — The one command that runs all eight phases

This is the fastest way to see the whole project work:

```powershell
python src\phase8_safety_testing\run_phase_walkthrough.py
```

It runs 18 commands, one per phase, in order, and saves every line of output.

Expected, at the end:

```
  PHASE  EXIT   RESULT
  1      0      PASS
  2      0      212 / 212
  3      0      VERDICT: PASS
  ...
  8      0      131 passed
  18 of 18 commands exited 0.

  transcript written to docs/phase_walkthrough_transcript.txt (81 KB)
```

**What it proves:** every phase entry point executes on a clean machine and each one's own
self-check passes.

**What it does not prove:** anything about hardware, and it is *not* an independent test — a
phase "passes" here when its own script says it did. Step 5 is the independent one.

Open the transcript with any text editor, or read it on GitHub. Steps 5–12 below explain what you
are looking at.

---

## Step 4 — The unit tests

**From the repository folder** (the simplest correct invocation):

```powershell
cd "$R"
python -m pytest -v
```

Expected: **`142 passed`** in a few seconds. (The count grows as tests are added — what matters is
`passed`, and the absence of `failed` or `error`.)

**From anywhere else**, either form works, because `conftest.py` and `pytest.ini` in the repo root
put the repository on the import path:

```powershell
python -m pytest "$R	ests" -v
python -m pytest "$R" -q
```

> **If you see `ModuleNotFoundError: No module named 'src'`**, pytest did not find the repository
> root: the test modules import the project as `src.phase5_control...`, and that import needs the
> repo root on `sys.path`. `python -m pytest` only adds the *current working directory*.
> Symptom: `rootdir` in the header is your home folder rather than the repo. The two commands
> above fix it; so does `cd`-ing first. This was a real bug in an earlier version of this guide —
> the bootstrap files exist now precisely so the absolute-path form works.

No pytest? Use the standard library — same tests, plainer output, and it must be run **from the
repository folder**:

```powershell
cd "$R"
python -m unittest tests.test_docs tests.test_firmware tests.test_foundations tests.test_phase3_tools tests.test_phase5_phase6 tests.test_phase7_phase8 -v
```

The suites map to phases:

| Test file | What it covers |
|---|---|
| `test_foundations.py` | Phase 1 sizing and the URDF loader |
| `test_phase5_phase6.py` | FK/IK, min-jerk planning, collision keep-outs, camera round trip, drive policy |
| `test_phase7_phase8.py` | the six gestures, the intent parser, the supervisor, the six injected faults |
| `test_firmware.py` | **compiles and runs the real `.ino`** against a simulated ESP32 (see Step 7) |
| `test_phase3_tools.py` | the wiring verifier and the serial driver framing |
| `test_docs.py` | every path and command in this guide and the checklist actually exists |

**If a test fails:** copy the assertion text — it is written to tell you what it expected. Do not
"fix" a test to make it pass; the tests are the specification.

---

## Step 5 — The master acceptance suite (the honest scorecard)

```powershell
python src\phase8_safety_testing\run_full_acceptance_suite.py
```

Expected ending:

```
  P1  Sizing: torque, power and workspace                  PASS
  P2  Mechanical: 14 STLs, true hollows, coaxiality         PASS
  P3  Electronics: pins, addresses, voltage drop            PASS
  P4  Simulation: URDF chain and the viewer model           PASS
  P5  Control: FK, IK, minimum jerk, collision keep-outs    PASS
  P6  Perception: camera round trip and the drive policy    PASS
  P7  Voice and personality: six gestures, deterministic parser PASS
  P8  Safety: six injected faults and the telemetry log     PASS
  50 / 50 checks passed in 27.0 s
  VERDICT: PASS
```

Add `--quick` for a 1-second version, or `--json` for a machine-readable one.

**Why this exists as well as the walkthrough:** the walkthrough reports what each script printed
about itself; this suite *asserts* requirements across files — that the URDF's link lengths equal
the config's, that the firmware defines the same GPIOs as the wiring table, that the arm's
current limit sits under the documented trip point. This is the one to run after you change
anything.

Saved copy for the record: `docs/phase8_acceptance_scorecard.txt`.

---

## Step 6 — Phase 2: regenerate the parts and re-audit them

This is the real test of the CAD code: delete the STLs, rebuild them from source, and re-run the
212-point audit. If the generator and the auditor disagreed, this catches it.

```powershell
python src\phase2_mechanical\generate_stl_parts.py
python src\phase2_mechanical\verify_print_parts.py
python src\phase2_mechanical\verify_assembly_fit.py
```

Expected:

```
PASS: every part is a structurally valid binary STL, seats on the bed, fits a
...
  TOTAL      212 / 212 checks   PASS
```

What the 212 checks are: mesh integrity (56), volume/hollowing (14), ray probes (50), coaxality
of every bore (25), pocket depths (36), the +0.60 mm total straddle clearances (12) and the rover
bolt circle (19).

`build_all_parts()` is the authority on volumes — the generator and the auditor share it, so a
part cannot pass by being measured twice with two different rulers.

**Optional deeper check:** `python src\phase2_mechanical\mesh_audit.py` prints the per-part
triangle, volume and wall-thickness tables.

---

## Step 7 — The firmware, compiled and executed (not just inspected)

Two programs, both real C++:

```powershell
g++ -std=c++11 -Wall -Wextra -Wpedantic -o reflex.exe firmware\phase3_full_6dof_esp32\test_reflex_logic.cpp
.\reflex.exe
```

Expected: `PASS: 73 / 73 checks`.

```powershell
g++ -std=c++11 -Wall -Wextra -I tests\firmware_host -o fw.exe tests\firmware_host\main.cpp
.\fw.exe
```

Expected: `PASS: 83 / 83 checks`, including this line:

```
   measured edge -> both channels written: 237 us (budget 5000 us)
```

**What that second program is:** the actual `.ino` `#include`d and compiled against stub
Arduino/ESP32/Wire headers, then driven with a simulated clock, simulated GPIO, a PCA9685 whose
register writes are decoded back into pulse widths, and a fake serial port. The sketch's boot
order, watchdog setup, serial parser, cliff reflex, recoil timing, I²C recovery and OE cut are
all really executed.

**If your compiler is missing:** `g++` will not be found. On Windows install MSYS2 (Step 2) or
run these two lines inside WSL.

**What this does not prove:** that it runs on an ESP32. The I²C peripheral, the interrupts, the
watchdog and the servos are models. See Part 2, Gates B–F, for the hardware.

---

## Step 8 — Phase 3: does everything agree on the pinout?

```powershell
python src\phase3_electronics\power_and_wiring_verifier.py
```

Expected: `VERDICT: PASS — 18 / 18 checks`, and two numbers worth reading:

```
  PASS  the arm alone can demand more current than the 5 A supply can give
           all six arm servos stalled = 10.5 A vs a 5 A supply: it will brown out (not trip)
  PASS  a copper gauge carries 5 A over 1 m and still leaves 4.8 V at the servo
           20 AWG: 333 mV; 22 AWG: 529 mV (ampacity too low) ...
```

Now the laptop end of the serial link, with no hardware attached:

```powershell
python src\phase3_electronics\esp32_arm_driver.py --dry-run --ping --home --drive 0.5 0.5 --stop
```

Expected: the frames it would send, ending `4 frame(s) prepared, nothing sent.` Note
`--drive 0.5 0.5` produces `W 1410 1590` — forward is *below* neutral on the left wheel and
*above* it on the right, because the two drive servos face each other.

`--dry-run` is the default; a real port is only opened if you pass `--port`, and commands that
can move the robot additionally require `--yes`.

---

## Step 9 — Phase 4: is the model the same robot the code plans for?

```powershell
python src\phase4_simulation\verify_urdf_model.py
```

Expected: `VERDICT: PASS — 10 / 10 checks`, including

```
  PASS  the URDF chain and the analytic solver agree on the TCP (6 poses)
           worst disagreement 0.00 mm ... (tolerance 1.5 mm)
```

This walks the URDF's own `<origin>` and `<axis>` data and compares the result against the
analytic solver. It is the check that stops the model and the control code from describing
different robots — writing it caught a metre/millimetre mismatch and a yaw-convention error.

---

## Step 10 — Phase 5 and Phase 6: the control core and the perception stack

Four self-checking scripts. None of them needs hardware.

```powershell
python src\phase5_control\kinematics.py
python src\phase5_control\trajectory_and_collision.py
python src\phase5_control\ros2_moveit_bridge.py --dry-run
python src\phase6_perception\camera_and_hand_eye_calib.py
python src\phase6_perception\vision_mobile_navigator.py
```

Expected output, and what each line is really telling you:

* `kinematics.py` — three FK/IK round trips that return the original joint angles, an unreachable
  target that **raises instead of guessing**, and the pitch-adaptation search finding 6 branches.
  The solver refusing a target is the feature; a solver that always finds "a" solution is how
  arms hit desks.
* `trajectory_and_collision.py` — the quintic minimum-jerk profile and the keep-out volumes. It
  checks that a deliberate collision is actually caught and that peak velocity respects the
  per-joint limits.
* `ros2_moveit_bridge.py --dry-run` — the six-stage pick-and-handover table (approach, descend,
  close, lift, present, release) with the solved TCP for each stage. **This is the analytic
  solver, not MoveIt 2, and no motor is commanded.**
* `camera_and_hand_eye_calib.py` — the pinhole model and a projection round trip whose residual
  (~1e-14 mm) proves the *algebra*, not the camera. The real calibration happens at Gate I.
* `vision_mobile_navigator.py` — the drive policy: bearings to wheel speeds, and the rule that
  stale sensors, a cliff, an obstacle or an e-stop all mean **stop**.

---

## Step 11 — The interactive viewer (this is the fun one)

```powershell
python src\phase4_simulation\web_sim_server.py
```

Then open **http://127.0.0.1:8080/** in a browser. Ctrl+C in the terminal stops it.

Things to try, and what each one demonstrates:

| What to click | What it proves |
|---|---|
| Drag the joint sliders | FK runs in the browser; the arm and the numbers agree |
| Click a target on the desk | IK finds a solution and the plan is drawn |
| The **voice** box: type `get me the pliers` | The Phase 7 parser turns text into an intent and a gesture |
| The **personality** buttons | Each of the six gestures plays as a minimum-jerk motion |
| `/api/personality/sequence?name=startled_cliff_recoil` | The cliff recoil gesture the hardware reflex mirrors |

Available from the same server: `/api/personality`, `/api/voice?text=...`, `/api/parts`,
`/api/plan`.

---

## Step 12 — Read the two reports

* `docs/phase_walkthrough_transcript.txt` — every command and its verbatim output.
* `docs/phase8_acceptance_scorecard.txt` — the 50-check scorecard, ending with the list of things
  it explicitly does **not** prove.

**Part 1 is now complete.** At this point you have verified that the code runs, the models are
self-consistent, and the firmware's logic compiles and executes. You have verified nothing about
atoms.

---

## If something in Part 1 fails

| Symptom | Cause | Fix |
|---|---|---|
| `python: command not found` | Python not on PATH | Reinstall from python.org, tick "Add to PATH"; or use `py` instead of `python` |
| `No module named pytest` | pytest not installed | `python -m pip install pytest`, or use the unittest line in Step 4 |
| `ModuleNotFoundError: No module named 'src'` | pytest could not find the repo root (`rootdir` in the header shows your home folder) | `cd` to the repo first, or run `python -m pytest "$R\tests" -v` — `conftest.py` + `pytest.ini` in the repo root make the absolute-path form work |
| `g++: command not found` | no C++ compiler | Step 2; the firmware tests skip without one |
| `can't open file '...\src\...'` | the relative path resolved against a folder you are not in | `python check_setup.py`, or run it by absolute path: `python "C:\full\path\to\src\phase8_safety_testing\run_phase_walkthrough.py"` |
| `fatal: not a git repository` | the folder is a ZIP download, not a clone | expected — Step 1 shows how to clone or re-download |
| `No such file or directory` for a file that should exist, from the **right** folder | wrong branch (`main` has no Phase 5–8) or an old snapshot | `python check_setup.py` — the `content` line names which branch you actually have |
| `g++ is not recognized` | no C++ compiler | optional — Step 2. Without it 6 tests skip; everything else runs |
| a file visible in Explorer is "missing" | OneDrive cloud-only placeholder | right-click the folder → "Always keep on this device" |
| `ModuleNotFoundError: src` | run from the wrong folder | `cd` to the repository root (the folder containing `README.md`) |
| A test asserts and fails | a real mismatch | Read the assertion text; do not edit the test |
| The viewer shows nothing | port in use | `python src\phase4_simulation\web_sim_server.py` prints which port it took; try `--port 8081` |
| `pip` refuses to install ("externally managed") | Linux system Python | `python -m pip install --user pytest`, or use a venv |

---

# PART 2 — On real hardware, in gates

**This part can destroy hardware and hurt a hand. Work through it in order, and do not skip a
gate because the one before it passed.**

You will need:

* a **multimeter** (continuity + DC volts);
* a **current-limited bench supply**, or at minimum a fused 6 V supply you can switch off with
  your foot;
* the **physical e-stop** wired and tested *before* any servo is connected (Gate A);
* a table you are willing to scratch, and a **contained floor area** for the rover;
* a helper is optional; the ability to hit the e-stop instantly is not.

> ### The three rules for every gate below
> 1. **The e-stop is the only thing that stops this robot.** `W 1500 1500` is a request over a
>    cable that can be unplugged. The supervisor is software that can hang.
> 2. **Never leave a stalled servo powered.** A stalled MG996R at 6 V draws ~2.5 A and will
>    cook itself and its plastic mount.
> 3. **One change at a time.** If you wire three things then power up, you learn nothing from a
>    failure.

---

## Gate A — Power off: the multimeter checks

Do all of this with **nothing plugged into USB and the supply unplugged**.

| Check | How | Pass criteria |
|---|---|---|
| Fuse | Look at it | Present, DC-rated, **no larger than the smallest wire or board rating** in the chain |
| E-stop | Multimeter continuity across the switch, button released | **Closed (continuity)**. Pressed → **open**. NC switches fail safe: a broken wire also kills the rail |
| OE pull-up | Ohmmeter from GPIO 4 to 3V3 | ~10 kΩ. Without it, a reset leaves OE floating and the servos twitch to a random pose |
| 6 V rail to logic | Ohmmeter from servo V+ to PCA9685 VCC and to 3V3 | **Open circuit.** Any reading at all means a short that will kill the ESP32 |
| Polarity | Trace + and − from supply → fuse → e-stop → INA219 VIN+ ; VIN− → PCA9685 V+ | Correct, and never reversed on the INA219 (it has no protection) |
| Signal grounds | Continuity from ESP32 GND to PCA9685 GND to INA219 GND to supply − | All connected, all near 0 Ω |
| Capacitor | Look at it | 470–1000 µF, ≥10 V, **correct polarity**, across servo V+/GND after the switch |

**Do not continue until every row passes.** A mis-wired 6 V rail is the fastest way to destroy
all three boards at once.

---

## Gate B — USB only: the ESP32 and the I²C bus

Servo rail **still unpowered**. Plug in USB only — it powers logic, never servos.

1. Install the Arduino IDE 2.x, then **Boards Manager → "esp32" by Espressif Systems** (v3.x).
2. Open `firmware\phase3_full_6dof_esp32\phase3_full_6dof_esp32.ino`, select board
   **ESP32 Dev Module** and your COM port, and click Upload.
3. Open **Tools → Serial Monitor**, set **115200** baud and line ending **Newline** (or
   "Both NL & CR" — the parser accepts either).

You should immediately see the boot line:

```json
{"ok":true,"event":"BOOT","fw":"BUDE-ESP32-full-1.0","pca9685":true,"ina219":true,
 "outputs":"off","cliff_pins":[25,26],"cliff_tripped_is_high":true,
 "wheel_channels":[6,7],"recoil_ms":350,"host_timeout_ms":500,"wdt_ms":5000}
```

Type these by hand and read the replies:

| Type | Expect | Meaning |
|---|---|---|
| `PING` | `{"ok":true,"fw":"BUDE-ESP32-full-1.0","pca9685":"0x40","ina219":"0x41",...}` | the board is alive and talking |
| `STATUS` | a long JSON line (see below) | the live state of everything |
| `HELP` | the command list | you know the protocol |

A real `STATUS` looks like this (this is genuine output from the host simulator):

```json
{"ok":true,"fw":"BUDE-ESP32-full-1.0","bus_v":6.00,"current_ma":0.0,"cliff":[0,0],
 "distance_mm":0,"sonar_fresh":false,"sonar_timeouts":0,"estop_latched":false,
 "outputs":"off","loop":1,"wheels":[1500,1500],"recoil_active":false,"recoils":0,
 "reflex_max_us":0,"reflex_last_us":0,"reflex_over_budget":0,"i2c_recoveries":0,
 "i2c_consecutive_failures":0,"wdt_ms":5000,"adc34":2048}
```

**Pass criteria for Gate B**

* `pca9685` and `ina219` are both `true` in the boot line.
* `outputs` is `"off"` — OE is HIGH, the servos cannot move.
* With the 6 V rail off, `bus_v` is near 0 and `current_ma` near 0. **If `bus_v` reads ~6 V while
  your supply is unplugged, stop — something is back-feeding through the USB.**

**If `pca9685` is false:** the I²C bus is not working. Look for `I2C_RECOVER` events (the firmware
clocks SCL nine times and retries — that is FMEA #12 doing its job), then check the pull-ups and
that SDA/SCL are not swapped. `I2C_BUS_DEAD` means three failures in a row and the firmware cut
the OE pin on purpose.

`firmware\phase3_bench_diagnostic\phase3_bench_diagnostic.ino` is the older, simpler bring-up
sketch if you want a second opinion on the bus and the two addresses.

---

## Gate C — One servo, unloaded, one channel

1. Power off. Connect **one** MG996R to PCA9685 CH0, with the horn **removed**.
2. Turn the current limit on your supply down to **1 A** before switching on.
3. Power the servo rail. `STATUS` should show `bus_v` ≈ 6.0 and `current_ma` small.
4. Enable the outputs (`OE 0`), then move it a little:

```
M 0:1600
M 0:1500
```

**Pass criteria**

* The servo moves smoothly and stops when told, and does not buzz or get hot.
* `current_ma` rises while moving and returns to a small value at rest.
* Nothing browns out: `bus_v` stays above 5 V, and there is no `BROWNOUT` event.

**If it buzzes continuously:** it is fighting its own limit or the horn is jammed. Stop, power
down, and check the mechanical assembly. **If the ESP32 resets when the servo moves:** your
supply cannot cover the transient — add or enlarge the bulk capacitor, shorten the power wires,
and use 20 AWG (Gate 5 in Part 1 explains why).

> **`OE 1` is your panic button on the bench**: it tri-states all eight channels. Then fix the
> cause before enabling again.

---

## Gate D — The cliff sensors: polarity first

**Do this before any test that involves driving.** TCRT5000 breakouts ship with two different
output conventions and the difference is not written on the board. The firmware expects:

* rover flat on the desk (floor seen) → `"cliff":[0,0]`
* rover lifted so a sensor sees air → `"cliff":[1,0]` or `[1,1]`

**Procedure**

1. Rest the rover (or just the two sensors, powered) flat on a desk. Type `STATUS`.
2. Slide the module off the edge, or lift that side 20 mm, so the sensor looks into air. `STATUS`
   again.

**Pass criteria**

| Reading | Verdict |
|---|---|
| Desk `[0,0]`, in air `[1,1]` | correct — continue |
| Desk `[0,0]`, in air `[1,0]` | one sensor is dead or misaligned — fix before continuing |
| **Desk `[1,1]`, in air `[0,0]`** | **your modules are inverted** — change one line and reflash |
| Anything that never changes | wiring or power to the sensor — check VCC/GND/OUT |

**If your modules are inverted:** open the sketch, find

```cpp
static const bool CLIFF_TRIPPED_IS_HIGH = true;
```

change `true` to `false`, and reflash. One line; the reads, the forward gate *and* the interrupt
edge all follow it. The boot line tells you which convention is compiled in
(`"cliff_tripped_is_high":true|false`). Both conventions are covered by the test suite, so you are
not on your own.

**An unplugged sensor reads as "cliff"** (the pins are `INPUT_PULLUP`). That is deliberate —
a broken wire leaves the rover refusing to drive forward — but it means **a bench test of the
wheels with no cliff sensors fitted will print `FORWARD_REFUSED` and the wheels will not turn.**
Fit the sensors, or temporarily jumper GPIO 25 and 26 to GND *and remember you have defeated a
safety input*.

---

## Gate E — The rangefinder (HC-SR04)

Front sensor on **GPIO 18 TRIG / GPIO 19 ECHO**, with the ECHO pin level-shifted to 3.3 V.

1. Hold a book or a wall at a measured distance (say 300 mm) in front of the sensor.
2. Watch `distance_mm` in `STATUS` (it refreshes every 60 ms).

**Pass criteria**

* `distance_mm` tracks your tape measure within about 10 mm. The firmware's conversion is
  deliberately ~3 % conservative and rounds in the safe direction.
* `sonar_fresh` is `true` while you are measuring. If it is `false`, the echo is not being heard:
  check TRIG/ECHO, the level shifter, and that the sensor has 5 V.
* Take the book away: within about half a second the reading goes stale (`sonar_fresh:false`)
  and forward commands are refused until you fit the sensor properly. **That is fail-closed
  behaviour, not a fault** — an HC-SR04 that has gone quiet must not be treated as "clear road".

`sonar_timeouts` counting up means no echo came back at all. A few is normal at long range; a
stream of them means a wiring or level-shift problem.

---

## Gate F — Wheels **off the ground**: direction, watchdog, reflex

Prop the rover so both wheels spin freely. Nothing can be damaged by a wrong direction here —
which is exactly why you do it here and not on the floor.

1. `OE 0` to enable.
2. Drive forward slowly:

```
W 1320 1680
```

3. Stop:

```
W 1500 1500
```

**Pass criteria**

| Test | Command | Expect |
|---|---|---|
| Forward | `W 1320 1680` | both wheels turn so the rover would move **away from you** |
| Reverse | `W 1680 1320` | both wheels turn the other way |
| Stop | `W 1500 1500` | both wheels actually stop, not creep |
| **500 ms watchdog** | `W 1320 1680` then type nothing for a second | within ~0.5 s the wheels stop and you get `WHEEL_TIMEOUT` — the laptop going quiet cannot leave the rover driving |
| **The reflex** | `W 1320 1680`, then slide a card under one sensor so it sees air | both wheels **reverse** and you get `CLIFF_REFLEX` with `latency_us` under 5000 |

The `CLIFF_REFLEX` line looks like this:

```json
{"ok":true,"event":"CLIFF_REFLEX","left_us":1680,"right_us":1320,"latency_us":237,"over_budget":false}
```

**`latency_us` is measured on the ESP32 itself**, from the GPIO interrupt to the I²C write — it
is the one real measurement this project will give you. `over_budget:true` would mean it missed
the 5 ms deadline; that number is in `STATUS` as `reflex_max_us` too.

**If the wheels turn the wrong way:** the two drive servos face each other, so the *same* pulse
width turns them opposite ways. If forward is backwards, flip both flags in
`config/robot_config.yaml` (`wheel_left_forward_above_neutral` / `wheel_right_forward_above_neutral`),
mirror them in the sketch, and reflash. **Do not test on the floor until this reads correctly** —
a wrong sign turns the cliff reflex into the cliff accelerator (FMEA #15).

---

## Gate G — The floor test (the one that matters)

Now you find out whether the reflex works on a real floor.

**Setup**

* Contained area: a rug, or a table edge with a cushion below.
* Arm stowed or removed. Nothing fragile on the rover.
* Rover on the floor, **2–3 cm of wheel travel to the edge** — a short runway so it cannot reach
  the edge before you can react.
* Your foot on the supply switch and your hand near the rover.
* Current limit still low (1–2 A).

**Procedure**

1. `OE 0`.
2. `W 1320 1680` — drive slowly toward the edge.
3. Let the front sensor pass over the edge.

**Pass criteria**

* The wheels reverse for ~350 ms and then stop, and you see `CLIFF_REFLEX` with
  `latency_us` under 5000.
* The rover does **not** drive off the edge.
* Repeat 3–5 times from different angles, including **one sensor only** (drive the rover at the
  edge at an angle so just the left sensor goes over).

**If it drives off:** stop and check, in this order — (1) the sensor polarity (Gate D), (2) that
the sensor is actually over the leading edge and not 30 mm behind it, (3) that the wheel direction
is right (Gate F) — a "reversal" that is really forward will drive it off the far side.

**If `latency_us` exceeds 5000:** the reflex still worked, but it missed its deadline. Report it
and check for a slow I²C bus or a very long `Wire` transaction; the number is stored in
`reflex_max_us` for you.

---

## Gate H — The arm, joint by joint

Withdraw the wheels (or leave the rover parked) and unpower the rover motors.

1. **Unloaded, one joint at a time.** For each channel 0…5:

```
M 0:1400
M 0:1500
M 1:1400
M 1:1500
...
```

Watch for: buzzing, heat, a joint that moves the wrong way, or a joint that jams. Stop at the
first one that misbehaves. A hot servo is a stalled servo — power off.

2. **Small blended moves**, which is how the real robot moves:

```
G 1500 0:1200,1:1300
```

3. **Loaded**, with a light object (a screwdriver, ≤100 g). The torque budget says the shoulder is
   at 42 % of stall for 100 g at full reach — if it buzzes or sags, stop and check the object mass
   and the joint angle.

4. **Home position** and back off:

```
G 1500 0:1500,1:1250,2:1944,3:1221,4:1500,5:1393
```

(the frame the Python driver prints for `--home`)

5. **Watch `current_ma`** in `STATUS` while a joint holds a load. `OVERCURRENT_MA = 2600` with a
   150 ms hold is the Phase 8 supervisor's limit; `ina219_arm_trip_current_a: 3.2` is the config's
   trip. The firmware itself does not cut torque on current — that policy lives in the Python
   supervisor, so **on the bench with no laptop attached, nothing is watching the current except
   you.**

**After an e-stop, read this:** when the physical e-stop opens, the INA219 sees ~0 V, the
firmware prints `ESTOP_LATCHED` and cuts the outputs — and it **will not re-enable itself**. Even
after you release the e-stop, `OE 0` is refused until you reboot the ESP32 (press EN, or unplug
USB). That is deliberate: the firmware does not get to decide that the e-stop event is over.

---

## Gate I — The camera

The model's intrinsics (`fx = fy = 532.57` at 640×480) are a *design number*, not a
calibration. Do it properly once the camera is mounted:

1. Print a chessboard, 9×6 inner squares, and **measure the square** with calipers.
2. Take 15–20 photos of it from different angles and distances, filling much of the frame.
3. Put them in a folder and run:

```powershell
python src\phase6_perception\camera_and_hand_eye_calib.py calibrate --images "calibration_photos\*.png" --square-mm 24.8 --cols 9 --rows 6 --output camera_intrinsics.json
```

**Pass criteria:** it writes `camera_intrinsics.json` and reports a reprojection error. Under about
1 pixel is a usable calibration; several pixels means blurry or too-few photos.

Then check the hand-eye relationship: put a known object on the desk (say at 185, 0 mm) and
confirm the pixel it lands on matches `/api/plan` in the viewer. `CAMERA_MOUNT_OFFSET_MM` is
`(0, 30, −43)`; if every target is off by a constant distance, that offset is what to adjust.

---

## Gate J — End to end

Everything at once, with the e-stop within reach and the wheels **off the ground** for the first
attempt:

1. `python src\phase4_simulation\web_sim_server.py`, open the viewer, and watch the model mirror
   the real commands.
2. From the driver:

```powershell
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --status
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --home --yes
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --open 100 --yes
```

3. Put the rover on the floor with a clear 1 m runway and drive it with:

```powershell
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --drive 0.35 0.35 --yes
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --stop --yes
```

Remember `--stop` is a **request**: if your laptop crashes mid-drive, the pilot's safety net is
the ESP32's own 500 ms timeout, and the final one is the physical e-stop.

4. Voice, if you have the Phase 7 path wired up:

```powershell
python src\phase7_voice_personality\voice_assistant.py --demo
python src\phase7_voice_personality\voice_assistant.py --command "get me the screwdriver"
```

---

# What you will then have proven

| Claim | Proven by |
|---|---|
| The code runs and its parts agree with each other | Part 1, Steps 3–10 |
| The parts are geometrically consistent and printable | Part 1, Step 6 |
| The firmware's safety logic executes correctly | Part 1, Step 7 |
| The ESP32 talks, the bus works, both chips answer | Gate B |
| One servo moves under control without browning out the rail | Gate C |
| The cliff sensors read the floor the way the firmware expects | Gate D |
| The rangefinder measures real distance and fails closed | Gate E |
| The wheels turn the right way, stop on command, time out, and reverse on a cliff | Gate F |
| The reflex actually saves the rover at a real edge | Gate G |
| The arm moves all six joints without stalling or overheating | Gate H |
| The camera measures the world | Gate I |

# What none of it proves

* **The servo torque figures are datasheets.** Your MG996R is not a datasheet MG996R. The
  shoulder's 2.38× margin is arithmetic, not a measurement — verify it by lifting the real load.
* **The ESP32 sketch has never been flashed by me.** Everything before Gate B is a simulation of
  its logic, however faithful.
* **`< 5 ms` reflex is a requirement, not a promise.** Gate F/G is where you find out whether
  your build meets it; the firmware measures and reports the real number.
* **The arm has never held anything.** 100 g is the design payload; the clamp's grip on a real
  screwdriver is unverified until Gate H.
* **Nothing has been drop-tested.** Printed PLA links, fatigue and impact are outside everything
  above.
* **The budget is a model.** $58.50 / $66.50 excludes shipping, taxes, a fuse holder, the
  capacitor, wire and the revised mounting hardware. The BOM currently totals $59.00 / $67.00.

---

# Quick reference — every command in one block

```powershell
# ---- Part 1, on the laptop -------------------------------------------------
python check_setup.py                                                # first: is my setup right?
python src\phase8_safety_testing\run_phase_walkthrough.py            # all 18 phase commands
python -m pytest -v                                                  # the whole suite
python src\phase8_safety_testing\run_full_acceptance_suite.py        # 50 checks, the scorecard
python src\phase8_safety_testing\run_full_acceptance_suite.py --quick
python src\phase1_sizing\torque_and_power_calc.py
python src\phase1_sizing\single_joint_ping.py --dry-run
python src\phase2_mechanical\generate_stl_parts.py
python src\phase2_mechanical\verify_print_parts.py
python src\phase2_mechanical\verify_assembly_fit.py                  # 212 / 212
python src\phase2_mechanical\mesh_audit.py
python src\phase3_electronics\power_and_wiring_verifier.py           # 18 checks
python src\phase3_electronics\esp32_arm_driver.py --dry-run --ping --home
python src\phase4_simulation\verify_urdf_model.py                    # 10 checks
python src\phase4_simulation\sim_runner.py --backend kinematic
python src\phase4_simulation\web_sim_server.py                       # then http://127.0.0.1:8080/
python src\phase5_control\kinematics.py
python src\phase5_control\trajectory_and_collision.py
python src\phase5_control\ros2_moveit_bridge.py --dry-run
python src\phase6_perception\camera_and_hand_eye_calib.py
python src\phase6_perception\vision_mobile_navigator.py
python src\phase7_voice_personality\personality_engine.py
python src\phase7_voice_personality\voice_assistant.py --demo
python src\phase8_safety_testing\safety_supervisor_and_logger.py --fault all --seconds 6 --no-log

g++ -std=c++11 -Wall -Wextra -Wpedantic -o reflex.exe firmware\phase3_full_6dof_esp32\test_reflex_logic.cpp ; .\reflex.exe
g++ -std=c++11 -Wall -Wextra -I tests\firmware_host -o fw.exe tests\firmware_host\main.cpp ; .\fw.exe

# ---- Part 2, with hardware ------------------------------------------------
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --status
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --home --yes
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --drive 0.35 0.35 --yes
python src\phase3_electronics\esp32_arm_driver.py --port COM3 --stop --yes
# serial monitor at 115200, or by hand:
#   PING | STATUS | HELP
#   W 1320 1680      forward          W 1500 1500   stop
#   W 1680 1320      reverse          OE 1 / OE 0   outputs off / on
#   M 0:1600         immediate        G 1500 0:1200 blend over 1500 ms
```

**When you are done, tell me which gate failed first and what it printed.** That is far more
useful to me than "it works" — the build notes in `docs/verification_status.md` exist precisely so
that a failure on your bench becomes a fix in the repo.
