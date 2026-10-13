# Phase 7 — Voice Control & Personality

B.U.D.-E should look like it is paying attention, not like a stepper driver. This phase adds
the two halves of that: **body language** that is smooth enough not to shake itself apart, and
a **voice front end** that works with no network, no microphone and no model installed.

| File | What it is |
|---|---|
| `src/phase7_voice_personality/personality_engine.py` | The gesture engine: 100 Hz minimum-jerk keyframes, 8 channels, and the director that picks a gesture from an event |
| `src/phase7_voice_personality/voice_assistant.py` | Optional Whisper session + deterministic text-to-intent parser + the assistant that drives Phase 6 |
| `src/phase7_voice_personality/__init__.py` | The public surface |
| `src/phase4_simulation/web_assets/personality.js` | The browser panel that plays the same frames on the drawn arm |

Try it right now, offline:

```bash
python3 src/phase7_voice_personality/personality_engine.py        # validates every gesture
python3 src/phase7_voice_personality/voice_assistant.py           # seven-turn demo
python3 src/phase7_voice_personality/voice_assistant.py -c "hand me the screwdriver"
python3 src/phase7_voice_personality/voice_assistant.py --list
python3 src/phase4_simulation/web_sim_server.py                   # then the panel, in-browser
```

---

## 1. The eight channels and why minimum jerk

```
index  0..5   base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, jaw   [degrees]
index  6..7   left wheel, right wheel                                              [microseconds]
```

The wheels take pulse widths, not angles, because the two drive servos are
continuous-rotation: **1500 µs is stop**, 1000 µs full reverse, 2000 µs full forward.

Every gesture is a list of keyframes and every segment between two keyframes is

```
q(tau) = q0 + dq * (10 tau^3 - 15 tau^4 + 6 tau^5),   tau = t / T
```

That quintic is the *unique* curve that covers the distance with zero velocity **and** zero
acceleration at both ends. Because each segment starts and ends at rest, keyframes chain
without a velocity step, so a whole gesture — including its reversals — is one smooth motion
instead of a series of slams. A step command to a hobby servo is what breaks gear teeth and
loosens printed hubs.

The peak speed of that curve is `1.875 * |dq| / T`. That is the same rule Phase 5's planner
uses (`T >= 1.875 * |dq| / v_max`), and it is what makes an infeasible gesture a build error
rather than a burnt servo: **every keyframe segment is validated at construction time** and a
too-fast gesture raises `ValueError` instead of running.

### Two speed budgets, deliberately

| Budget | Channels (deg/s) | Where it applies |
|---|---|---|
| Transport (Phase 5 `VELOCITY_LIMITS_DEG_S`) | 35, 25, 35, 40, 40 | Moving a *tool* across the desk. A slip drops something |
| Gesture (`GESTURE_SPEED_DEG_S`) | 70, 55, 70, 90, 90, 90 | Short, small, empty-handed expressions |
| `Sequence.speed_scale` | ×1.4 nod, ×1.5 wiggle, ×2.0 startle | On top of the gesture budget |

An MG996R at 6 V turns about 430 °/s with no load, so even the startle reflex peaks near a
third of the servo's capability — but the printed arm's inertia has never been measured.
**Start at `speed_scale` 0.5** and listen to the servos before you trust the table.

## 2. The six gestures

| Gesture | Length | Frames | Trigger | Priority | Peak joint speed (deg/s) |
|---|---:|---:|---|---:|---|
| `curious_head_tilt` | 1.35 s | 136 | wake word, new object | 3 | roll 66, pitch 24, yaw 22 |
| `affirmative_nod` | 1.76 s | 177 | command acknowledged | 2 | pitch 119 (double nod on J4) |
| `sad_failed_grasp_droop` | 3.40 s | 341 | closed jaw < 180 mA | 2 | jaw 82, elbow 69, yaw 66 |
| `excited_success_wiggle` | 2.10 s | 211 | grasp confirmed | 2 | roll 112, yaw 103 |
| `startled_cliff_recoil` | 1.95 s | 196 | TCRT5000 cliff trip | **0** | yaw 131, elbow 117, jaw 117 |
| `idle_breathing_and_look_around` | 8.00 s | 801 | idle > 10 s, loops | 3 | yaw 59 at 0.25 Hz breathing |

Details that matter:

* **The nod is a double nod on J4** (`wrist_pitch`), 20° down, 35° up, 20° down, settle.
* **The droop is slow on purpose** (2.2 s to reach the sag). The repository's `sad_droop` pose
  `(0, 25.8, 84.8, 39.4, 0)` is where it ends, so the viewer, the MoveIt draft and the
  personality engine all agree on what "sad" looks like.
* **The wiggle keeps the jaw shut** (2°) so a confirmed grasp is not thrown away mid-celebration.
* **The cliff recoil splits the reflex from the body.** The wheels are commanded full reverse
  for 0.27 s (40 mm at the declared 150 mm/s) and stop 0.15 s later; the body pose takes 0.75 s
  to get there. A printed arm cannot snap 25° in a quarter second and should not try.
* **Idle breathing is 0.25 Hz** — a 4 s period, ±2.5° on J2/J4 — with a slow glance left and
  right. It loops until any other event wins.

Priority rules: **0 interrupts everything** (the cliff reflex), 2 is an acknowledgement or a
grasp outcome, 3 is attention and idling. A gesture already playing is not interrupted by one
of equal or lower priority.

### When is a grasp a miss?

```python
looks_like_failed_grasp(current_ma, jaw_target_deg, jaw_deg, held_s)
```

True when the jaw has been commanded shut (`<= 5°`), the jaw is actually near shut (`<= 12°`),
it has held for at least 0.35 s, **and** the INA219 reads less than **180 mA**. An MG90S
closing on air draws a few tens of mA; squeezing a 6 mm screwdriver shaft draws several
hundred. The test is one-sided: it reports a *miss*, never a *success*, because a jammed jaw
also draws nothing. Phase 8 turns this into the retry policy.

### The halt is not a gesture

`PersonalityDirector.halt(t)` returns one immediate frame: both wheels to 1500 µs and the arm
frozen at its last commanded pose, latched until `resume()`. There is **no** minimum-jerk
blend on a halt — blending would keep the wheels turning *after* you asked them to stop. The
wheel slew limit (`WHEEL_SLEW_US_S`) deliberately does not apply to a stop or a launch, only
to slamming straight from forward to reverse.

## 3. Voice

### Offline first, and three ways in

| Path | Needs | Latency on a laptop |
|---|---|---|
| `--command "stop"` / `--demo` | nothing at all (stdlib) | instant |
| `--transcribe file.wav` | `faster-whisper` or `openai-whisper` | ~0.3–1 s for 4 s of audio |
| `--mic --seconds 4` | a whisper package **and** `sounddevice` (PortAudio) | ~1–2 s including the recording |

```bash
python -m pip install faster-whisper sounddevice soundfile     # int8 CPU, the fast path
python -m pip install openai-whisper sounddevice soundfile     # reference implementation
python3 src/phase7_voice_personality/voice_assistant.py --mic --seconds 4
python3 src/phase7_voice_personality/voice_assistant.py --model base.en --mic   # noisy rooms
```

`tiny.en` (75 MB) is the default; `base.en` (145 MB) is noticeably better with background
noise. The model is downloaded on first use and then runs offline. **The microphone is a
privacy surface**: this code records one utterance when you ask for it and never opens a
background stream.

If no backend is installed the CLI prints the two `pip` lines and exits 3. It never falls back
to a cloud API, because a robot that stops working when the wifi drops is worse than one that
never had wifi.

### Intents

| You say | Intent | What happens |
|---|---|---|
| *"hand me the screwdriver"*, *"fetch the allen key"*, *"get me the pliers"* | `FETCH_TOOL` | nod → **stow the arm** → Phase 6 navigator drives to the object → wiggle |
| *"look at my hand"*, *"watch my hand"* | `LOOK_AT_HAND` | head tilt + Phase 6 PCA hand-roll estimator armed |
| *"follow me"*, *"come here"* | `FOLLOW_ME` | follow policy, with the cliff reflex on every tick |
| *"stop"*, *"freeze"*, *"halt"*, *"abort"*, *"e-stop"* | `HALT` | immediate: wheels 1500 µs, arm pose held |
| *"go home"*, *"sleep"*, *"park"* | `GO_HOME` | `home_perched` `(0, -45, 75, -60, 0, 0)`, wheels stopped |
| *"hey bud-e"* | `WAKE` | curious head tilt |
| *"thank you"*, *"good job"* | `PRAISE` | excited wiggle |
| anything else | `UNKNOWN` | says so, suggests the nearest known phrase, **moves nothing** |

The parser is pure stdlib and deterministic: same sentence, same intent, every time. Matching
is word-boundary based, so *"screwdriver"* never trips *"stop"*. `HALT` is checked **first**,
against the raw text, before any other pattern can claim the sentence — *"bud-e stop, the
screwdriver is fine"* stops.

Three tools also map from what people actually say: `allen key`/`hex key`/`allen wrench`,
`screwdriver`/`phillips`/`driver`, `pliers`/`wire cutters`/`nippers`.

### The fetch flow, step by step

1. `affirmative_nod` (1.76 s) — it heard you.
2. Stow to `home_perched`. **Phase 6's `choose_wheels` returns `(0, 0)` unless the arm is
   stowed**, so this is not decoration; it is the interlock.
3. `plan_fetch_sequence(object_mm)` runs the fail-closed drive policy to a standoff.
4. Grasp, then check the jaw current. Under 180 mA → `sad_failed_grasp_droop` and (Phase 8) one
   retry. Over → `excited_success_wiggle`.

## 4. In the browser

`python3 src/phase4_simulation/web_sim_server.py` and open the viewer. The **Voice &
personality** panel:

* one button per gesture, labelled with its length and whether it is a reflex;
* `Say it` runs a typed sentence through the real parser and prints the intent, the confidence,
  the evidence, the phrase the simulated robot *would* say, the action log, and the gestures;
* preset chips for the known phrases;
* `Stop` clears the plan, hides the rover and halts the viewer — labelled as a **viewer**
  stop, because the only real stop is the physical e-stop.

The panel plays the server's own 100 Hz samples through `window.BUDE.setJoints()`, the same
joints the sliders drive, so the drawn arm and the command stream cannot disagree.

## 5. Safety — read this before connecting servos

* **A voice "stop" is a convenience, not a safety device.** It depends on a microphone, a
  model, a network stack and a Python process. The safety device is the LA38-11ZS e-stop wired
  to GPIO 4 through the PCA9685 OE pin, which cuts the servo rail in hardware.
* The first gestures on real hardware: arm **unloaded**, no tool in the jaw, one hand on the
  e-stop, `speed_scale` 0.5, 2 s per gesture minimum. Only then try the full-speed table.
* The rover gestures (`startled_cliff_recoil`, `FOLLOW_ME`) move **wheels**. Test on the
  **floor inside a contained area**, arm stowed, never on a desk or a table edge. There is no
  braking-distance, traction or tipping model anywhere in this repository.
* The wheel commands are **open loop**. 40 mm of reverse assumes 150 mm/s at full command,
  which is an estimate from the 48 mm wheel and a ~90 RPM CR servo. Calibrate it with a tape
  measure: command reverse for 1 s, measure the distance, and fix `ROVER_MM_S_AT_FULL`.
* Continuous-rotation servos rarely stop at exactly 1500 µs. Find each wheel's dead band
  before any autonomous move, or "stop" will mean "creep".
* Whisper downloads its model from the internet **on first use**. Do that on the bench, not
  while the robot is balancing on a table edge.

## 6. What is not demonstrated

* No gesture has ever been run on a servo. The angles are model-validated against declared
  speed limits and nothing else.
* Collision checking: the Phase 5 keep-outs are for the arm's *task* poses. A gesture is
  checked against nothing — `curious_head_tilt` swings J5 by 35° with no obstacle test.
* The `sad_droop`/`home_perched` poses are declarations. `home_perched` in particular has not
  been checked against the rover deck for interference.
* No latency budget: `Assistant.handle()` is instant, but from speech to first servo command
  there is an unmeasured path through the model, the parser and the serial link. Measure it
  before claiming any response time.
