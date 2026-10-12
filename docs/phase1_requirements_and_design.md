> **STOP: ARCHIVED DESIGN ESTIMATES, NOT VERIFIED BUILD INSTRUCTIONS.** Existing Phase 2 STLs have unverified and likely nonfunctional mating features; price/payload/current-safety claims below are not validated. Read [verification status](verification_status.md) and the revised Phase 3–6 guides before using any hardware.

# Phase 1 — Requirements, Naming, Architecture & System Design (Sub-$75 3D-Printed Edition)

## 1. Robot Naming & Personality Identity
Inspired by Tony Stark's **DUM-E** (and its workshop sibling **U**), our desktop assistant is an original, compact, expressive 3D-printed arm with an eye-in-hand camera "head" that tilts, nods, perks up, and droops when a grasp fails.

### Top 5 Name Candidates
| Name | Backronym | Personality & Wake-Word Vibe |
|---|---|---|
| **B.U.D.-E** *(Default)* | **B**enchtop **U**tility & **D**exterity **E**ngine | Eager workshop sidekick; perks up on *"Hey Bud!"* and double-clicks its claw happily before handing you a screwdriver. |
| **F.U.M.B.L.-E** | **F**lexible **U**tility **M**anipulator for **B**enchtop **L**ab **E**xperiments | Pure DUM-E energy; tries its absolute best, and if it misses a screw, performs a dramatic slow-motion head droop. |
| **C.L.A.W.-E** | **C**ompact **L**ab **A**ssistant & **W**orkshop **E**xecutor | Crisp, curious, slightly proud of its pinchers; greets you with a quick crab-snap and 35° head tilt. |
| **G.I.Z.M.O.** | **G**eneral **I**ntelligent **Z**ero-cost **M**anipulator **O**rganism | Bird-like, inquisitive desk gremlin; tracks your hand smoothly and cocks its wrist camera whenever a new tool appears. |
| **P.I.P.** | **P**ersonal **I**nteractive **P**icker | Short, punchy one-syllable wake word (*"Hey Pip!"*); fits the compact `22.5 cm` footprint. |

---

## 2. How We Get the Full Build Under $75 (`$58.50` Total!)
To slash the total cost from `~$200` down to **`$58.50`** without losing 5-DOF + Gripper dexterity, vision, voice, or safety, we apply four clever engineering tricks:

1. **Hybrid Standard + Micro Metal-Gear Servos (`$19.50` total for all 6 motors!)**:
   - **J1 (Base Yaw), J2 (Shoulder Pitch), J3 (Elbow Pitch)**: **3× `MG996R` (or `DS3218`) Metal-Gear Standard Servos** (`$12.75` for 3).
   - **J4 (Wrist Pitch), J5 (Wrist Roll), J6 (Claw Gripper)**: **3× `MG90S` All-Metal-Gear Micro Servos** (`$6.75` for 3).
   - *Why this works so well*: Each `MG90S` weighs only **`13.4 g`** (compared to `55 g` for a standard servo). Putting 3 micro servos at the wrist cuts distal weight by **75% (`-125 g`)**, meaning the shoulder and elbow servos easily lift the arm + a `100–160 g` payload!
2. **Adding "Smart Servo" Safety to Cheap PWM Servos with a `$2.50` `INA219` Current Sensor**:
   - Normal hobby PWM servos are "blind" to overload and stall current.
   - By wiring a **`$2.50` `INA219` I2C High-Side Current Sensor** into the `6V` servo power rail connected to an **`$4.50` `ESP32`** and **`$3.00` `PCA9685`**, the ESP32 monitors real-time servo current at `200 Hz`:
     - When the claw closes around a screwdriver or cup and current rises above `450 mA`, the ESP32 **automatically stops closing** so it never crushes a cup or burns out the `MG90S`!
     - If the arm bumps into your hand and current spikes above `3.2 A`, the ESP32 triggers an instant software E-stop!
     - *(Optional $0 upgrade in Phase 3: Soldering a single wire to the internal potentiometer wiper of the servos feeds analog joint angles into the ESP32's 12-bit ADC pins!)*
3. **100 Hz On-Board Cosine Smoothing on the ESP32**:
   - Hobby servos look "jerky" only when commanded with step changes. Our ESP32 firmware interpolates every move at `100 Hz` (`10 ms` ticks) using smooth cosine velocity profiles, giving expressive DUM-E head-tilts and nods.
4. **3D-Printing the Mechanical Hardware (`$0` extra vs. `$28+` of store-bought parts)**:
   - **3D-Printed Heavy-Duty Desk C-Clamp** (with printed ACME thread screw) replaces a bought cast-iron clamp.
   - **3D-Printed Slewing Bearing** at the base waist (using printed races + $0.50 of 6mm airsoft BBs or printed rollers) replaces expensive steel bearings.
   - **3D-Printed Captive M3 Hex-Nut Traps** replace brass heat-set inserts.
   - **3D-Printed Shoulder Gravity-Assist Spring/Band Hook** lets you loop a standard household rubber band or extension spring behind J2, cutting shoulder static gravity load in half!

---

## 3. Kinematic Architecture (`22.5 cm` Compact Reach, 5-DOF + Gripper)

```text
                       [J4: Wrist Pitch (MG90S)] ---> Nods "yes", looks down at desk or up at face
                              |
              L3 = 75 mm      v    L4+L5 = 70 mm (includes 6g USB Camera "Eyes" + Geared Claw)
         o================o=======o=====[J5: Wrist Roll (MG90S)] ====< [J6: Claw Gripper (MG90S)]
        /  [J3: Elbow Pitch (MG996R)]      (Tilts head 35° like a curious puppy & aligns with tools)
       /
      /  L2 = 80 mm
     /
    o  [J2: Shoulder Pitch (MG996R/DS3218 + 3D-printed rubber-band assist lug)]
    |
    |  L1 = 55 mm
  [===] [J1: Base Waist Yaw (MG996R + 3D-printed slewing ring)]
 ═══════ Desk Edge (Secured with 3D-Printed C-Clamp)
```

---

## 4. Torque & Power Budget Summary (`src/phase1_sizing/torque_and_power_calc.py`)
- **Total Horizontal Reach**: `225 mm` (`22.5 cm`)
- **Moving Arm Mass**: Only **`170 g`** (including J3 `MG996R`, J4–J6 `MG90S`, 3D-printed links, claw, and `6 g` camera)
- **Total 3D-Printed PLA Needed**: **`~210 g`** (`~$4.00` of filament from an existing spool)
- **Nominal Payload (`100 g` at full `225 mm` horizontal stretch, with 25% dynamic margin)**:
  - **J2 Shoulder (`MG996R`/`DS3218`)**: `0.535 N·m` (`42.0%` of stall → **`2.38×` safety factor**, or **`>4.5×`** with a rubber band hooked to the printed shoulder assist lug!)
  - **J3 Elbow (`MG996R`)**: `0.281 N·m` (`22.0%` of stall → **`4.54×` safety factor**)
  - **J4 Wrist Pitch (`MG90S`)**: `0.111 N·m` (`51.5%` of stall → **`1.94×` safety factor**)
- **Electrical Power Budget (`6.0V 5A` Regulated Brick)**:
  - Idle holding: `0.35 A` (`2.1 W`)
  - Normal gesture / tracking: `1.25 A` (`7.5 W`)
  - Full-reach `100 g` pick & handover: `1.85 A` (`11.1 W`)
  - `INA219` Software Over-Current Cutoff: `3.20 A` (`19.2 W` — trips in `< 5 ms` before the `5.0A` PSU can overload).
