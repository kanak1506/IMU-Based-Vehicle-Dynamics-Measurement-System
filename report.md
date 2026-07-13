# Vehicle Dynamics IMU Project — Project Report

**Author:** Kanak Potdar
**Location:** `OneDrive/Documents/Arduino`
**Report date:** 2026-07-10

---

## 1. Project Overview

This project builds a complete pipeline for measuring **vehicle handling dynamics** — chassis roll, yaw rate, and lateral/longitudinal acceleration — using a 9-axis IMU mounted in a test vehicle, and analyzing the recorded motion against the **ISO 4138:2021** ("Passenger cars — Steady-state circular test procedures") standard.

The work spans four layers, all built from scratch in this project:

1. **Custom hardware** — a 2-layer PCB designed in KiCad that mounts an ESP32 and a BNO085 IMU breakout as a matched shield.
2. **Embedded firmware** — seven progressive ESP32 sketches (Step01–Step07) that bring up I2C communication with the BNO085, handle sensor resets, synchronize multi-sensor output, and stream a clean CSV over serial.
3. **Python analysis pipeline** — four scripts (Step08–Step11) that log the serial stream, plot raw sensor health, convert quaternions into vehicle-frame dynamics, and run ISO 4138 steady-state regression to compute Roll Gradient and Yaw Rate Gradient.
4. **Streamlit dashboard** — a browser-based control panel that runs the whole pipeline (start/stop logging, vehicle-parameter presets, live results) without touching a terminal.

A companion 12-chapter engineering handbook (`Vehicle_Dynamics_IMU_Handbook.md`) documents the underlying theory (I2C protocol timing, MEMS sensor physics, quaternion math, Kalman fusion, ISO 4138 derivations) — this report instead documents **what was actually built and where it stands**.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          CUSTOM PCB (KiCad)                         │
│   ESP32 Dev Module  ──I2C (GPIO22/21, 400kHz)──  BNO085 IMU (0x4B)  │
└───────────────────────────────┬───────────────────────────────────┘
                                 │ USB Serial (115200 baud, CSV)
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         PYTHON / STREAMLIT HOST                     │
│  step08 (serial logger) → imu_log_*.csv                             │
│  step09 (raw sensor plots) → step10 (vehicle dynamics) → step11     │
│  (ISO 4138 steady-state regression: Roll Gradient, Yaw Rate Grad.)  │
│  dashboard/  → app.py  (Streamlit UI orchestrating all of the above)│
└─────────────────────────────────────────────────────────────────────┘
```

**Data flow:** physical motion → BNO085 MEMS + on-chip EKF sensor fusion → SH-2/SHTP packets over I2C → ESP32 synchronization gate → CSV over USB serial → Python logger → filtering/Euler conversion → steady-state window detection → linear regression → dashboard display.

---

## 3. Hardware

| Component | Role |
|---|---|
| **ESP32 Dev Module** | I2C master, 240 MHz dual-core, runs firmware Step01–Step07, streams CSV over USB serial |
| **BNO085 IMU** (7Semi ES-12242 breakout) | 9-axis sensor-in-package (accel + gyro + magnetometer) with an on-board Cortex-M0+ running Hillcrest/CEVA's SH-2 sensor-fusion firmware; outputs a fused orientation quaternion and gravity-compensated linear acceleration, so the ESP32 never has to run its own Kalman filter |
| **Custom carrier PCB** | Designed in KiCad (see §4) — mounts the ESP32 and BNO085 with correct I2C pull-ups, decoupling, and address/protocol strapping |

**Wiring (as implemented, intentionally non-default):**
- SDA → ESP32 GPIO 22, SCL → ESP32 GPIO 21 (swapped vs. ESP32's usual default pins, to match the routed PCB)
- SA0 → VCC → I2C address `0x4B`
- PS0, PS1 → GND → I2C protocol mode selected
- I2C clock: 400 kHz (Fast Mode)

Two sensor datasheets and a sensor-selection writeup are kept alongside the code: `7Semi ES-12242_BNO085_1.0.pdf`, `BNO080_085-Datasheet.pdf`, and `Comparision of two sensor.pdf` (the comparison document backing the decision to use the BNO085 over an alternative IMU).

---

## 4. PCB Design (KiCad)

A custom 2-layer shield, **`esp32_bno085_shield`**, was designed in **KiCad 10.0.4 (Pcbnew)** to carry the ESP32 dev module and the BNO085 breakout as a single rigid assembly instead of loose dupont wires — eliminating the long-wire I2C capacitance and intermittent-connector failure modes that a bench breadboard setup would have.

### Board specification (from the exported fabrication job file)

| Property | Value |
|---|---|
| Board outline | 77.55 mm × 65.55 mm |
| Layers | 2 (F.Cu / B.Cu) |
| Board thickness | 1.6 mm |
| Copper weight | 0.035 mm (1 oz) per layer |
| Dielectric core | FR4, 1.51 mm |
| Solder mask | 0.01 mm, top and bottom |
| Surface finish | None specified |
| Min track / clearance | 0.2 mm (pad-to-pad, pad-to-track, track-to-track) on outer layers |
| Design date | 2026-07-06 |

### Stack-up
```
Top Silkscreen
Top Solder Paste
Top Solder Mask     (0.01 mm)
F.Cu                (0.035 mm)   ← Layer 1
FR4 core            (1.51 mm)
B.Cu                (0.035 mm)   ← Layer 2
Bottom Solder Mask  (0.01 mm)
Bottom Solder Paste
Bottom Silkscreen
```

### Electrical design intent
The layout implements the I2C interface circuit documented in the project handbook:
- 10 kΩ pull-up resistors on SDA and SCL, tied to the 3.3 V rail, sized to keep signal rise-time compatible with 400 kHz Fast-Mode I2C given the board's trace capacitance.
- 0.1 µF and 10 µF decoupling capacitors placed close to the BNO085's VCC/GND pins to suppress switching noise from the EKF core.
- SA0 strapped high (address `0x4B`) and PS0/PS1 strapped to ground (I2C protocol select) directly on-board, removing manual jumper wiring.
- Routed SDA/SCL matching the firmware's GPIO 22/21 pin mapping.

### Manufacturing outputs
The project directory holds the complete **fabrication-ready output set** (`Gerber_PCB/`, zipped as `Gerber_PCB.zip`) generated by KiCad's plot function, ready to submit to a board house:

- `esp32_bno085_shield-F_Cu.gbr` / `-B_Cu.gbr` — copper layers
- `esp32_bno085_shield-F_Mask.gbr` / `-B_Mask.gbr` — solder mask
- `esp32_bno085_shield-F_Paste.gbr` / `-B_Paste.gbr` — solder paste stencil
- `esp32_bno085_shield-F_Silkscreen.gbr` / `-B_Silkscreen.gbr` — legend/silkscreen
- `esp32_bno085_shield-Edge_Cuts.gbr` — board outline
- `esp32_bno085_shield-PTH.drl` / `-NPTH.drl` — plated / non-plated drill files
- `esp32_bno085_shield-job.gbrjob` — the Gerber job file describing the full stack-up (source of the table above)

> Note: only the exported Gerber/drill fabrication set is present in this workspace — the editable KiCad source (`.kicad_pro` / `.kicad_sch` / `.kicad_pcb`) was not found on this machine at the time of writing. If those files live elsewhere (another machine, a KiCad project folder, cloud backup), they should be located and archived alongside this Gerber set so the board can be revised later.

---

## 5. Embedded Firmware — ESP32 (Step01–Step07)

Built incrementally, each step adding one capability and being verified on hardware before moving on. All sketches live as individual Arduino projects (`StepNN_.../*.ino`) and use the **SparkFun BNO08x Cortex-Based IMU** Arduino library (vendored under `libraries/`).

| Step | Sketch | What it adds |
|---|---|---|
| 01 | `Step01_BNO085_Initialization` | Brings up I2C (`Wire.begin(22,21)`, 400 kHz), opens an SH-2 session with the BNO085 at `0x4B`, and reads back firmware/product-ID metadata to confirm the physical link works |
| 02 | `Step02_Read_SH2` | Adds reset recovery — automotive 12V systems brown-out during engine cranking, so `imu.wasReset()` is polled every loop and all sensor reports are automatically re-enabled if the chip resets mid-drive |
| 03 | `Step03_Enable_Reports` | Enables the three SH-2 sensor reports needed (Rotation Vector `0x05`, Gyroscope `0x02`, Linear Acceleration `0x04`) and drains the SHTP input queue each loop |
| 04 | `Step04_Verify_Report_IDs` | Verifies each report ID is actually arriving at its configured rate before trusting the decoded values |
| 05 | `Step05_Read_Reports` | Decodes the raw SH-2 payloads (quaternion i/j/k/real, gyro XYZ, linear accel XYZ) plus per-sensor accuracy/status flags, and sanity-checks quaternion magnitude (should be ≈1.0) |
| 06 | `Step06_Synchronized_Acquisition` | Adds a synchronization gate: a row is only emitted once the rotation vector, gyroscope, **and** linear acceleration have all freshly updated, so every logged row represents one consistent instant rather than three independently-timed sensor updates |
| 07 | `Step07_CSV_Logger` | Final firmware — formats each synchronized sample as a comma-separated row with a microsecond `micros()` timestamp, and prefixes all diagnostic/boot text with `#` so the Python logger can cleanly separate data from log noise |

**Output format (Step07, what ships to the PC):**
```
Time_us,Quaternion_i,Quaternion_j,Quaternion_k,Quaternion_real,GyroX,GyroY,GyroZ,LinearAccelX,LinearAccelY,LinearAccelZ
464447,-0.0669,0.0311,-0.9920,0.1027,0.0000,0.0000,-0.0020,-0.0234,0.0117,-0.1562
```

**Firmware design decisions worth noting:**
- No `delay()` calls in the acquisition loop — blocking calls were identified as a cause of I2C stalls and packet drops, so all loops are non-blocking.
- `snprintf` into a single fixed buffer per row rather than multiple `Serial.print()` calls, so a full row is written to the UART in one transaction.
- Union-safe field access — the SparkFun library returns sensor reports through a C union (`sh2_SensorValue_t.un`), and only the field matching the current `sensorId` is read to avoid garbled cross-read data.

---

## 6. Python Analysis Pipeline (Step08–Step11 + `imu_utils.py`)

| Script | Purpose |
|---|---|
| `step08_serial_logger.py` | Opens the ESP32's serial port at 115200 baud, waits for the exact CSV header string (to reject boot-time garbage), and streams incoming rows into a timestamped `imu_log_YYYYMMDD_HHMMSS.csv` |
| `step09_plots.py` | Loads a logged CSV and produces a 4-panel raw-sensor-health figure: quaternion components, quaternion magnitude `|q|` (should sit at ~1.0000 as a sanity check), gyroscope XYZ, linear acceleration XYZ |
| `step10_vehicle_dynamics.py` | Converts quaternions to Roll/Pitch/Yaw (ZYX Euler convention), converts gyro rad/s to deg/s, converts linear acceleration m/s² → g, applies a 4th-order Butterworth low-pass filter (10 Hz cutoff) to remove chassis vibration, and renders a 6-panel vehicle-dynamics dashboard figure |
| `step11_iso4138_analysis.py` | The core handling-analysis script — see §6.1 below |
| `imu_utils.py` | Shared, side-effect-free helpers used by all three: `quaternion_to_euler()`, `butterworth_lpf()` (via `scipy.signal.filtfilt`, zero-phase), `find_latest_csv()`, and output-path helpers for `plots/` and `results/` |
| `generatedata.py` | Synthetic CSV generator that simulates an EV656 running an ISO 4138 constant-radius test (known ground-truth Roll Gradient = 9.5 deg/g) — used to validate the analysis pipeline against a known-correct answer before trusting it on real vehicle data |

### 6.1 ISO 4138 Steady-State Analysis (`step11_iso4138_analysis.py`)

This is the analytical heart of the project. It implements the ISO 4138 steady-state circular test method using IMU data alone:

1. **Filter** — roll angle, lateral acceleration, and yaw rate are passed through a 4th-order Butterworth low-pass filter at **2 Hz** (tighter than Step10's 10 Hz, chosen to remove driver steering corrections at 1–5 Hz while preserving body motion below ~1 Hz).
2. **Steady-state window detection** — a sliding ≥2.0 s window is accepted only if, within it: lateral-accel std < 0.02 g, roll std < 0.5°, yaw-rate std < 2.0°/s, and mean |aᵧ| falls between 0.05 g and 0.85 g.
3. **Direction split** — accepted windows are separated into CW (aᵧ < 0) and CCW (aᵧ > 0) groups, per the standard's requirement to test both directions.
4. **Linear regression** — for each direction, an OLS fit (`scipy.stats.linregress`) of Roll vs. |aᵧ| gives the **Roll Gradient** (deg/g); a second fit of Yaw Rate vs. |aᵧ| gives the **Yaw Rate Gradient** ((deg/s)/g).
5. **Combine** — CW and CCW gradients are averaged into a single reported Roll Gradient, canceling road-camber and sensor Y-axis bias.
6. **Quality gate** — ISO 4138 requires R² ≥ 0.98 for a result to be considered valid; the script and dashboard both flag PASS/FAIL against this threshold.
7. **IMU-only speed estimate** — `v = r · |yaw rate|` using the known test-circle radius, used as a stand-in until a dedicated speed sensor is added.

The script can run standalone from the command line (`python step11_iso4138_analysis.py --name EV656 --mass 422 --wheelbase 2.07 ...`) producing a text report and a 5-panel figure (time history, steady-state windows highlighted, Roll Gradient regression, Yaw Rate Gradient regression, FFT justifying the filter cutoff), or it can be driven headlessly by the dashboard pipeline.

**Understeer Gradient (UG)** is scaffolded into the vehicle/result schema (CLI args for wheelbase, CG height, CG-to-front, track width, front mass) but is **not yet computable** — it requires a steering-angle sensor that has not yet been added to the hardware. The results CSV already reserves `ug_cw_deg_per_g` / `ug_ccw_deg_per_g` / `ug_combined_deg_per_g` columns for when that sensor is integrated.

---

## 7. User Interface — Streamlit Dashboard

`app.py` launches a Streamlit web app (`streamlit run app.py`, served at `localhost:8501`) that wraps the entire firmware→analysis pipeline in a single-page browser UI, so a test can be run end-to-end without a terminal. It's built as a small package under `dashboard/`:

| Module | Responsibility |
|---|---|
| `dashboard/ui.py` | Renders the page layout and all widgets (this is the actual UI code) |
| `dashboard/state.py` | Centralized Streamlit session-state key constants, single source of truth for widget↔JSON field mapping |
| `dashboard/runner.py` | Background-thread serial logger — opens the COM port, waits for the CSV header, streams rows to disk, tracks row count/elapsed time/sample rate live |
| `dashboard/pipeline.py` | Background-thread analysis runner — validates the logged CSV, then runs Step09 → Step10 → Step11 in sequence, collecting each step's summary numbers and figure into a results dict for the UI |
| `dashboard/presets.py` | Saves/loads named vehicle configurations as JSON files under `presets/` |

### Layout

**Two-column top section:**
- **Left — Vehicle Configuration:** text/number inputs for vehicle name, mass, wheelbase, CG height, CG-to-front, track width, front-axle mass, with input validation (all values must be positive, front mass < total mass, CG-to-front < wheelbase) and a preset save/load control (`presets/HandTest.json` exists as an example saved preset).
- **Right — Test Control:** file-name field (optional — defaults to a timestamp), COM port field (defaults `COM12`), test-radius field (default 10 m), a **START TEST / STOP TEST** button, and a live status block showing logger state (`Waiting for ESP32` → `Logging` → `Done`, color-coded) with live elapsed time / row count / sample rate metrics while a test is running. When logging finishes, the analysis pipeline **auto-triggers** — no separate "Analyze" button needed.

**Results section — three tabs:**
1. **Raw Sensor** — the Step09 4-panel plot, plus row count, duration, sample rate, and quaternion-magnitude min/max as quick health-check metrics.
2. **Vehicle Dynamics** — the Step10 6-panel plot, plus Roll/Pitch/Yaw min/max and filtered longitudinal/lateral/vertical acceleration min/max.
3. **ISO 4138** — the Step11 5-panel plot, a live-rendered (not static-image) regression plot built directly from the pipeline's numeric results, an expandable "Method & Formulae" panel that documents the exact analysis procedure and LaTeX-rendered equations in-app, and a results summary: windows used, Roll Gradient, R², Yaw Rate Gradient, and a green PASS / red FAIL badge per ISO 4138's R² ≥ 0.98 criterion for each direction, plus the combined Roll Gradient.

Each results tab gracefully degrades: it shows fresh data when a pipeline run just completed, falls back to the last-saved plot image with a "stale data" note if the app was restarted, or shows a placeholder/spinner if nothing has been run yet. Every test run also archives its three output figures under `plots/` tagged with the run name (e.g. `imu_log_20260709_142929_vehicle_dynamics.png`) instead of overwriting the previous run, and appends one row per session to a per-vehicle results CSV under `results/`.

Styling is a light custom CSS theme (`_THEME_CSS` in `ui.py`) giving Streamlit's metric tiles a white card look with a blue accent border, rather than Streamlit's default flat styling.

---

## 8. Data Collected & Current Testing Status

Real hardware logging sessions on record (`imu_log_*.csv`), spanning **2026-06-29 through 2026-07-09**:

- 8 sessions on 2026-06-29 (initial bring-up/bench testing)
- 2 sessions on 2026-06-30
- 1 session on 2026-07-07
- 4 sessions on 2026-07-09 (most recent)
- 1 synthetic validation file: `imu_log_synthetic_EV656_ISO4138.csv` (generated by `generatedata.py`, ground-truth Roll Gradient 9.5 deg/g, used to validate the analysis math independent of real hardware noise)

**Current status of ISO 4138 results:** the results log (`results/unnamed_results.csv`) shows every real-vehicle session run through the pipeline so far returned **0 steady-state windows** (`n_windows_total = 0`), meaning no Roll Gradient has yet been computed from real vehicle data — the recorded drives haven't yet contained a long enough, stable-enough constant-radius segment to satisfy the strict ISO 4138 acceptance thresholds (±0.02 g lateral-accel std, ±0.5° roll std, ±2.0°/s yaw-rate std, sustained ≥2.0 s). All of these test rows also have a blank `vehicle_name`, meaning they were run without a saved vehicle preset loaded. This points to the immediate next step being on the **test-execution side** (longer, steadier circles at the test track) rather than the software, since the pipeline itself is already validated against the synthetic ground-truth file.

---

## 9. Supporting Documentation

Produced alongside the code, all present in the project root:

- **`Vehicle_Dynamics_IMU_Handbook.md`** — a 12-chapter, first-principles engineering reference covering I2C protocol timing, MEMS accelerometer/gyroscope/magnetometer physics, BNO085 SH-2/SHTP packet architecture, the full firmware walkthrough, C++ struct/union/pointer concepts used in the driver, quaternion/Euler/Butterworth-filter derivations, a debugging matrix, and a learning roadmap with reference texts (Milliken & Milliken, Gillespie, Lee & Seshia, the ISO 4138 standard itself).
- **`Vehicle_Dynamics_ML_Presentation_corrected.pptx`** / **`_updated.pptx`** — presentation decks for the project.
- **`BNO085 sensor.pptx`** — sensor-focused presentation.
- **`Comparision of two sensor.pdf`** — the write-up comparing IMU candidates that led to selecting the BNO085.
- **Reference datasheets/standards kept for offline use:** `7Semi ES-12242_BNO085_1.0.pdf`, `BNO080_085-Datasheet.pdf`, `ISO-4138-2021.pdf`.

---

## 10. Project File Inventory

```
Arduino/
├── app.py                          Streamlit entry point
├── dashboard/                      UI package (ui, state, runner, pipeline, presets)
├── Gerber_PCB/  + Gerber_PCB.zip   KiCad-exported fabrication files (esp32_bno085_shield)
├── StepNN_*/  (01–07)              ESP32 firmware, one .ino sketch per stage
├── step08_serial_logger.py         Serial → CSV logger
├── step09_plots.py                 Raw sensor health plots
├── step10_vehicle_dynamics.py      Quaternion → vehicle-frame dynamics + filtering
├── step11_iso4138_analysis.py      ISO 4138 steady-state Roll/Yaw-Rate Gradient analysis
├── imu_utils.py                    Shared math/IO helpers
├── generatedata.py                 Synthetic ISO 4138 dataset generator (validation)
├── libraries/SparkFun_BNO08x_Cortex_Based_IMU/   Vendored Arduino IMU driver
├── presets/                        Saved vehicle configuration JSON files
├── results/                        Per-vehicle ISO 4138 results history (CSV)
├── plots/                          Archived analysis figures per test run
├── imu_log_*.csv                   Raw logged test sessions (14 real + 1 synthetic)
├── Vehicle_Dynamics_IMU_Handbook.md   First-principles engineering handbook
└── *.pptx / *.pdf                  Presentations, datasheets, ISO standard, sensor comparison
```

---

## 11. Summary of What's Working vs. Outstanding

**Working end-to-end today:**
- Custom PCB fabrication files ready for board-house submission.
- Full firmware chain from I2C bring-up through synchronized CSV streaming, with reset recovery for automotive power conditions.
- Full Python pipeline from serial capture through filtered vehicle-dynamics computation.
- ISO 4138 Roll Gradient / Yaw Rate Gradient math, validated against a synthetic ground-truth dataset.
- A polished Streamlit dashboard that runs the entire test-and-analysis cycle from one browser page, with live status, presets, and per-run archiving.

**Outstanding:**
- No real-world drive session has yet produced a valid (R² ≥ 0.98) steady-state window — needs longer/steadier constant-radius test runs at the track.
- Understeer Gradient is schema-ready but blocked on adding a steering-angle sensor.
- Editable KiCad source files (`.kicad_pro/.kicad_sch/.kicad_pcb`) were not found in this workspace — only the exported Gerber/drill set survives; worth locating and backing up the source project if the board ever needs a revision.
