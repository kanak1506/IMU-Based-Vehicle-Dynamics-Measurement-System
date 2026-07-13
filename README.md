# IMU-Based Vehicle Dynamics Measurement System

An ESP32 + BNO085 IMU pipeline for logging and analyzing vehicle dynamics: roll, pitch, yaw, and lateral/longitudinal acceleration, with post-processing against the ISO 4138 steady-state circular driving test standard and a Streamlit dashboard for visualization.

## What's here

- **`Step01`–`Step07`** — incremental ESP32 Arduino firmware, from basic I2C init through synchronized multi-report acquisition and CSV logging over serial.
- **`step08_serial_logger.py`** — host-side serial capture to CSV.
- **`step09_plots.py`, `step10_vehicle_dynamics.py`, `step11_iso4138_analysis.py`** — signal plotting, vehicle dynamics derivation (understeer/roll gradient, etc.), and ISO 4138 analysis.
- **`dashboard/`, `app.py`** — Streamlit UI tying the pipeline together.
- **`Gerber_PCB/`** — PCB manufacturing files for the sensor breakout/shield.
- **`Vehicle_Dynamics_IMU_Handbook.md`** — a first-principles write-up covering the hardware, I2C/SPI protocol details, sensor fusion, and the math behind the vehicle dynamics calculations.
- **`report.md`** — project report.

## Setup

1. Flash the `Step01`–`Step07` sketches to an ESP32 wired to a BNO085 (see the handbook for wiring). Install the [SparkFun BNO08x Arduino library](https://github.com/sparkfun/SparkFun_BNO08x_Cortex_Based_IMU_Arduino_Library) via the Arduino Library Manager.
2. `pip install -r requirements.txt`
3. `streamlit run app.py`

## Data

Raw IMU logs (CSV) and generated plots are not tracked in this repo — running the pipeline regenerates them locally.

## Documentation

This README covers setup and repo structure only. For the full technical write-up — hardware theory, circuit design, I2C/SPI protocol deep-dive, sensor fusion, and the math derivations behind the vehicle dynamics calculations — see [Vehicle_Dynamics_IMU_Handbook.md](Vehicle_Dynamics_IMU_Handbook.md).
