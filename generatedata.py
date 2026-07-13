"""
=========================================================
 Vehicle Dynamics IMU Project
 Synthetic Dataset Generator — EV656 ISO 4138
=========================================================

Generates a realistic synthetic CSV that simulates the
EV656 performing an ISO 4138 constant radius test at 10m.

Physics used:
    Lateral acceleration : ay = v² / R
    Yaw rate             : r  = v / R  (rad/s)
    Roll angle           : phi = RG * ay  (deg)
    where RG = roll gradient (deg/g)

Noise model:
    Low frequency driver corrections : 0.5-2 Hz sine waves
    High frequency road/vibration    : white noise > 5 Hz
    Sensor noise                     : BNO085 spec noise floor

Test profile:
    5 speed steps CW  (negative ay)
    5 speed steps CCW (positive ay)
    Each step: 3s ramp up + 6s steady state + 2s transition
    ay range: 0.1g to 0.5g (safe for 10m radius)

Known ground truth (to verify analysis output):
    Roll Gradient    : 9.5 deg/g
    Yaw Rate Gradient: depends on speed/radius
=========================================================
"""

import numpy as np
import pandas as pd

# ── Vehicle Parameters (EV656) ────────────────────────────
MASS_KG        = 422.0
WHEELBASE_M    = 2.070
TRACK_M        = 0.875
CG_HEIGHT_M    = 0.633
RADIUS_M       = 10.0
G              = 9.81

# ── Known ground truth roll gradient ─────────────────────
# Theoretical: RG = (m * h * g) / (K_phi)
# For EV656 geometry, estimated from CG height and track:
# Simple estimate: RG ≈ 57.3 * m * h / (K_phi_total)
# We set this as the TARGET — analysis must recover it
ROLL_GRADIENT_DEG_PER_G = 9.5   # deg/g  (ground truth)

# ── Sample rate ───────────────────────────────────────────
SAMPLE_RATE_HZ = 90             # Matches real BNO085 output rate
DT             = 1.0 / SAMPLE_RATE_HZ

# ── Test speed steps ──────────────────────────────────────
# ay = v²/R -> v = sqrt(ay * R * g)
# At R=10m:
#   0.1g -> v = sqrt(0.1*9.81*10) = 3.13 m/s = 11.3 kph
#   0.2g -> v = sqrt(0.2*9.81*10) = 4.43 m/s = 15.9 kph
#   0.3g -> v = sqrt(0.3*9.81*10) = 5.42 m/s = 19.5 kph
#   0.4g -> v = sqrt(0.4*9.81*10) = 6.26 m/s = 22.5 kph
#   0.5g -> v = sqrt(0.5*9.81*10) = 7.00 m/s = 25.2 kph
AY_STEPS_G = [0.10, 0.20, 0.30, 0.40, 0.50]

# Duration of each phase (seconds)
T_RAMP       = 3.0   # Speed ramp up
T_STEADY     = 6.0   # Steady state hold
T_TRANSITION = 2.0   # Between steps

# ── Noise Parameters ──────────────────────────────────────
# Driver correction amplitude (low frequency, 0.5-2 Hz)
DRIVER_CORRECTION_AY_G  = 0.025   # ±0.025g correction noise
DRIVER_CORRECTION_ROLL  = 0.20    # ±0.20 deg roll noise

# Road surface noise (high frequency, >5 Hz)
ROAD_NOISE_AY_G         = 0.015
ROAD_NOISE_ROLL_DEG     = 0.10

# BNO085 sensor noise floor
SENSOR_NOISE_AY_G       = 0.003
SENSOR_NOISE_GYRO_DEGS  = 0.05
SENSOR_NOISE_ROLL_DEG   = 0.02


# ─────────────────────────────────────────────────────────
def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert Euler angles to quaternion.
    ZYX convention matching BNO085 output.
    """
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)

    qi   = sr*cp*cy - cr*sp*sy
    qj   = cr*sp*cy + sr*cp*sy
    qk   = cr*cp*sy - sr*sp*cy
    qr   = cr*cp*cy + sr*sp*sy

    return qi, qj, qk, qr


# ─────────────────────────────────────────────────────────
def generate_noise(n_samples, amplitude, freq_low, freq_high,
                   sample_rate, rng):
    """
    Generate bandlimited noise between freq_low and freq_high Hz.
    Used to simulate driver corrections and road surface inputs.
    """
    # Generate white noise
    noise = rng.normal(0, amplitude, n_samples)

    # Bandpass filter using FFT
    freqs  = np.fft.rfftfreq(n_samples, d=1.0/sample_rate)
    fft    = np.fft.rfft(noise)

    # Zero out frequencies outside band
    mask   = (freqs >= freq_low) & (freqs <= freq_high)
    fft[~mask] = 0

    filtered = np.fft.irfft(fft, n=n_samples)

    # Rescale to target amplitude
    if filtered.std() > 0:
        filtered = filtered * (amplitude / filtered.std())

    return filtered


# ─────────────────────────────────────────────────────────
def generate_speed_step(ay_target_g, direction, rng):
    """
    Generate one speed step: ramp + steady state + transition.

    Returns arrays of:
        time, ay_clean, ay_noisy, roll_clean, roll_noisy,
        yaw_rate_clean, yaw_rate_noisy, long_accel, vert_accel
    """
    # Total samples
    n_ramp       = int(T_RAMP       * SAMPLE_RATE_HZ)
    n_steady     = int(T_STEADY     * SAMPLE_RATE_HZ)
    n_transition = int(T_TRANSITION * SAMPLE_RATE_HZ)
    n_total      = n_ramp + n_steady + n_transition

    t = np.arange(n_total) * DT

    # ── Clean lateral acceleration profile ───────────────
    # Ramp: smooth sigmoid from 0 to target
    ramp_t   = np.linspace(0, 1, n_ramp)
    ramp_ay  = ay_target_g * (3*ramp_t**2 - 2*ramp_t**3)  # smooth step

    # Steady state: constant
    steady_ay = np.full(n_steady, ay_target_g)

    # Transition: back to zero
    trans_t  = np.linspace(0, 1, n_transition)
    trans_ay = ay_target_g * (1 - (3*trans_t**2 - 2*trans_t**3))

    ay_clean = np.concatenate([ramp_ay, steady_ay, trans_ay])

    # Apply direction sign
    ay_signed_clean = ay_clean * direction   # +1 CCW, -1 CW

    # ── Clean roll angle ──────────────────────────────────
    # phi = RG * ay   (linear model)
    roll_clean = ROLL_GRADIENT_DEG_PER_G * ay_clean * direction

    # Small pitch due to longitudinal load transfer during ramp
    # Pitch is small — ~1 deg at max acceleration
    pitch_clean = -0.8 * np.gradient(ay_clean, DT) * 0.3

    # Yaw accumulates during cornering
    # yaw_rate = v/R = sqrt(ay*g*R)/R = sqrt(ay*g/R)
    yaw_rate_clean_rads = np.sqrt(
        np.maximum(ay_clean * G / RADIUS_M, 0)
    )  # rad/s
    yaw_rate_clean_degs = np.degrees(yaw_rate_clean_rads) * direction

    # Integrate yaw rate to get yaw angle
    yaw_clean = np.cumsum(yaw_rate_clean_degs) * DT

    # ── Add realistic noise ───────────────────────────────
    # Driver corrections (0.5-2 Hz) — present throughout
    driver_noise_ay = generate_noise(
        n_total, DRIVER_CORRECTION_AY_G, 0.5, 2.0,
        SAMPLE_RATE_HZ, rng)
    driver_noise_roll = generate_noise(
        n_total, DRIVER_CORRECTION_ROLL, 0.5, 2.0,
        SAMPLE_RATE_HZ, rng)

    # Road surface (5-15 Hz) — present throughout
    road_noise_ay = generate_noise(
        n_total, ROAD_NOISE_AY_G, 5.0, 15.0,
        SAMPLE_RATE_HZ, rng)
    road_noise_roll = generate_noise(
        n_total, ROAD_NOISE_ROLL_DEG, 5.0, 15.0,
        SAMPLE_RATE_HZ, rng)

    # Sensor noise floor (white noise)
    sensor_noise_ay   = rng.normal(0, SENSOR_NOISE_AY_G, n_total)
    sensor_noise_roll = rng.normal(0, SENSOR_NOISE_ROLL_DEG, n_total)
    sensor_noise_yr   = rng.normal(0, SENSOR_NOISE_GYRO_DEGS, n_total)

    # Combined noisy signals
    ay_noisy   = (ay_signed_clean +
                  driver_noise_ay +
                  road_noise_ay +
                  sensor_noise_ay)

    roll_noisy = (roll_clean +
                  driver_noise_roll +
                  road_noise_roll +
                  sensor_noise_roll)

    yr_noisy   = (yaw_rate_clean_degs +
                  generate_noise(n_total, 2.0, 0.5, 2.0,
                                 SAMPLE_RATE_HZ, rng) +
                  sensor_noise_yr)

    # Longitudinal accel — small, from ramp up/braking
    long_accel = np.gradient(
        np.sqrt(np.maximum(ay_clean * G * RADIUS_M, 0)),
        DT) / G
    long_accel += rng.normal(0, 0.01, n_total)

    # Vertical accel — road surface bumps
    vert_accel = generate_noise(
        n_total, 0.05, 2.0, 15.0, SAMPLE_RATE_HZ, rng)

    return {
        'n'             : n_total,
        'ay_signed'     : ay_signed_clean,
        'ay_noisy'      : ay_noisy,
        'roll_clean'    : roll_clean,
        'roll_noisy'    : roll_noisy,
        'pitch_clean'   : pitch_clean,
        'yaw_clean'     : yaw_clean,
        'yr_noisy'      : yr_noisy,
        'long_accel'    : long_accel,
        'vert_accel'    : vert_accel,
    }


# ─────────────────────────────────────────────────────────
def generate_full_test():
    """
    Generate complete ISO 4138 test:
    5 steps CW then 5 steps CCW.
    Returns a DataFrame matching the imu_log CSV format.
    """
    print('[SynGen] Generating EV656 ISO 4138 synthetic dataset...')
    print(f'[SynGen] Ground truth Roll Gradient: '
          f'{ROLL_GRADIENT_DEG_PER_G} deg/g')
    print(f'[SynGen] Test radius: {RADIUS_M} m')
    print(f'[SynGen] Speed steps: {AY_STEPS_G} g')
    print()

    rng = np.random.default_rng(seed=42)  # Reproducible

    rows = []
    t_global_us = 0
    step_num = 0

    # Direction sequence: CW then CCW
    for direction, dir_label in [(-1, 'CW'), (1, 'CCW')]:
        print(f'[SynGen] Generating {dir_label} steps...')

        # Add 3 second straight section between directions
        n_straight = int(3.0 * SAMPLE_RATE_HZ)
        for _ in range(n_straight):
            qi, qj, qk, qr = euler_to_quaternion(0, 0, 0)
            rows.append({
                'Time_us'        : int(t_global_us),
                'Quaternion_i'   : round(qi, 4),
                'Quaternion_j'   : round(qj, 4),
                'Quaternion_k'   : round(qk, 4),
                'Quaternion_real': round(qr, 4),
                'GyroX'          : round(rng.normal(0, 0.002), 4),
                'GyroY'          : round(rng.normal(0, 0.002), 4),
                'GyroZ'          : round(rng.normal(0, 0.002), 4),
                'LinearAccelX'   : round(rng.normal(0, 0.03), 4),
                'LinearAccelY'   : round(rng.normal(0, 0.03), 4),
                'LinearAccelZ'   : round(rng.normal(0, 0.05), 4),
            })
            t_global_us += DT * 1e6

        for ay_target in AY_STEPS_G:
            step_num += 1
            data = generate_speed_step(ay_target, direction, rng)

            print(f'  Step {step_num}: {dir_label}  '
                  f'ay={ay_target:.2f}g  '
                  f'v={np.sqrt(ay_target*G*RADIUS_M):.2f} m/s  '
                  f'({np.sqrt(ay_target*G*RADIUS_M)*3.6:.1f} kph)  '
                  f'n={data["n"]} samples')

            # Accumulate yaw
            yaw_offset = rows[-1]['Quaternion_k'] if rows else 0

            for i in range(data['n']):
                roll  = data['roll_noisy'][i]
                pitch = data['pitch_clean'][i]
                yaw   = data['yaw_clean'][i]

                qi, qj, qk, qr = euler_to_quaternion(roll, pitch, yaw)

                # Gyro in rad/s
                gyro_x = np.radians(
                    data['yr_noisy'][i] * 0.1 +
                    rng.normal(0, SENSOR_NOISE_GYRO_DEGS))
                gyro_y = np.radians(rng.normal(0, 0.05))
                gyro_z = np.radians(data['yr_noisy'][i])

                # Linear accel in m/s²
                lin_x = data['long_accel'][i] * G
                lin_y = data['ay_noisy'][i] * G
                lin_z = data['vert_accel'][i] * G

                rows.append({
                    'Time_us'        : int(t_global_us),
                    'Quaternion_i'   : round(qi, 4),
                    'Quaternion_j'   : round(qj, 4),
                    'Quaternion_k'   : round(qk, 4),
                    'Quaternion_real': round(qr, 4),
                    'GyroX'          : round(gyro_x, 6),
                    'GyroY'          : round(gyro_y, 6),
                    'GyroZ'          : round(gyro_z, 6),
                    'LinearAccelX'   : round(lin_x, 4),
                    'LinearAccelY'   : round(lin_y, 4),
                    'LinearAccelZ'   : round(lin_z, 4),
                })
                t_global_us += DT * 1e6

    df = pd.DataFrame(rows)

    print()
    print(f'[SynGen] Total rows     : {len(df)}')
    print(f'[SynGen] Duration       : '
          f'{df["Time_us"].iloc[-1]/1e6:.1f} seconds')
    print(f'[SynGen] Sample rate    : '
          f'{len(df)/(df["Time_us"].iloc[-1]/1e6):.1f} Hz')
    print(f'[SynGen] Lat accel range: '
          f'{df["LinearAccelY"].min()/G:.3f} to '
          f'{df["LinearAccelY"].max()/G:.3f} g')

    return df


# ── Entry Point ───────────────────────────────────────────
if __name__ == '__main__':
    df = generate_full_test()

    filename = 'imu_log_synthetic_EV656_ISO4138.csv'
    df.to_csv(filename, index=False)

    print()
    print('=' * 55)
    print(f'  Synthetic dataset saved: {filename}')
    print(f'  Ground truth Roll Gradient: '
          f'{ROLL_GRADIENT_DEG_PER_G} deg/g')
    print(f'  Run step11_iso4138_analysis.py to verify')
    print(f'  Analysis should recover ~{ROLL_GRADIENT_DEG_PER_G} deg/g')
    print('=' * 55)