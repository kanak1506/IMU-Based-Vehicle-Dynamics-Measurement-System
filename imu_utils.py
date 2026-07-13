"""
Shared utilities used by step09, step10, and step11.
All functions are pure and have no side-effects on the step scripts.
"""

import glob
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

G = 9.81  # m/s²

PLOTS_DIR   = Path('plots')
RESULTS_DIR = Path('results')


def plots_path(filename: str) -> str:
    """Return the path for a plot output file, creating PLOTS_DIR if needed."""
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    return str(PLOTS_DIR / filename)


def results_path(filename: str) -> str:
    """Return the path for a results CSV output file, creating RESULTS_DIR if needed."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    return str(RESULTS_DIR / filename)


def find_latest_csv() -> str:
    """Return the path of the most recently created imu_log_*.csv file."""
    files = sorted(glob.glob('imu_log_*.csv'))
    if not files:
        print('ERROR: No imu_log_*.csv files found in current directory.')
        sys.exit(1)
    return files[-1]


def quaternion_to_euler(qi, qj, qk, qr):
    """
    Convert unit quaternion (i, j, k, real) to Roll, Pitch, Yaw in degrees.
    ZYX convention (aerospace standard). Gimbal lock at pitch = ±90 deg.
    """
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll      = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    sinp  = np.clip(2.0 * (qr * qj - qk * qi), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw       = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def butterworth_lpf(data, cutoff_hz, sample_rate_hz, order=4):
    """Butterworth low-pass filter. Returns filtered array of same length."""
    nyquist = sample_rate_hz / 2.0
    normal  = min(cutoff_hz / nyquist, 0.99)
    b, a    = butter(order, normal, btype='low', analog=False)
    return filtfilt(b, a, data)
