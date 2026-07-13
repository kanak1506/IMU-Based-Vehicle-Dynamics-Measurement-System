"""
=========================================================
 Vehicle Dynamics IMU Project
 Step 09 - Data Plots
=========================================================

Purpose:
    Read the logged CSV file and produce four plots:
      1. Quaternion components over time
      2. Quaternion magnitude |q| as a health check
      3. Gyroscope XYZ over time
      4. Linear Acceleration XYZ over time

Usage:
    python step09_plots.py

Requirements:
    pip install pandas matplotlib
=========================================================
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from imu_utils import find_latest_csv as _find_latest_csv, plots_path

# ── Load Data ─────────────────────────────────────────────
def load_data(filepath):
    """
    Load CSV, convert Time_us to seconds from zero,
    compute quaternion magnitude as a health check column.
    """
    print(f'[Plot] Loading: {filepath}')
    df = pd.read_csv(filepath)

    # Convert microseconds to seconds, starting from zero
    df['Time_s'] = (df['Time_us'] - df['Time_us'].iloc[0]) / 1e6

    # Quaternion magnitude — must always equal ~1.0
    df['Quat_mag'] = np.sqrt(
        df['Quaternion_i']**2 +
        df['Quaternion_j']**2 +
        df['Quaternion_k']**2 +
        df['Quaternion_real']**2
    )

    print(f'[Plot] Rows loaded   : {len(df)}')
    print(f'[Plot] Duration      : {df["Time_s"].iloc[-1]:.1f} seconds')
    print(f'[Plot] Sample rate   : {len(df) / df["Time_s"].iloc[-1]:.1f} rows/sec')
    print(f'[Plot] |q| min/max   : {df["Quat_mag"].min():.6f} / {df["Quat_mag"].max():.6f}')
    print()

    return df


# ── Plot ──────────────────────────────────────────────────
def plot_all(df, csv_file):
    """
    Four-panel plot covering all sensor streams.
    """
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('Vehicle Dynamics IMU — Session: ' + Path(csv_file).name,
                 fontsize=12, fontweight='bold')

    gs = gridspec.GridSpec(4, 1, hspace=0.45)

    t = df['Time_s']

    # ── Panel 1: Quaternion Components ───────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df['Quaternion_i'],    label='i', linewidth=0.8)
    ax1.plot(t, df['Quaternion_j'],    label='j', linewidth=0.8)
    ax1.plot(t, df['Quaternion_k'],    label='k', linewidth=0.8)
    ax1.plot(t, df['Quaternion_real'], label='real', linewidth=0.8)
    ax1.set_title('Rotation Vector — Quaternion Components')
    ax1.set_ylabel('Value (unitless)')
    ax1.legend(loc='upper right', ncol=4, fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(t.iloc[0], t.iloc[-1])

    # ── Panel 2: Quaternion Magnitude ────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, df['Quat_mag'], color='black', linewidth=0.8, label='|q|')
    ax2.axhline(y=1.0, color='red', linestyle='--',
                linewidth=0.8, label='Expected = 1.0')
    ax2.set_title('Quaternion Magnitude |q| — Health Check (must be ~1.0000)')
    ax2.set_ylabel('|q|')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(t.iloc[0], t.iloc[-1])
    # Tight y-axis to make deviations visible
    ax2.set_ylim(0.999, 1.001)

    # ── Panel 3: Gyroscope ───────────────────────────────
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, df['GyroX'], label='X', linewidth=0.8)
    ax3.plot(t, df['GyroY'], label='Y', linewidth=0.8)
    ax3.plot(t, df['GyroZ'], label='Z', linewidth=0.8)
    ax3.set_title('Calibrated Gyroscope')
    ax3.set_ylabel('Angular Rate (rad/s)')
    ax3.legend(loc='upper right', ncol=3, fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(t.iloc[0], t.iloc[-1])
    ax3.axhline(y=0, color='black', linewidth=0.5, linestyle='--')

    # ── Panel 4: Linear Acceleration ─────────────────────
    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, df['LinearAccelX'], label='X', linewidth=0.8)
    ax4.plot(t, df['LinearAccelY'], label='Y', linewidth=0.8)
    ax4.plot(t, df['LinearAccelZ'], label='Z', linewidth=0.8)
    ax4.set_title('Linear Acceleration (gravity removed)')
    ax4.set_ylabel('Acceleration (m/s²)')
    ax4.set_xlabel('Time (seconds)')
    ax4.legend(loc='upper right', ncol=3, fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(t.iloc[0], t.iloc[-1])
    ax4.axhline(y=0, color='black', linewidth=0.5, linestyle='--')

    out_path = plots_path('imu_plots.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'[Plot] Saved: {out_path}')
    plt.show()
    return fig


# ── Entry Point ───────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Step 09 — IMU data plots',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Example: python step09_plots.py --csv imu_log_20260629_110120.csv'
    )
    parser.add_argument('--csv', metavar='FILE',
                        help='CSV file to plot (default: latest imu_log_*.csv in current directory)')
    args = parser.parse_args()

    csv_file = args.csv if args.csv else _find_latest_csv()
    print(f'[Plot] Using: {csv_file}')
    df = load_data(csv_file)
    plot_all(df, csv_file)