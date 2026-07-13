"""
=========================================================
 Vehicle Dynamics IMU Project
 Step 10 - Vehicle Dynamics Analysis
=========================================================

Purpose:
    Convert raw IMU data into vehicle dynamics quantities:
      - Roll, Pitch, Yaw (degrees) from quaternion
      - Angular rates (deg/s) from gyroscope
      - Longitudinal, Lateral, Vertical acceleration (g)
      - Dashboard-style plot in vehicle coordinate frame

Coordinate Frame Convention (sensor mounted flat, USB forward):
    X axis  = Longitudinal (forward/back)
    Y axis  = Lateral      (left/right)
    Z axis  = Vertical     (up/down)

    Roll    = rotation about X (longitudinal axis)
    Pitch   = rotation about Y (lateral axis)
    Yaw     = rotation about Z (vertical axis)

Usage:
    python step10_vehicle_dynamics.py

Requirements:
    pip install pandas matplotlib numpy scipy
=========================================================
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from imu_utils import find_latest_csv as _find_latest_csv, quaternion_to_euler, butterworth_lpf as lowpass_filter, G, plots_path

# 10 Hz removes vibration noise while preserving vehicle body motion.
# Increase to 20 Hz for faster transient response.
ACCEL_LPF_CUTOFF = 10.0  # Hz


# ── Load and Prepare Data ─────────────────────────────────
def load_data(filepath):
    """
    Load CSV and compute time axis in seconds from zero.
    """
    print(f'[Analysis] Loading: {filepath}')
    df = pd.read_csv(filepath)
    df['Time_s'] = (df['Time_us'] - df['Time_us'].iloc[0]) / 1e6

    print(f'[Analysis] Rows     : {len(df)}')
    print(f'[Analysis] Duration : {df["Time_s"].iloc[-1]:.1f} seconds')
    print(f'[Analysis] Rate     : {len(df) / df["Time_s"].iloc[-1]:.1f} rows/sec')
    return df


# ── Compute Vehicle Dynamics ──────────────────────────────
def compute_dynamics(df):
    """
    Compute all vehicle dynamics quantities from raw data.

    Adds the following columns to the DataFrame:
        Roll_deg, Pitch_deg, Yaw_deg
        RollRate_degs, PitchRate_degs, YawRate_degs
        LongAccel_g, LatAccel_g, VertAccel_g
        LongAccel_filt_g, LatAccel_filt_g, VertAccel_filt_g
    """
    print('[Analysis] Computing vehicle dynamics...')

    # ── Sample rate ──────────────────────────────────────
    duration    = df['Time_s'].iloc[-1]
    sample_rate = len(df) / duration

    # ── Euler angles from quaternion ─────────────────────
    df['Roll_deg'], df['Pitch_deg'], df['Yaw_deg'] = quaternion_to_euler(
        df['Quaternion_i'].values,
        df['Quaternion_j'].values,
        df['Quaternion_k'].values,
        df['Quaternion_real'].values
    )

    # ── Angular rates: rad/s to deg/s ────────────────────
    df['RollRate_degs']  = np.degrees(df['GyroX'])
    df['PitchRate_degs'] = np.degrees(df['GyroY'])
    df['YawRate_degs']   = np.degrees(df['GyroZ'])

    # ── Linear acceleration: m/s² to g ───────────────────
    # Raw (unfiltered) — for reference
    df['LongAccel_g'] = df['LinearAccelX'] / G
    df['LatAccel_g']  = df['LinearAccelY'] / G
    df['VertAccel_g'] = df['LinearAccelZ'] / G

    # Filtered — for vehicle dynamics analysis
    # Low-pass removes bench vibration and sensor noise
    df['LongAccel_filt_g'] = lowpass_filter(
        df['LongAccel_g'].values, ACCEL_LPF_CUTOFF, sample_rate)
    df['LatAccel_filt_g']  = lowpass_filter(
        df['LatAccel_g'].values,  ACCEL_LPF_CUTOFF, sample_rate)
    df['VertAccel_filt_g'] = lowpass_filter(
        df['VertAccel_g'].values, ACCEL_LPF_CUTOFF, sample_rate)

    # ── Print statistics ─────────────────────────────────
    print(f'[Analysis] Roll  : {df["Roll_deg"].min():+.1f} to {df["Roll_deg"].max():+.1f} deg')
    print(f'[Analysis] Pitch : {df["Pitch_deg"].min():+.1f} to {df["Pitch_deg"].max():+.1f} deg')
    print(f'[Analysis] Yaw   : {df["Yaw_deg"].min():+.1f} to {df["Yaw_deg"].max():+.1f} deg')
    print(f'[Analysis] Long  : {df["LongAccel_filt_g"].min():+.3f} to {df["LongAccel_filt_g"].max():+.3f} g')
    print(f'[Analysis] Lat   : {df["LatAccel_filt_g"].min():+.3f} to {df["LatAccel_filt_g"].max():+.3f} g')
    print(f'[Analysis] Vert  : {df["VertAccel_filt_g"].min():+.3f} to {df["VertAccel_filt_g"].max():+.3f} g')
    print()

    return df


# ── Dashboard Plot ────────────────────────────────────────
def plot_dashboard(df, csv_file):
    """
    Six-panel vehicle dynamics dashboard.

    Panel 1: Roll, Pitch, Yaw (degrees)
    Panel 2: Yaw rate (deg/s) — most relevant for cornering
    Panel 3: Roll and Pitch rate (deg/s)
    Panel 4: Longitudinal acceleration (g) raw + filtered
    Panel 5: Lateral acceleration (g) raw + filtered
    Panel 6: Vertical acceleration (g) raw + filtered
    """
    fig = plt.figure(figsize=(14, 14))
    fig.suptitle(
        f'Vehicle Dynamics Analysis — {Path(csv_file).name}',
        fontsize=12, fontweight='bold'
    )

    gs  = gridspec.GridSpec(6, 1, hspace=0.55)
    t   = df['Time_s']
    xlim = (t.iloc[0], t.iloc[-1])

    # ── Panel 1: Euler Angles ────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df['Roll_deg'],  label='Roll',  linewidth=0.9)
    ax1.plot(t, df['Pitch_deg'], label='Pitch', linewidth=0.9)
    ax1.plot(t, df['Yaw_deg'],   label='Yaw',   linewidth=0.9)
    ax1.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax1.set_title('Orientation — Roll / Pitch / Yaw')
    ax1.set_ylabel('Angle (degrees)')
    ax1.legend(loc='upper right', ncol=3, fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(xlim)

    # ── Panel 2: Yaw Rate ────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, df['YawRate_degs'], color='green',
             label='Yaw Rate', linewidth=0.9)
    ax2.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax2.set_title('Yaw Rate (cornering / heading change)')
    ax2.set_ylabel('Rate (deg/s)')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(xlim)

    # ── Panel 3: Roll and Pitch Rate ─────────────────────
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, df['RollRate_degs'],  label='Roll Rate',  linewidth=0.9)
    ax3.plot(t, df['PitchRate_degs'], label='Pitch Rate', linewidth=0.9)
    ax3.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax3.set_title('Roll Rate / Pitch Rate')
    ax3.set_ylabel('Rate (deg/s)')
    ax3.legend(loc='upper right', ncol=2, fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(xlim)

    # ── Panel 4: Longitudinal Acceleration ───────────────
    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, df['LongAccel_g'],
             color='steelblue', alpha=0.3,
             linewidth=0.6, label='Raw')
    ax4.plot(t, df['LongAccel_filt_g'],
             color='steelblue', linewidth=1.2,
             label=f'Filtered ({ACCEL_LPF_CUTOFF} Hz LPF)')
    ax4.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax4.set_title('Longitudinal Acceleration (forward/brake)')
    ax4.set_ylabel('Acceleration (g)')
    ax4.legend(loc='upper right', ncol=2, fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(xlim)

    # ── Panel 5: Lateral Acceleration ────────────────────
    ax5 = fig.add_subplot(gs[4])
    ax5.plot(t, df['LatAccel_g'],
             color='darkorange', alpha=0.3,
             linewidth=0.6, label='Raw')
    ax5.plot(t, df['LatAccel_filt_g'],
             color='darkorange', linewidth=1.2,
             label=f'Filtered ({ACCEL_LPF_CUTOFF} Hz LPF)')
    ax5.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax5.set_title('Lateral Acceleration (cornering left/right)')
    ax5.set_ylabel('Acceleration (g)')
    ax5.legend(loc='upper right', ncol=2, fontsize=8)
    ax5.grid(True, alpha=0.3)
    ax5.set_xlim(xlim)

    # ── Panel 6: Vertical Acceleration ───────────────────
    ax6 = fig.add_subplot(gs[5])
    ax6.plot(t, df['VertAccel_g'],
             color='forestgreen', alpha=0.3,
             linewidth=0.6, label='Raw')
    ax6.plot(t, df['VertAccel_filt_g'],
             color='forestgreen', linewidth=1.2,
             label=f'Filtered ({ACCEL_LPF_CUTOFF} Hz LPF)')
    ax6.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax6.set_title('Vertical Acceleration (bumps / heave)')
    ax6.set_ylabel('Acceleration (g)')
    ax6.set_xlabel('Time (seconds)')
    ax6.legend(loc='upper right', ncol=2, fontsize=8)
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim(xlim)

    out_path = plots_path('vehicle_dynamics.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'[Analysis] Saved: {out_path}')
    plt.show()
    return fig


# ── Entry Point ───────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Step 10 — Vehicle dynamics analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Example: python step10_vehicle_dynamics.py --csv imu_log_20260629_110120.csv'
    )
    parser.add_argument('--csv', metavar='FILE',
                        help='CSV file to analyse (default: latest imu_log_*.csv in current directory)')
    args = parser.parse_args()

    csv_file = args.csv if args.csv else _find_latest_csv()
    print(f'[Analysis] Using: {csv_file}')
    df = load_data(csv_file)
    df = compute_dynamics(df)
    plot_dashboard(df, csv_file)