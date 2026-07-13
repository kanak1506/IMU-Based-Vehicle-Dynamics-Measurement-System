"""
=========================================================
 Vehicle Dynamics IMU Project
 Step 11 - ISO 4138 Steady State Analysis (IMU Only)
=========================================================

Purpose:
    Perform ISO 4138 compliant steady-state circular test
    analysis using IMU data only.

    Calculates:
      - Roll Gradient        [deg/g]
      - Yaw Rate Gradient    [deg/s per g]
      - Lateral Acceleration vs Roll Angle
      - Steady state window detection and validation
      - Both CW and CCW direction analysis
      - Combined analysis with direction correction

    When steering angle sensor is added (future):
      - Understeer Gradient  [deg/g]
      - Ackermann correction
      - Full ISO 4138 report

ISO 4138 Steady State Definition:
    A time window is considered steady state when:
      1. Lateral acceleration variation < 0.02g over window
      2. Roll angle variation < 0.5 deg over window
      3. Yaw rate variation < 2.0 deg/s over window
      4. Window duration >= MIN_STEADY_STATE_DURATION_S

Usage:
    python step11_iso4138_analysis.py

Requirements:
    pip install pandas matplotlib numpy scipy
=========================================================
"""

import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.stats import linregress

from imu_utils import find_latest_csv, quaternion_to_euler, butterworth_lpf as lpf, G, plots_path, results_path

# VEHICLE and TEST are populated from command-line arguments in __main__.
# They must be set before any analysis function is called.
VEHICLE = {}
TEST    = {}

# ── Analysis Configuration ────────────────────────────────
STEADY_STATE = {
    'min_duration_s'              : 2.0,   # Minimum window length
    'max_ay_variation_g'          : 0.02,  # Max lateral accel variation
    'max_roll_variation_deg'      : 0.5,   # Max roll angle variation
    'max_yawrate_variation_degs'  : 2.0,   # Max yaw rate variation
    'min_ay_g'                    : 0.05,  # Minimum lateral accel to include
    'max_ay_g'                    : 0.85,  # Maximum lateral accel (safety)
}

# 2 Hz preserves vehicle body motion (< 1 Hz) and removes driver corrections (1–5 Hz).
FILTER = {
    'accel_lpf_hz'  : 2.0,
    'angle_lpf_hz'  : 2.0,
    'order'         : 4,
}


# ── Load CSV ──────────────────────────────────────────────
def load_csv(filepath=None):
    if filepath is None:
        filepath = find_latest_csv()
    print(f'[ISO4138] Loading: {filepath}')
    df = pd.read_csv(filepath)
    df['Time_s'] = (df['Time_us'] - df['Time_us'].iloc[0]) / 1e6
    print(f'[ISO4138] Rows     : {len(df)}')
    print(f'[ISO4138] Duration : {df["Time_s"].iloc[-1]:.1f} s')
    print(f'[ISO4138] Rate     : {len(df)/df["Time_s"].iloc[-1]:.1f} Hz')
    return df


# ── Compute All Signals ───────────────────────────────────
def compute_signals(df):
    """
    Compute all derived signals needed for ISO 4138 analysis.

    High frequency vs Low frequency explanation:
        Raw IMU data contains:
          - Low freq  (0-2 Hz)  : vehicle body motion — WANTED
          - Mid freq  (2-5 Hz)  : driver steering corrections — UNWANTED
          - High freq (5+ Hz)   : road surface, vibration — UNWANTED

        We keep BOTH in the DataFrame:
          - _raw  : unfiltered — for reference and transient study
          - _filt : 2 Hz LPF  — for ISO 4138 steady state analysis

        ISO 4138 explicitly requires steady state data.
        Using raw data for gradient calculations gives wrong results
        because transient corrections contaminate the regression.
    """
    print('[ISO4138] Computing signals...')

    rate_hz = len(df) / df['Time_s'].iloc[-1]

    # ── Euler angles ──────────────────────────────────────
    df['Roll_deg'], df['Pitch_deg'], df['Yaw_deg'] = \
        quaternion_to_euler(
            df['Quaternion_i'].values,
            df['Quaternion_j'].values,
            df['Quaternion_k'].values,
            df['Quaternion_real'].values
        )

    # ── Filtered roll — used for gradient calculation ─────
    df['Roll_filt_deg'] = lpf(
        df['Roll_deg'].values,
        FILTER['angle_lpf_hz'], rate_hz, FILTER['order']
    )

    # ── Angular rates ─────────────────────────────────────
    df['YawRate_degs']   = np.degrees(df['GyroZ'])
    df['RollRate_degs']  = np.degrees(df['GyroX'])
    df['PitchRate_degs'] = np.degrees(df['GyroY'])

    # ── Lateral acceleration ──────────────────────────────
    # Raw (m/s² to g)
    df['LatAccel_raw_g']  = df['LinearAccelY'] / G
    df['LongAccel_raw_g'] = df['LinearAccelX'] / G
    df['VertAccel_raw_g'] = df['LinearAccelZ'] / G

    # Filtered — ISO 4138 steady state analysis uses this
    df['LatAccel_filt_g'] = lpf(
        df['LatAccel_raw_g'].values,
        FILTER['accel_lpf_hz'], rate_hz, FILTER['order']
    )
    df['LongAccel_filt_g'] = lpf(
        df['LongAccel_raw_g'].values,
        FILTER['accel_lpf_hz'], rate_hz, FILTER['order']
    )

    # ── Yaw rate filtered ─────────────────────────────────
    df['YawRate_filt_degs'] = lpf(
        df['YawRate_degs'].values,
        FILTER['accel_lpf_hz'], rate_hz, FILTER['order']
    )

    # ── Estimated speed from yaw rate and radius ──────────
    # v = r * omega  where omega = yaw rate in rad/s
    # This is IMU-only speed estimate — replace with
    # actual speed sensor when available
    df['Speed_est_ms'] = (
        TEST['radius_m'] *
        np.abs(np.radians(df['YawRate_filt_degs']))
    )

    print(f'[ISO4138] Signals computed.')
    print(f'[ISO4138] Lateral accel range: '
          f'{df["LatAccel_filt_g"].min():+.3f} to '
          f'{df["LatAccel_filt_g"].max():+.3f} g')
    print(f'[ISO4138] Roll range: '
          f'{df["Roll_filt_deg"].min():+.2f} to '
          f'{df["Roll_filt_deg"].max():+.2f} deg')
    print()

    return df


# ── Steady State Window Detection ─────────────────────────
def detect_steady_state_windows(df):
    """
    ISO 4138 Section 5.3 — Steady state acceptance criteria.

    Scans the time series using a sliding window.
    A window is accepted as steady state when ALL of:
      1. Duration >= MIN_STEADY_STATE_DURATION_S
      2. Lateral accel std < max_ay_variation_g
      3. Roll angle std < max_roll_variation_deg
      4. Yaw rate std < max_yawrate_variation_degs
      5. Mean lateral accel is within [min_ay_g, max_ay_g]

    Returns list of (start_idx, end_idx, mean_ay, mean_roll,
                     mean_yawrate) for each accepted window.

    Why sliding window and not just filtering:
        Filtering gives a smooth signal but does not tell you
        WHERE the vehicle was in steady state. The window
        detector finds specific time periods where the vehicle
        was genuinely settled, so we can extract one data point
        per speed step as ISO 4138 requires.
    """
    print('[ISO4138] Detecting steady state windows...')

    rate_hz      = len(df) / df['Time_s'].iloc[-1]
    window_size  = int(STEADY_STATE['min_duration_s'] * rate_hz)
    windows      = []

    ay   = df['LatAccel_filt_g'].values
    roll = df['Roll_filt_deg'].values
    yr   = df['YawRate_filt_degs'].values

    i = 0
    while i < len(df) - window_size:
        w_ay   = ay  [i : i + window_size]
        w_roll = roll[i : i + window_size]
        w_yr   = yr  [i : i + window_size]

        mean_ay   = np.mean(w_ay)
        std_ay    = np.std(w_ay)
        std_roll  = np.std(w_roll)
        std_yr    = np.std(w_yr)

        # Check all acceptance criteria
        ay_in_range   = (STEADY_STATE['min_ay_g'] <=
                         abs(mean_ay) <=
                         STEADY_STATE['max_ay_g'])
        ay_stable     = std_ay   < STEADY_STATE['max_ay_variation_g']
        roll_stable   = std_roll < STEADY_STATE['max_roll_variation_deg']
        yr_stable     = std_yr   < STEADY_STATE['max_yawrate_variation_degs']

        if ay_in_range and ay_stable and roll_stable and yr_stable:
            windows.append({
                'start_idx'   : i,
                'end_idx'     : i + window_size,
                'start_s'     : df['Time_s'].iloc[i],
                'end_s'       : df['Time_s'].iloc[i + window_size - 1],
                'mean_ay_g'   : mean_ay,
                'mean_roll_deg': np.mean(w_roll),
                'mean_yr_degs': np.mean(w_yr),
                'std_ay'      : std_ay,
                'std_roll'    : std_roll,
            })
            # Jump past this window to avoid overlapping
            i += window_size
        else:
            i += 1

    print(f'[ISO4138] Steady state windows found: {len(windows)}')
    return windows


# ── Separate CW and CCW ───────────────────────────────────
def separate_directions(windows):
    """
    Separate steady state windows by cornering direction.

    CW  (clockwise)     : negative lateral acceleration
    CCW (anticlockwise) : positive lateral acceleration

    ISO 4138 requires both directions to be tested and
    results averaged to cancel sensor bias and road camber.
    """
    cw  = [w for w in windows if w['mean_ay_g'] < 0]
    ccw = [w for w in windows if w['mean_ay_g'] > 0]

    print(f'[ISO4138] CW  windows : {len(cw)}')
    print(f'[ISO4138] CCW windows : {len(ccw)}')
    print()

    return cw, ccw


# ── Calculate Gradients ───────────────────────────────────
def calculate_gradients(windows, direction_label):
    """
    ISO 4138 Section 6 — Gradient calculations.

    Roll Gradient:
        Slope of linear regression: Roll angle vs Lat accel
        Units: deg/g
        Typical range: 3-15 deg/g for passenger cars
                       8-20 deg/g for SUVs
                       5-15 deg/g for light EVs

    Yaw Rate Gradient:
        Slope of: Yaw rate vs Lat accel
        Units: (deg/s) / g
        Related to understeer: higher yaw rate gradient
        at given lateral accel = more oversteer tendency

    R² value:
        Quality of the linear fit.
        ISO 4138 requires R² > 0.98 for valid results.
        Lower R² means insufficient steady state data
        or non-linear behavior at the tested conditions.
    """
    if len(windows) < 3:
        print(f'[ISO4138] {direction_label}: Insufficient windows '
              f'({len(windows)}) for regression. Need >= 3.')
        return None

    ay_vals   = np.array([w['mean_ay_g']    for w in windows])
    roll_vals = np.array([w['mean_roll_deg'] for w in windows])
    yr_vals   = np.array([w['mean_yr_degs']  for w in windows])

    # Use absolute values for consistent sign convention
    ay_abs   = np.abs(ay_vals)
    roll_abs = np.abs(roll_vals)
    yr_abs   = np.abs(yr_vals)

    # Linear regression — Roll gradient
    slope_rg, intercept_rg, r_rg, _, se_rg = linregress(ay_abs, roll_abs)
    r2_rg = r_rg**2

    # Linear regression — Yaw rate gradient
    slope_yr, intercept_yr, r_yr, _, se_yr = linregress(ay_abs, yr_abs)
    r2_yr = r_yr**2

    results = {
        'direction'       : direction_label,
        'n_windows'       : len(windows),
        'roll_gradient'   : slope_rg,
        'roll_intercept'  : intercept_rg,
        'roll_r2'         : r2_rg,
        'roll_se'         : se_rg,
        'yr_gradient'     : slope_yr,
        'yr_intercept'    : intercept_yr,
        'yr_r2'           : r2_yr,
        'ay_range'        : (ay_abs.min(), ay_abs.max()),
        'ay_vals'         : ay_abs,
        'roll_vals'       : roll_abs,
        'yr_vals'         : yr_abs,
    }

    # ISO 4138 quality check
    quality = 'PASS' if r2_rg >= 0.98 else 'FAIL (need more steady state data)'

    print(f'[ISO4138] {direction_label} Results:')
    print(f'          Roll Gradient   : {slope_rg:.3f} deg/g  '
          f'(R²={r2_rg:.4f}) [{quality}]')
    print(f'          Yaw Rate Grad   : {slope_yr:.3f} (deg/s)/g  '
          f'(R²={r2_yr:.4f})')
    print(f'          ay range        : {ay_abs.min():.3f} – '
          f'{ay_abs.max():.3f} g')
    print(f'          Windows used    : {len(windows)}')
    print()

    return results


# ── Combined Gradient ─────────────────────────────────────
def combined_gradient(cw_results, ccw_results):
    """
    ISO 4138 Section 6.4 — Average CW and CCW results.

    Averaging cancels:
      - Road camber effect (tilts ay reading)
      - Sensor Y-axis bias
      - Asymmetric tire wear

    This is the reported vehicle roll gradient.
    """
    if cw_results is None and ccw_results is None:
        return None

    gradients = [r['roll_gradient'] for r in
                 [cw_results, ccw_results] if r is not None]
    combined  = np.mean(gradients)

    print(f'[ISO4138] Combined Roll Gradient (avg CW+CCW): '
          f'{combined:.3f} deg/g')
    print()

    return combined


# ── Dashboard Plot ────────────────────────────────────────
def plot_iso4138(df, windows, cw_results, ccw_results,
                 combined_rg):
    """
    ISO 4138 analysis dashboard — 5 panels.

    Panel 1: Full time history — filtered signals
    Panel 2: Steady state windows highlighted on time history
    Panel 3: Roll angle vs lateral acceleration (scatter + fit)
    Panel 4: Yaw rate vs lateral acceleration (scatter + fit)
    Panel 5: Frequency content — shows why LPF is needed
    """
    fig = plt.figure(figsize=(14, 16))
    fig.suptitle(
        f'ISO 4138 Steady State Analysis — {VEHICLE["name"]} — '
        f'R={TEST["radius_m"]}m',
        fontsize=12, fontweight='bold'
    )

    gs = gridspec.GridSpec(5, 1, hspace=0.55)
    t  = df['Time_s']

    # ── Panel 1: Time History ─────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df['LatAccel_raw_g'],
             alpha=0.3, color='steelblue',
             linewidth=0.5, label='Lat Accel Raw')
    ax1.plot(t, df['LatAccel_filt_g'],
             color='steelblue', linewidth=1.2,
             label='Lat Accel Filtered (2Hz)')
    ax1.plot(t, df['Roll_filt_deg'] / 10,
             color='darkorange', linewidth=1.2,
             label='Roll /10 (deg)')
    ax1.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax1.set_title('Time History — Filtered Signals')
    ax1.set_ylabel('g  /  deg/10')
    ax1.legend(loc='upper right', ncol=3, fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(t.iloc[0], t.iloc[-1])

    # ── Panel 2: Steady State Windows Highlighted ─────────
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, df['LatAccel_filt_g'],
             color='steelblue', linewidth=0.8,
             label='Lat Accel Filtered')

    # Shade each accepted steady state window
    for w in windows:
        color = 'green' if w['mean_ay_g'] > 0 else 'red'
        ax2.axvspan(w['start_s'], w['end_s'],
                    alpha=0.3, color=color)

    ax2.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax2.set_title('Steady State Windows  '
                  '(Green=CCW  Red=CW  Shaded=Accepted)')
    ax2.set_ylabel('Lat Accel (g)')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(t.iloc[0], t.iloc[-1])

    # ── Panel 3: Roll Gradient ────────────────────────────
    ax3 = fig.add_subplot(gs[2])

    for results, color, label in [
        (cw_results,  'red',   'CW'),
        (ccw_results, 'green', 'CCW')
    ]:
        if results is None:
            continue
        ax3.scatter(results['ay_vals'], results['roll_vals'],
                    color=color, s=40, zorder=5,
                    label=f'{label} data points')
        # Regression line
        ay_line   = np.linspace(0, results['ay_vals'].max() * 1.1, 100)
        roll_line = (results['roll_gradient'] * ay_line +
                     results['roll_intercept'])
        ax3.plot(ay_line, roll_line,
                 color=color, linewidth=1.5, linestyle='--',
                 label=f'{label} fit: {results["roll_gradient"]:.3f} deg/g '
                       f'(R²={results["roll_r2"]:.3f})')

    if combined_rg is not None:
        ax3.set_title(
            f'Roll Gradient — Combined: {combined_rg:.3f} deg/g  '
            f'(ISO 4138)'
        )
    else:
        ax3.set_title('Roll Gradient')

    ax3.set_xlabel('Lateral Acceleration (g)')
    ax3.set_ylabel('Roll Angle (deg)')
    ax3.legend(loc='upper left', fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(left=0)
    ax3.set_ylim(bottom=0)

    # ── Panel 4: Yaw Rate Gradient ────────────────────────
    ax4 = fig.add_subplot(gs[3])

    for results, color, label in [
        (cw_results,  'red',   'CW'),
        (ccw_results, 'green', 'CCW')
    ]:
        if results is None:
            continue
        ax4.scatter(results['ay_vals'], results['yr_vals'],
                    color=color, s=40, zorder=5,
                    label=f'{label} data points')
        ay_line  = np.linspace(0, results['ay_vals'].max() * 1.1, 100)
        yr_line  = (results['yr_gradient'] * ay_line +
                    results['yr_intercept'])
        ax4.plot(ay_line, yr_line,
                 color=color, linewidth=1.5, linestyle='--',
                 label=f'{label} fit: {results["yr_gradient"]:.3f} '
                       f'(deg/s)/g  (R²={results["yr_r2"]:.3f})')

    ax4.set_title('Yaw Rate vs Lateral Acceleration')
    ax4.set_xlabel('Lateral Acceleration (g)')
    ax4.set_ylabel('Yaw Rate (deg/s)')
    ax4.legend(loc='upper left', fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(left=0)
    ax4.set_ylim(bottom=0)

    # ── Panel 5: Frequency Analysis ───────────────────────
    ax5 = fig.add_subplot(gs[4])

    rate_hz = len(df) / t.iloc[-1]
    ay_raw  = df['LatAccel_raw_g'].values
    n       = len(ay_raw)
    freqs   = np.fft.rfftfreq(n, d=1.0/rate_hz)
    fft_mag = np.abs(np.fft.rfft(ay_raw)) / n

    ax5.semilogy(freqs, fft_mag,
                 color='steelblue', linewidth=0.8,
                 label='Lateral Accel spectrum')
    ax5.axvline(x=FILTER['accel_lpf_hz'],
                color='red', linewidth=1.5, linestyle='--',
                label=f'LPF cutoff ({FILTER["accel_lpf_hz"]} Hz)')
    ax5.axvspan(0, FILTER['accel_lpf_hz'],
                alpha=0.1, color='green',
                label='Vehicle body motion (kept)')
    ax5.axvspan(FILTER['accel_lpf_hz'], freqs[-1],
                alpha=0.1, color='red',
                label='Corrections + noise (removed)')
    ax5.set_title('Frequency Content — Why Low Pass Filter is Applied')
    ax5.set_xlabel('Frequency (Hz)')
    ax5.set_ylabel('Magnitude (g)')
    ax5.set_xlim(0, min(20, freqs[-1]))
    ax5.legend(loc='upper right', fontsize=8)
    ax5.grid(True, alpha=0.3)

    out_path = plots_path('iso4138_analysis.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'[ISO4138] Saved: {out_path}')
    plt.show()
    return fig


# ── Print Final Report ────────────────────────────────────
def print_report(cw_results, ccw_results, combined_rg):
    """
    ISO 4138 style text report.
    """
    def _fmt(val, unit=''):
        return f'{val} {unit}'.strip() if val is not None else '—'

    print()
    print('=' * 55)
    print(f'  ISO 4138 ANALYSIS REPORT')
    print(f'  Vehicle  : {VEHICLE["name"]}')
    print(f'  Mass     : {_fmt(VEHICLE["mass_kg"], "kg")}')
    print(f'  Wheelbase: {_fmt(VEHICLE["wheelbase_m"], "m")}')
    print(f'  CG height: {_fmt(VEHICLE["cg_height_m"], "m")}')
    print(f'  CG front : {_fmt(VEHICLE["cg_to_front_m"], "m")}')
    print(f'  CG rear  : {_fmt(VEHICLE["cg_to_rear_m"], "m")}')
    print(f'  Track    : {_fmt(VEHICLE["track_width_m"], "m")}')
    print(f'  Front    : {_fmt(VEHICLE["front_mass_kg"], "kg")}')
    print(f'  Rear     : {_fmt(VEHICLE["rear_mass_kg"], "kg")}')
    print(f'  Radius   : {TEST["radius_m"]} m')
    print('=' * 55)

    for results in [cw_results, ccw_results]:
        if results is None:
            continue
        q = 'PASS' if results['roll_r2'] >= 0.98 else 'INSUFFICIENT DATA'
        print(f'  Direction        : {results["direction"]}')
        print(f'  Windows used     : {results["n_windows"]}')
        print(f'  ay range         : {results["ay_range"][0]:.3f}'
              f' – {results["ay_range"][1]:.3f} g')
        print(f'  Roll Gradient    : {results["roll_gradient"]:.3f} deg/g')
        print(f'  R²               : {results["roll_r2"]:.4f}  [{q}]')
        print(f'  Yaw Rate Grad    : {results["yr_gradient"]:.3f} (deg/s)/g')
        print('-' * 55)

    if combined_rg is not None:
        print(f'  COMBINED Roll Gradient : {combined_rg:.3f} deg/g')

    print()
    print('  NOTE: Understeer gradient requires steering angle')
    print('  sensor. Connect sensor and re-run for full ISO 4138.')
    print('=' * 55)


# ── Entry Point ───────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Step 11 — ISO 4138 steady-state handling analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Example:\n'
            '  python step11_iso4138_analysis.py \\\n'
            '    --csv imu_log_20260629_110120.csv \\\n'
            '    --name EV656 --mass 422 --wheelbase 2.07 \\\n'
            '    --cg-height 0.633 --cg-front 1.2508 \\\n'
            '    --track-width 0.875 --front-mass 159 --radius 10\n\n'
            'Parameters needed for Roll Gradient  : --name, --radius\n'
            'Parameters needed for future UG      : --wheelbase, --cg-height,\n'
            '                                       --cg-front, --track-width,\n'
            '                                       --front-mass (+ steering sensor)\n'
        )
    )

    # ── Data source ───────────────────────────────────────
    parser.add_argument('--csv', metavar='FILE',
                        help='CSV file to analyse (default: latest imu_log_*.csv)')

    # ── Vehicle identification ────────────────────────────
    parser.add_argument('--name', required=True, metavar='NAME',
                        help='Vehicle name or identifier, e.g. EV656')

    # ── Geometry and mass (needed for UG; used as metadata for RG) ───
    parser.add_argument('--mass', type=float, default=None, metavar='KG',
                        help='Total vehicle mass (kg)')
    parser.add_argument('--wheelbase', type=float, default=None, metavar='M',
                        help='Wheelbase (m)  [needed for UG]')
    parser.add_argument('--cg-height', type=float, default=None, metavar='M',
                        help='CG height above ground (m)  [needed for UG]')
    parser.add_argument('--cg-front', type=float, default=None, metavar='M',
                        help='Distance from CG to front axle (m)  [needed for UG]')
    parser.add_argument('--track-width', type=float, default=None, metavar='M',
                        help='Track width (m)  [needed for UG]')
    parser.add_argument('--front-mass', type=float, default=None, metavar='KG',
                        help='Front axle mass (kg); rear = total - front  [needed for UG]')

    # ── Test parameters ───────────────────────────────────
    parser.add_argument('--radius', type=float, default=10.0, metavar='M',
                        help='Constant radius of test circle (m, default: 10)')

    args = parser.parse_args()

    # ── Derive dependent values ───────────────────────────
    cg_to_rear = (
        (args.wheelbase - args.cg_front)
        if (args.wheelbase is not None and args.cg_front is not None)
        else None
    )
    rear_mass = (
        (args.mass - args.front_mass)
        if (args.mass is not None and args.front_mass is not None)
        else None
    )

    # ── Build configuration dicts ─────────────────────────
    VEHICLE.update({
        'name'          : args.name,
        'mass_kg'       : args.mass,
        'wheelbase_m'   : args.wheelbase,
        'track_width_m' : args.track_width,
        'cg_height_m'   : args.cg_height,
        'cg_to_front_m' : args.cg_front,
        'cg_to_rear_m'  : cg_to_rear,
        'front_mass_kg' : args.front_mass,
        'rear_mass_kg'  : rear_mass,
    })
    TEST.update({
        'radius_m': args.radius,
    })

    # ── Run pipeline ──────────────────────────────────────
    df           = load_csv(args.csv)
    df           = compute_signals(df)
    windows      = detect_steady_state_windows(df)
    cw_wins, ccw_wins = separate_directions(windows)
    cw_results   = calculate_gradients(cw_wins,  'CW')
    ccw_results  = calculate_gradients(ccw_wins, 'CCW')
    combined_rg  = combined_gradient(cw_results, ccw_results)

    print_report(cw_results, ccw_results, combined_rg)
    plot_iso4138(df, windows, cw_results, ccw_results, combined_rg)