import re
import shutil
import sys
import threading
import traceback
from datetime import datetime
from pathlib import Path
from typing import Optional

import pandas as pd
import matplotlib.pyplot as plt

import step09_plots            as step09
import step10_vehicle_dynamics as step10
import step11_iso4138_analysis as step11
from imu_utils import plots_path, results_path

# Suppress the interactive window; images are already saved before show() is called.
plt.show = lambda *a, **kw: None

# Fixed output filenames produced by the step scripts (relative to plots/).
IMG_09 = plots_path("imu_plots.png")
IMG_10 = plots_path("vehicle_dynamics.png")
IMG_11 = plots_path("iso4138_analysis.png")

# Columns the CSV must contain for the pipeline to run.
_REQUIRED_COLS = frozenset({
    "Time_us",
    "Quaternion_i", "Quaternion_j", "Quaternion_k", "Quaternion_real",
    "GyroX", "GyroY", "GyroZ",
    "LinearAccelX", "LinearAccelY", "LinearAccelZ",
})


class PipelineState:
    def __init__(self) -> None:
        self.status:  str                         = "idle"
        self.step:    str                         = ""
        self.error:   str                         = ""
        self.results: dict                        = {}
        self._thread: Optional[threading.Thread] = None


def start(
    csv_path: str,
    vehicle:  dict,
    radius:   float,
    ps:       PipelineState,
    run_name: str = "",
) -> None:
    """Reset state and launch the analysis pipeline in a background thread."""
    ps.status  = "running"
    ps.step    = "Validating..."
    ps.error   = ""
    ps.results = {}

    ps._thread = threading.Thread(
        target=_run,
        args=(csv_path, vehicle, radius, ps, run_name),
        daemon=True,
    )
    ps._thread.start()


# ── Validation ────────────────────────────────────────────

def _validate_csv(csv_path: str) -> Optional[str]:
    """Return a user-friendly error string, or None if the CSV looks valid."""
    p = Path(csv_path)
    if not p.exists():
        return f"CSV file not found: {csv_path}"
    if p.stat().st_size == 0:
        return "CSV file is empty — no data was logged."
    try:
        header = pd.read_csv(csv_path, nrows=0)
    except Exception as exc:
        return f"Cannot read CSV file: {exc}"
    missing = _REQUIRED_COLS - set(header.columns)
    if missing:
        return (
            f"CSV is missing columns: {', '.join(sorted(missing))}. "
            "Check that Step 07 firmware is flashed correctly."
        )
    return None


# ── Worker ────────────────────────────────────────────────

def _run(
    csv_path: str,
    vehicle:  dict,
    radius:   float,
    ps:       PipelineState,
    run_name: str = "",
) -> None:
    try:
        # ── Validate CSV ──────────────────────────────────
        ps.step = "Validating CSV"
        err = _validate_csv(csv_path)
        if err:
            ps.status = "error"
            ps.error  = err
            return

        # ── Step 09 — Raw sensor plots ────────────────────
        ps.step = "Step 09 — Raw sensor plots"
        df09     = step09.load_data(csv_path)

        if len(df09) < 2:
            raise ValueError("CSV contains fewer than 2 rows — not enough data to analyse.")

        duration = float(df09["Time_s"].iloc[-1])
        if duration <= 0:
            raise ValueError("CSV time column is invalid (duration ≤ 0).")

        fig09 = step09.plot_all(df09, csv_path)
        plt.close(fig09)
        archived_09 = _archive_image(IMG_09, run_name, "imu_plots")

        ps.results["step09"] = {
            "img":        IMG_09,
            "archive":    archived_09,
            "rows":       len(df09),
            "duration_s": round(duration, 1),
            "rate_hz":    round(len(df09) / duration, 1),
            "quat_min":   round(float(df09["Quat_mag"].min()), 6),
            "quat_max":   round(float(df09["Quat_mag"].max()), 6),
        }

        # ── Step 10 — Vehicle dynamics ────────────────────
        ps.step = "Step 10 — Vehicle dynamics"
        df10 = step10.load_data(csv_path)
        df10 = step10.compute_dynamics(df10)
        fig10 = step10.plot_dashboard(df10, csv_path)
        plt.close(fig10)
        archived_10 = _archive_image(IMG_10, run_name, "vehicle_dynamics")

        ps.results["step10"] = {
            "img":       IMG_10,
            "archive":   archived_10,
            "roll_min":  round(float(df10["Roll_deg"].min()), 1),
            "roll_max":  round(float(df10["Roll_deg"].max()), 1),
            "pitch_min": round(float(df10["Pitch_deg"].min()), 1),
            "pitch_max": round(float(df10["Pitch_deg"].max()), 1),
            "yaw_min":   round(float(df10["Yaw_deg"].min()), 1),
            "yaw_max":   round(float(df10["Yaw_deg"].max()), 1),
            "long_min":  round(float(df10["LongAccel_filt_g"].min()), 3),
            "long_max":  round(float(df10["LongAccel_filt_g"].max()), 3),
            "lat_min":   round(float(df10["LatAccel_filt_g"].min()), 3),
            "lat_max":   round(float(df10["LatAccel_filt_g"].max()), 3),
            "vert_min":  round(float(df10["VertAccel_filt_g"].min()), 3),
            "vert_max":  round(float(df10["VertAccel_filt_g"].max()), 3),
        }

        # ── Step 11 — ISO 4138 ────────────────────────────
        ps.step = "Step 11 — ISO 4138 analysis"

        step11.VEHICLE.clear()
        step11.VEHICLE.update(vehicle)
        step11.TEST.clear()
        step11.TEST.update({"radius_m": radius})

        df11        = step11.load_csv(csv_path)
        df11        = step11.compute_signals(df11)
        windows     = step11.detect_steady_state_windows(df11)
        cw_wins, ccw_wins = step11.separate_directions(windows)
        cw_r        = step11.calculate_gradients(cw_wins,  "CW")
        ccw_r       = step11.calculate_gradients(ccw_wins, "CCW")
        combined_rg = step11.combined_gradient(cw_r, ccw_r)
        fig11 = step11.plot_iso4138(df11, windows, cw_r, ccw_r, combined_rg)
        plt.close(fig11)
        archived_11 = _archive_image(IMG_11, run_name, "iso4138_analysis")

        ps.results["step11"] = {
            "img":          IMG_11,
            "archive":      archived_11,
            "n_windows":    len(windows),
            "n_windows_cw": len(cw_wins),
            "n_windows_ccw":len(ccw_wins),
            "cw":           cw_r,
            "ccw":          ccw_r,
            "combined_rg":  combined_rg,
        }

        ps.results["step11"]["results_csv"] = _save_results_csv(
            vehicle, radius, Path(csv_path).name, ps.results
        )

        ps.status = "done"
        ps.step   = ""

    except Exception as exc:
        # Log the full traceback to the server console for debugging.
        print(f"[Pipeline] Error at '{ps.step}':", file=sys.stderr)
        print(traceback.format_exc(), file=sys.stderr)
        # Show only the exception message in the UI.
        ps.status = "error"
        ps.error  = f"{ps.step}: {exc}"


# ── Image archiving ───────────────────────────────────────

def _archive_image(src: str, run_name: str, suffix: str) -> Optional[str]:
    """
    Copy a fixed-name plot image to a run_name-tagged file so each test run
    keeps its own plots instead of overwriting the previous run's. Returns
    the archive path, or None if run_name is blank.
    """
    safe_name = re.sub(r'[\\/:*?"<>|]', '', run_name).strip()
    if not safe_name:
        return None

    dest = plots_path(f"{safe_name}_{suffix}.png")
    try:
        shutil.copyfile(src, dest)
    except OSError:
        return None
    return dest


# ── Results CSV ───────────────────────────────────────────

def _save_results_csv(
    vehicle:  dict,
    radius:   float,
    csv_name: str,
    results:  dict,
) -> str:
    """
    Append one summary row to {vehicle_name}_results.csv.
    Each test session adds a row so the file builds a history
    across setup changes. Returns the filepath as a string.
    """
    r11  = results.get("step11", {})
    cw   = r11.get("cw")
    ccw  = r11.get("ccw")

    row = {
        "vehicle_name":          vehicle.get("name", ""),
        "test_date":             datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "session_csv":           csv_name,
        "test_radius_m":         radius,
        "n_windows_total":       r11.get("n_windows",     0),
        "n_windows_cw":          r11.get("n_windows_cw",  0),
        "n_windows_ccw":         r11.get("n_windows_ccw", 0),
        # Roll Gradient
        "rg_cw_deg_per_g":       cw["roll_gradient"]   if cw  else None,
        "rg_cw_r2":              cw["roll_r2"]         if cw  else None,
        "rg_ccw_deg_per_g":      ccw["roll_gradient"]  if ccw else None,
        "rg_ccw_r2":             ccw["roll_r2"]        if ccw else None,
        "rg_combined_deg_per_g": r11.get("combined_rg"),
        # Yaw Rate Gradient
        "yrg_cw_degs_per_g":     cw["yr_gradient"]     if cw  else None,
        "yrg_cw_r2":             cw["yr_r2"]           if cw  else None,
        "yrg_ccw_degs_per_g":    ccw["yr_gradient"]    if ccw else None,
        "yrg_ccw_r2":            ccw["yr_r2"]          if ccw else None,
        # Understeer Gradient — requires steering angle sensor (future)
        "ug_cw_deg_per_g":       None,
        "ug_ccw_deg_per_g":      None,
        "ug_combined_deg_per_g": None,
    }

    name      = str(vehicle.get("name", "unnamed")).strip()
    safe_name = re.sub(r'[\\/:*?"<>|]', '', name).strip() or "unnamed"
    filepath  = results_path(f"{safe_name}_results.csv")

    df_new = pd.DataFrame([row])
    if Path(filepath).exists():
        df_out = pd.concat([pd.read_csv(filepath), df_new], ignore_index=True)
    else:
        df_out = df_new

    df_out.to_csv(filepath, index=False)
    print(f"[Pipeline] Results saved: {filepath}")
    return filepath
