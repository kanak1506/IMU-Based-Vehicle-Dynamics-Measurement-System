import time
from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
import streamlit as st

from dashboard import pipeline, presets, runner, state
from dashboard.pipeline import PipelineState
from dashboard.runner import LogState

_THEME_CSS = """
<style>
div[data-testid="stMetric"] {
    background-color: #FFFFFF;
    border: 1px solid #E2E5EE;
    border-top: 3px solid #2E56A6;
    border-radius: 10px;
    padding: 0.75rem 1rem;
    box-shadow: 0 1px 3px rgba(18, 23, 43, 0.06);
}
[data-testid="stMetricLabel"] p {
    color: #2E56A6;
    font-weight: 600;
    text-transform: uppercase;
    font-size: 0.72rem;
    letter-spacing: 0.02em;
}
div[data-testid="stMetricValue"] {
    color: #12172B;
}
</style>
"""


def render() -> None:
    st.markdown(_THEME_CSS, unsafe_allow_html=True)
    st.title("IMU Vehicle Dynamics Dashboard")
    st.divider()

    col_left, col_right = st.columns([3, 2])

    with col_left:
        _vehicle_config()

    with col_right:
        _test_control()

    st.divider()
    _results()


# ── Vehicle Configuration ─────────────────────────────────

def _vehicle_config() -> None:
    st.subheader("Vehicle Configuration")

    st.text_input("Vehicle Name", placeholder="e.g. EV656", key=state.NAME)

    c1, c2 = st.columns(2)
    c1.number_input("Mass (kg)",       min_value=0.0, step=1.0,   format="%.1f", key=state.MASS)
    c2.number_input("Wheelbase (m)",   min_value=0.0, step=0.001, format="%.3f", key=state.WHEELBASE)
    c1.number_input("CG Height (m)",   min_value=0.0, step=0.001, format="%.3f", key=state.CG_HEIGHT)
    c2.number_input("CG Front (m)",    min_value=0.0, step=0.001, format="%.3f", key=state.CG_FRONT)
    c1.number_input("Track Width (m)", min_value=0.0, step=0.001, format="%.3f", key=state.TRACK_WIDTH)
    c2.number_input("Front Mass (kg)", min_value=0.0, step=1.0,   format="%.1f", key=state.FRONT_MASS)

    _preset_controls()


def _preset_controls() -> None:
    st.markdown("**Presets**")

    msg = st.session_state.pop(state.PRESET_MSG, None)
    if msg:
        (st.error if msg["type"] == "error" else st.success)(msg["text"])

    available = presets.list_presets()

    if available:
        col_sel, col_load = st.columns([3, 1])
        col_sel.selectbox(
            "Preset",
            options=available,
            key="preset_selector",
            label_visibility="collapsed",
        )
        col_load.button(
            "Load",
            on_click=_on_load,
            use_container_width=True,
            key="btn_load_preset",
        )
    else:
        st.caption("No saved presets.")

    st.button(
        "Save Preset",
        on_click=_on_save,
        use_container_width=True,
        key="btn_save_preset",
    )


# ── Preset callbacks ──────────────────────────────────────

def _on_load() -> None:
    name = st.session_state.get("preset_selector")
    if not name:
        return
    try:
        data = presets.load_preset(name)
    except Exception as exc:
        st.session_state[state.PRESET_MSG] = {
            "type": "error",
            "text": f"Could not load preset '{name}': {exc}",
        }
        return

    for widget_key, json_key in state.VEHICLE_FIELDS.items():
        if json_key in data:
            st.session_state[widget_key] = data[json_key]

    st.session_state[state.PRESET_MSG] = {
        "type": "success",
        "text": f"Loaded preset '{name}'.",
    }


def _on_save() -> None:
    vehicle = _collect_vehicle()
    errors  = _validate_vehicle(vehicle)

    if errors:
        st.session_state[state.PRESET_MSG] = {
            "type": "error",
            "text": "\n".join(errors),
        }
        return

    try:
        presets.save_preset(vehicle)
        st.session_state[state.PRESET_MSG] = {
            "type": "success",
            "text": f"Saved preset '{vehicle['name']}'.",
        }
    except Exception as exc:
        st.session_state[state.PRESET_MSG] = {
            "type": "error",
            "text": f"Could not save preset: {exc}",
        }


# ── Helpers ───────────────────────────────────────────────

def _collect_vehicle() -> dict:
    return {
        json_key: st.session_state.get(widget_key, "" if widget_key == state.NAME else 0.0)
        for widget_key, json_key in state.VEHICLE_FIELDS.items()
    }


def _validate_vehicle(vehicle: dict) -> list[str]:
    errors: list[str] = []

    if not str(vehicle.get("name", "")).strip():
        errors.append("Vehicle Name is required.")

    numeric_fields = [
        ("Mass",        "mass_kg"),
        ("Wheelbase",   "wheelbase_m"),
        ("CG Height",   "cg_height_m"),
        ("CG Front",    "cg_to_front_m"),
        ("Track Width", "track_width_m"),
        ("Front Mass",  "front_mass_kg"),
    ]
    for label, key in numeric_fields:
        if vehicle.get(key, 0.0) <= 0.0:
            errors.append(f"{label} must be greater than 0.")

    if (
        vehicle.get("front_mass_kg", 0.0) > 0.0
        and vehicle.get("mass_kg", 0.0) > 0.0
        and vehicle["front_mass_kg"] >= vehicle["mass_kg"]
    ):
        errors.append("Front Mass must be less than total Mass.")

    if (
        vehicle.get("cg_to_front_m", 0.0) > 0.0
        and vehicle.get("wheelbase_m", 0.0) > 0.0
        and vehicle["cg_to_front_m"] >= vehicle["wheelbase_m"]
    ):
        errors.append("CG Front must be less than Wheelbase.")

    return errors


# ── Test Control ──────────────────────────────────────────

_STATUS_LABELS: dict[str, tuple[str, str]] = {
    "idle":    ("Waiting",           "gray"),
    "waiting": ("Waiting for ESP32", "orange"),
    "logging": ("Logging",           "green"),
    "done":    ("Done",              "blue"),
    "error":   ("Error",             "red"),
}


def _test_control() -> None:
    st.subheader("Test Control")

    ls: Optional[LogState]     = st.session_state.get(state.LOGGER_STATE)
    ps: Optional[PipelineState] = st.session_state.get(state.PIPELINE_STATE)

    is_logging  = ls is not None and ls.status in ("waiting", "logging")
    is_pipeline = ps is not None and ps.status == "running"

    # Auto-trigger pipeline the moment logging transitions to done
    if (
        ls is not None
        and ls.status == "done"
        and ls.csv_path
        and (ps is None or ps.status == "idle")
    ):
        ps = PipelineState()
        st.session_state[state.PIPELINE_STATE] = ps
        pipeline.start(
            csv_path=ls.csv_path,
            vehicle=_collect_vehicle(),
            radius=float(st.session_state.get(state.RADIUS, 10.0)),
            ps=ps,
            run_name=Path(ls.csv_path).stem,
        )
        is_pipeline = True

    is_busy = is_logging or is_pipeline

    st.text_input(
        "File Name", placeholder="e.g. Run1 (optional — timestamp used if blank)",
        key=state.FILENAME, disabled=is_busy,
    )
    st.text_input("COM Port", value="COM12", key=state.PORT,   disabled=is_busy)
    st.number_input(
        "Test Radius (m)", min_value=1.0, value=10.0,
        step=0.5, format="%.1f", key=state.RADIUS, disabled=is_busy,
    )

    _status_display(ls, ps)

    if is_logging:
        st.button("STOP TEST", use_container_width=True,
                  key="btn_stop", on_click=_on_stop)
    else:
        st.button(
            "START TEST", type="primary", use_container_width=True,
            key="btn_start", on_click=_on_start, disabled=is_pipeline,
        )

    if is_busy:
        time.sleep(0.5)
        st.rerun()


def _format_duration(seconds: float) -> str:
    m, s = divmod(int(seconds), 60)
    return f"{m:02d}:{s:02d}"


def _status_display(ls: Optional[LogState], ps: Optional[PipelineState]) -> None:
    # Logger status
    log_status = ls.status if ls else "idle"
    label, color = _STATUS_LABELS.get(log_status, ("Unknown", "gray"))
    st.markdown(f"**Logger:** :{color}[{label}]")

    if ls and log_status in ("logging", "done"):
        c1, c2, c3 = st.columns(3)
        c1.metric("Elapsed", _format_duration(ls.elapsed))
        c2.metric("Rows", ls.rows)
        c3.metric("Rate", f"{ls.rate:.1f} /s")

    if ls and log_status == "done" and ls.csv_path:
        st.caption(f"Saved: {Path(ls.csv_path).name}")

    if ls and log_status == "error":
        st.error(ls.error)

    # Pipeline status
    if ps is None or ps.status == "idle":
        return

    if ps.status == "running":
        st.markdown(f"**Analysis:** :orange[{ps.step}]")
    elif ps.status == "done":
        st.markdown("**Analysis:** :green[Complete]")
    elif ps.status == "error":
        st.error(f"Analysis failed — {ps.error}")


# ── Test callbacks ────────────────────────────────────────

def _on_start() -> None:
    port = st.session_state.get(state.PORT, "").strip()
    name = st.session_state.get(state.FILENAME, "").strip()

    ls = LogState()
    st.session_state[state.LOGGER_STATE]   = ls
    st.session_state[state.PIPELINE_STATE] = None

    if not port:
        ls.status = "error"
        ls.error  = "COM Port is required. Enter a port name (e.g. COM12)."
        return

    runner.start(port=port, log_state=ls, name=name)


def _on_stop() -> None:
    ls: Optional[LogState] = st.session_state.get(state.LOGGER_STATE)
    if ls:
        runner.stop(ls)


# ── Results ───────────────────────────────────────────────

def _results() -> None:
    st.subheader("Results")

    ps: Optional[PipelineState] = st.session_state.get(state.PIPELINE_STATE)

    tab_sensor, tab_dynamics, tab_iso = st.tabs(
        ["Raw Sensor", "Vehicle Dynamics", "ISO 4138"]
    )

    with tab_sensor:
        r = _tab_header(ps, "step09", pipeline.IMG_09, "raw sensor plots")
        if r:
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("Rows",       r["rows"])
            c2.metric("Duration",   f'{r["duration_s"]} s')
            c3.metric("Rate",       f'{r["rate_hz"]} Hz')
            c4.metric("|q| min",    f'{r["quat_min"]:.6f}')
            c5.metric("|q| max",    f'{r["quat_max"]:.6f}')

    with tab_dynamics:
        r = _tab_header(ps, "step10", pipeline.IMG_10, "vehicle dynamics analysis")
        if r:
            st.markdown("**Orientation — min / max (deg)**")
            c1, c2, c3 = st.columns(3)
            c1.metric("Roll",  f'{r["roll_min"]:+.1f} / {r["roll_max"]:+.1f}')
            c2.metric("Pitch", f'{r["pitch_min"]:+.1f} / {r["pitch_max"]:+.1f}')
            c3.metric("Yaw",   f'{r["yaw_min"]:+.1f} / {r["yaw_max"]:+.1f}')
            st.markdown("**Acceleration — filtered, min / max (g)**")
            c1, c2, c3 = st.columns(3)
            c1.metric("Longitudinal", f'{r["long_min"]:+.3f} / {r["long_max"]:+.3f}')
            c2.metric("Lateral",      f'{r["lat_min"]:+.3f} / {r["lat_max"]:+.3f}')
            c3.metric("Vertical",     f'{r["vert_min"]:+.3f} / {r["vert_max"]:+.3f}')

    with tab_iso:
        r = _tab_header(ps, "step11", pipeline.IMG_11, "ISO 4138 analysis")
        _method_and_formulae()
        if r:
            _regression_plots(r)
            _iso4138_summary(r)


def _method_and_formulae() -> None:
    with st.expander("Method & Formulae — ISO 4138 Steady-State Analysis"):
        st.markdown("**Method**")
        st.markdown(
            "1. **Filter** — Roll angle, lateral accel, and yaw rate are passed "
            "through a 4th-order Butterworth low-pass filter (2 Hz cutoff) to "
            "remove driver corrections and road noise while preserving vehicle "
            "body motion.\n"
            "2. **Steady-state window detection** — A sliding window "
            "(≥ 2.0 s) is accepted only if, within it: lateral accel std "
            "< 0.02 g, roll std < 0.5°, yaw rate std < 2.0°/s, and mean |aᵧ| "
            "lies between 0.05 g and 0.85 g.\n"
            "3. **Direction split** — Windows are separated into CW "
            "(aᵧ < 0) and CCW (aᵧ > 0) groups.\n"
            "4. **Linear regression** — For each direction, an ordinary "
            "least-squares (OLS) fit is computed over the steady-state points "
            "using `scipy.stats.linregress`.\n"
            "5. **Combine** — CW and CCW gradients are averaged to cancel "
            "sensor bias and road camber."
        )

        st.markdown("**Roll Gradient (RG)**")
        st.latex(r"\varphi = RG \cdot a_y + \varphi_0")
        st.caption(
            "φ = roll angle (deg), aᵧ = lateral acceleration (g), "
            "RG = regression slope (deg/g), φ₀ = intercept."
        )

        st.markdown("**Yaw Rate Gradient (YRG)**")
        st.latex(r"\dot{\psi} = YRG \cdot a_y + \dot{\psi}_0")
        st.caption(
            "ψ̇ = yaw rate (deg/s), YRG = regression slope ((deg/s)/g)."
        )

        st.markdown("**Goodness of fit**")
        st.latex(
            r"R^2 = \left(\frac{\sum (x_i-\bar{x})(y_i-\bar{y})}"
            r"{\sqrt{\sum (x_i-\bar{x})^2 \sum (y_i-\bar{y})^2}}\right)^2"
        )
        st.caption("ISO 4138 requires R² ≥ 0.98 for the result to be valid.")

        st.markdown("**Combined Roll Gradient**")
        st.latex(r"RG_{combined} = \frac{RG_{CW} + RG_{CCW}}{2}")

        st.markdown("**Estimated Speed (IMU-only)**")
        st.latex(r"v = r \cdot |\dot{\psi}|")
        st.caption(
            "r = test circle radius (m), ψ̇ in rad/s — used until a dedicated "
            "speed sensor is added."
        )


def _regression_plots(r: dict) -> None:
    """Dedicated regression chart, rendered live from pipeline results (not the
    static composite image): roll vs lat-accel and yaw-rate vs lat-accel,
    scatter of steady-state points with the OLS fit line, per direction."""
    cw, ccw = r.get("cw"), r.get("ccw")
    if cw is None and ccw is None:
        return

    st.markdown("**Regression Plot**")
    fig, (ax_roll, ax_yr) = plt.subplots(1, 2, figsize=(11, 4.5))

    for res, color, label in [(cw, "red", "CW"), (ccw, "green", "CCW")]:
        if res is None:
            continue
        ay_line = np.linspace(0, res["ay_vals"].max() * 1.1, 100)

        ax_roll.scatter(res["ay_vals"], res["roll_vals"], color=color, s=35,
                         zorder=5, label=f"{label} data")
        ax_roll.plot(
            ay_line, res["roll_gradient"] * ay_line + res["roll_intercept"],
            color=color, linewidth=1.5, linestyle="--",
            label=f'{label} fit: {res["roll_gradient"]:.3f} deg/g (R²={res["roll_r2"]:.3f})',
        )

        ax_yr.scatter(res["ay_vals"], res["yr_vals"], color=color, s=35,
                      zorder=5, label=f"{label} data")
        ax_yr.plot(
            ay_line, res["yr_gradient"] * ay_line + res["yr_intercept"],
            color=color, linewidth=1.5, linestyle="--",
            label=f'{label} fit: {res["yr_gradient"]:.3f} (deg/s)/g (R²={res["yr_r2"]:.3f})',
        )

    ax_roll.set_title("Roll Gradient")
    ax_roll.set_xlabel("Lateral Acceleration (g)")
    ax_roll.set_ylabel("Roll Angle (deg)")
    ax_roll.set_xlim(left=0)
    ax_roll.set_ylim(bottom=0)
    ax_roll.legend(fontsize=7, loc="upper left")
    ax_roll.grid(True, alpha=0.3)

    ax_yr.set_title("Yaw Rate Gradient")
    ax_yr.set_xlabel("Lateral Acceleration (g)")
    ax_yr.set_ylabel("Yaw Rate (deg/s)")
    ax_yr.set_xlim(left=0)
    ax_yr.set_ylim(bottom=0)
    ax_yr.legend(fontsize=7, loc="upper left")
    ax_yr.grid(True, alpha=0.3)

    fig.tight_layout()
    st.pyplot(fig)
    plt.close(fig)


def _tab_header(
    ps: Optional[PipelineState],
    step_key: str,
    img_fallback: str,
    placeholder_label: str,
) -> Optional[dict]:
    """
    Renders the image for a results tab and returns the result dict when fresh
    pipeline data is available, or None otherwise.

    Three states:
      1. Fresh data in ps.results  → show image, return result dict
      2. Image file exists on disk → show image with stale-data note, return None
      3. Nothing available         → show placeholder, return None
    """
    r = ps.results.get(step_key) if ps else None
    if r:
        _show_image(r["img"])
        if r.get("archive"):
            st.caption(f"Archived as: {Path(r['archive']).name}")
        return r
    if Path(img_fallback).exists():
        st.caption("Showing image from previous session — run a new test to refresh.")
        _show_image(img_fallback)
        return None
    _pipeline_placeholder(ps, placeholder_label)
    return None


def _show_image(path: str) -> None:
    p = Path(path)
    if p.exists():
        st.image(str(p), use_container_width=True)
    else:
        st.warning(f"Image not found: {path}")


def _pipeline_placeholder(ps: Optional[PipelineState], label: str) -> None:
    if ps and ps.status == "running":
        st.info(f"Processing... {ps.step}")
    elif ps and ps.status == "error":
        st.error(ps.error)
    else:
        st.info(f"No data yet. Run a test to see {label}.")


def _iso4138_summary(r: dict) -> None:
    st.metric("Total steady state windows", r["n_windows"])

    for key, direction in [("cw", "CW"), ("ccw", "CCW")]:
        res = r.get(key)
        st.markdown(f"**{direction}**")
        if res is None:
            st.markdown(":orange[Insufficient windows for regression (need ≥ 3).]")
            continue
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Windows used",    res["n_windows"])
        c2.metric("Roll Gradient",   f'{res["roll_gradient"]:.3f} deg/g')
        c3.metric("R²",              f'{res["roll_r2"]:.4f}')
        c4.metric("Yaw Rate Grad",   f'{res["yr_gradient"]:.3f} (deg/s)/g')
        ay_lo, ay_hi = res["ay_range"]
        quality = ":green[PASS]" if res["roll_r2"] >= 0.98 else ":red[FAIL — need more steady state data]"
        st.markdown(
            f"ay range tested: **{ay_lo:.3f} – {ay_hi:.3f} g** &nbsp;|&nbsp; Quality: {quality}"
        )

    if r.get("combined_rg") is not None:
        st.divider()
        st.metric("Combined Roll Gradient (avg CW + CCW)", f'{r["combined_rg"]:.3f} deg/g')

    if r.get("results_csv"):
        st.caption(f"Results saved to: {r['results_csv']}")
