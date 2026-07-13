import threading
import time
from typing import Optional

import serial

import step08_serial_logger as step08

_READ_TIMEOUT    = 0.5    # serial read timeout — keeps stop() responsive
_HEADER_TIMEOUT  = 30.0   # seconds to wait for the ESP32 CSV header


class LogState:
    """Mutable state shared between the logger thread and the Streamlit UI."""

    def __init__(self) -> None:
        self.status:   str                        = "idle"
        self.rows:     int                        = 0
        self.rate:     float                      = 0.0
        self.elapsed:  float                      = 0.0
        self.csv_path: str                        = ""
        self.name:     str                        = ""
        self.error:    str                        = ""
        self._stop:    threading.Event            = threading.Event()
        self._thread:  Optional[threading.Thread] = None


def start(port: str, log_state: LogState, name: str = "") -> None:
    """Reset state and launch the logger in a background thread."""
    log_state._stop.clear()
    log_state.rows     = 0
    log_state.rate     = 0.0
    log_state.elapsed  = 0.0
    log_state.csv_path = ""
    log_state.name     = name.strip()
    log_state.error    = ""
    log_state.status   = "waiting"

    log_state._thread = threading.Thread(
        target=_worker,
        args=(port, log_state),
        daemon=True,
    )
    log_state._thread.start()


def stop(log_state: LogState) -> None:
    """Signal the worker to stop. Returns immediately; thread cleans up itself."""
    log_state._stop.set()


# ── Worker ────────────────────────────────────────────────

def _worker(port: str, ls: LogState) -> None:
    """
    Background thread body.
    try/finally guarantees serial port and output file are always closed,
    even if an unexpected exception occurs.
    """
    ser     = None
    outfile = None

    try:
        # ── Open serial port ──────────────────────────────
        try:
            ser = serial.Serial(
                port=port,
                baudrate=step08.BAUD_RATE,
                timeout=_READ_TIMEOUT,
            )
        except (serial.SerialException, ValueError) as exc:
            suggestions = step08.find_esp32_port()
            hint = f"  Try: {suggestions}" if suggestions else "  No ESP32 ports detected."
            ls.status = "error"
            ls.error  = f"Cannot open {port}: {exc}.{hint}"
            return

        # ── Wait for CSV header ───────────────────────────
        columns = _wait_for_header(ser, ls)
        if columns is None:
            # _wait_for_header already set ls.status / ls.error on timeout
            if ls.status not in ("error",):
                ls.status = "idle"
            return

        # ── Open output file ──────────────────────────────
        try:
            filepath, outfile, writer = step08.open_output_file(ls.name)
        except OSError as exc:
            ls.status = "error"
            ls.error  = f"Cannot create output file: {exc}"
            return

        writer.writerow(columns)
        ls.csv_path     = str(filepath)
        ls.status       = "logging"
        expected_fields = len(columns)
        start_time      = time.time()

        # ── Main logging loop ─────────────────────────────
        while not ls._stop.is_set():
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as exc:
                ls.status = "error"
                ls.error  = f"Serial disconnected: {exc}"
                break

            ls.elapsed = time.time() - start_time

            if not line or line.startswith(step08.COMMENT_PREFIX):
                continue

            fields = line.split(",")
            if len(fields) != expected_fields:
                continue

            try:
                writer.writerow(fields)
                outfile.flush()
            except OSError as exc:
                ls.status = "error"
                ls.error  = f"File write failed (disk full?): {exc}"
                break

            ls.rows += 1
            if ls.elapsed > 0:
                ls.rate = round(ls.rows / ls.elapsed, 1)

        # Normal stop — only mark done if no error was set in the loop
        if ls.status == "logging":
            ls.status = "done"

    except Exception as exc:
        # Catch-all for anything not explicitly handled above
        if ls.status not in ("error", "done"):
            ls.status = "error"
            ls.error  = f"Unexpected logger error: {exc}"

    finally:
        if outfile is not None:
            try:
                outfile.close()
            except Exception:
                pass
        if ser is not None and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass


def _wait_for_header(ser: serial.Serial, ls: LogState) -> Optional[list]:
    """
    Read lines until the exact CSV header arrives or a timeout/stop occurs.
    Returns the column list, or None (with ls.status/error set) on failure.
    """
    deadline = time.monotonic() + _HEADER_TIMEOUT

    while not ls._stop.is_set():
        if time.monotonic() > deadline:
            ls.status = "error"
            ls.error  = (
                f"No response from ESP32 after {_HEADER_TIMEOUT:.0f} s. "
                "Check: Step 07 firmware is flashed, Arduino Serial Monitor is closed, "
                "correct COM port is selected."
            )
            return None

        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except serial.SerialException as exc:
            ls.status = "error"
            ls.error  = f"Serial error while waiting for header: {exc}"
            return None

        if not line or line.startswith(step08.COMMENT_PREFIX):
            continue

        if line == step08.EXPECTED_HEADER:
            return line.split(",")

        # Boot garbage or noise — keep waiting

    return None
