"""
=========================================================
 Vehicle Dynamics IMU Project
 Step 08 - Python Serial Logger
=========================================================

Purpose:
    Receive the CSV stream from the ESP32, skip comment
    lines, parse the header, and save a timestamped CSV
    file to disk for later analysis.

Usage:
    1. Flash Step 07 firmware to ESP32
    2. Close Arduino IDE Serial Monitor
    3. Open Command Prompt
    4. cd "C:\\Users\\Kanak Potdar\\OneDrive\\Documents\\Arduino"
    5. python step08_serial_logger.py
    6. Press Ctrl+C to stop logging

Output:
    imu_log_YYYYMMDD_HHMMSS.csv in the same directory

Requirements:
    pip install pyserial
=========================================================
"""

import serial
import serial.tools.list_ports
import csv
import re
import sys
import time
from datetime import datetime
from pathlib import Path

# ── Configuration ─────────────────────────────────────────
BAUD_RATE        = 115200
PORT             = 'COM12'
TIMEOUT_SEC      = 5.0
OUTPUT_DIR       = Path('.')
PRINT_EVERY_N    = 100
COMMENT_PREFIX   = '#'

# Exact header string the firmware emits.
# Matching against this prevents boot garbage lines like
# "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)"
# from being mistaken for a valid header.
EXPECTED_HEADER = (
    "Time_us,Quaternion_i,Quaternion_j,Quaternion_k,"
    "Quaternion_real,GyroX,GyroY,GyroZ,"
    "LinearAccelX,LinearAccelY,LinearAccelZ"
)

# Characters that are invalid in filenames on Windows, macOS, and Linux.
_INVALID_CHARS = re.compile(r'[\\/:*?"<>|]')


# ─────────────────────────────────────────────────────────
def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or '').lower()
        if any(chip in desc for chip in
               ['cp210', 'ch340', 'ch341', 'ftdi', 'uart', 'esp']):
            candidates.append(p.device)
    return candidates


# ─────────────────────────────────────────────────────────
def open_output_file(name: str = ""):
    """
    Create the output CSV file. If `name` is given, use it as the base
    filename (falling back to a timestamp if a file with that name already
    exists, so previous test data is never overwritten). Otherwise use the
    original imu_log_<timestamp>.csv scheme.
    """
    safe_name = _INVALID_CHARS.sub('', name).strip() if name else ""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    if safe_name:
        filename = OUTPUT_DIR / f'{safe_name}.csv'
        if filename.exists():
            filename = OUTPUT_DIR / f'{safe_name}_{timestamp}.csv'
    else:
        filename = OUTPUT_DIR / f'imu_log_{timestamp}.csv'

    f      = open(filename, 'w', newline='', encoding='utf-8')
    writer = csv.writer(f)
    return filename, f, writer


# ─────────────────────────────────────────────────────────
def wait_for_header(ser):
    """
    Reads lines until the exact CSV header is found.
    .strip() on every line removes Windows CRLF endings.
    Exact string match prevents boot garbage being accepted.
    """
    print('[Logger] Waiting for CSV header from ESP32...')
    print('[Logger] (Power-cycle ESP32 if this takes >15 seconds)')

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
        except Exception:
            continue

        if not line:
            continue

        if line.startswith(COMMENT_PREFIX):
            print(f'  {line}')
            continue

        # Accept only the exact expected header string
        if line == EXPECTED_HEADER:
            columns = line.split(',')
            print(f'[Logger] Header received: {len(columns)} columns')
            return columns

        # Anything else is boot garbage or noise
        print(f'[Logger] Skipping non-header line: {line[:70]}')


# ─────────────────────────────────────────────────────────
def log_session(port):
    print(f'[Logger] Opening port {port} at {BAUD_RATE} baud...')

    try:
        ser = serial.Serial(
            port     = port,
            baudrate = BAUD_RATE,
            timeout  = TIMEOUT_SEC
        )
    except serial.SerialException as e:
        print(f'[Logger] ERROR: Could not open port: {e}')
        candidates = find_esp32_port()
        if candidates:
            print(f'[Logger] Possible ESP32 ports: {candidates}')
        sys.exit(1)

    print(f'[Logger] Port open. Listening...')

    columns = wait_for_header(ser)
    expected_fields = len(columns)

    filepath, outfile, writer = open_output_file()
    writer.writerow(columns)
    print(f'[Logger] Logging to: {filepath}')
    print(f'[Logger] Press Ctrl+C to stop.')
    print()

    row_count     = 0
    skipped_count = 0
    start_time    = time.time()

    try:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
            except Exception:
                continue

            if not line:
                continue

            if line.startswith(COMMENT_PREFIX):
                print(f'  {line}')
                continue

            fields = line.split(',')

            if len(fields) != expected_fields:
                skipped_count += 1
                continue

            writer.writerow(fields)
            outfile.flush()
            row_count += 1

            if row_count % PRINT_EVERY_N == 0:
                elapsed = time.time() - start_time
                rate    = row_count / elapsed if elapsed > 0 else 0
                print(f'[Logger] Rows: {row_count:6d} | '
                      f'Rate: {rate:5.1f} rows/sec | '
                      f'Skipped: {skipped_count} | '
                      f'Elapsed: {elapsed:6.1f}s')

    except KeyboardInterrupt:
        print()
        print('[Logger] Ctrl+C received. Stopping...')

    finally:
        elapsed = time.time() - start_time
        outfile.close()
        ser.close()

        print()
        print('=' * 50)
        print(f'  Logging complete.')
        print(f'  Rows logged : {row_count}')
        print(f'  Rows skipped: {skipped_count}')
        print(f'  Duration    : {elapsed:.1f} seconds')
        if elapsed > 0:
            print(f'  Avg rate    : {row_count / elapsed:.1f} rows/sec')
        print(f'  File        : {filepath}')
        print('=' * 50)


# ── Entry Point ───────────────────────────────────────────
if __name__ == '__main__':
    log_session(PORT)