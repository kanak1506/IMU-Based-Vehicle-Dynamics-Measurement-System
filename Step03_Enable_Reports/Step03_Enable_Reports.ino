/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 03 — Enable Sensor Reports & Verify Report IDs
 * =========================================================
 *
 *  Purpose:
 *    Call getSensorEvent() in loop() for the first time and
 *    verify that the BNO085 is delivering all three expected
 *    sensor report streams by inspecting raw report IDs.
 *
 *  New in this step:
 *    - getSensorEvent() called in loop()
 *    - Arrival counters per report type
 *    - Rate-limited 1Hz status printout (no serial flooding)
 *    - Report ID mapped to human-readable name
 *    - No sensor data values extracted yet (Step 05)
 *
 *  Hardware:
 *    ESP32 Dev Module
 *    SparkFun BNO085 breakout (SA0 pulled HIGH -> 0x4B)
 *
 *  Wiring (SDA/SCL intentionally swapped vs ESP32 standard):
 *    BNO085 SDA -> ESP32 GPIO 22
 *    BNO085 SCL -> ESP32 GPIO 21
 *    BNO085 PS0 -> GND   (I2C mode)
 *    BNO085 PS1 -> GND   (I2C mode)
 *    BNO085 VCC -> 3.3V
 *    BNO085 GND -> GND
 *
 *  Library:
 *    SparkFun BNO08x Cortex Based IMU v1.0.6
 *
 *  Author:  Vehicle Dynamics IMU Project
 *  Step:    03
 * =========================================================
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ── I2C Pin Configuration ────────────────────────────────
static const int      PIN_SDA              = 22;
static const int      PIN_SCL              = 21;
static const uint32_t I2C_CLOCK_HZ         = 400000;

// ── BNO085 I2C Address ───────────────────────────────────
static const uint8_t  IMU_I2C_ADDRESS      = BNO08x_DEFAULT_ADDRESS;  // 0x4B

// ── Sensor Report Intervals ──────────────────────────────
static const uint32_t RATE_ROTATION_VECTOR_US  = 10000;  // 100 Hz
static const uint32_t RATE_GYROSCOPE_US        = 10000;  // 100 Hz
static const uint32_t RATE_LINEAR_ACCEL_US     = 10000;  // 100 Hz

// ── Status Print Interval ────────────────────────────────
// Print a summary once per second to avoid flooding Serial.
// At 100 Hz we expect ~100 events per report type per second.
static const uint32_t STATUS_PRINT_INTERVAL_MS = 1000;

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── Event Counters ───────────────────────────────────────
// Count how many events of each type arrived since last print.
static uint32_t countRotationVector  = 0;
static uint32_t countGyroscope       = 0;
static uint32_t countLinearAccel     = 0;
static uint32_t countUnknown         = 0;

// ── Timing ───────────────────────────────────────────────
static uint32_t lastStatusPrint_ms   = 0;

// ── Forward Declarations ─────────────────────────────────
void     printBanner(void);
bool     initI2C(void);
bool     initIMU(void);
bool     enableSensorReports(void);
void     handleIMUReset(void);
void     processIMUEvent(void);
void     printStatusReport(void);
const char* reportIDName(uint8_t id);
const char* resetCauseString(uint8_t cause);
void     printFirmwareSummary(void);

// ─────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    printBanner();

    if (!initI2C())             { while (1); }
    if (!initIMU())             { while (1); }

    printFirmwareSummary();

    if (!enableSensorReports()) { while (1); }

    lastStatusPrint_ms = millis();

    Serial.println();
    Serial.println("Listening for sensor events...");
    Serial.println("(Status printed every 1 second)");
    Serial.println("=================================");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * loop()
 *
 * Two responsibilities:
 *   1. Poll for sensor events and count arrivals by type
 *   2. Print a 1Hz status summary showing event counts
 *
 * Why poll and not interrupt-driven at this stage?
 *   The BNO085 INT pin is not connected in this setup.
 *   The library's I2C HAL polls via clock-stretching —
 *   the sensor holds SCL low until it has data.
 *   getSensorEvent() handles all of this internally.
 *   Interrupt-driven operation can be added later by
 *   passing an INT pin to imu.begin().
 */
void loop()
{
    // ── Reset Recovery ───────────────────────────────────
    if (imu.wasReset())
    {
        handleIMUReset();
    }

    // ── Event Processing ─────────────────────────────────
    if (imu.getSensorEvent())
    {
        processIMUEvent();
    }

    // ── 1Hz Status Print ─────────────────────────────────
    uint32_t now_ms = millis();
    if ((now_ms - lastStatusPrint_ms) >= STATUS_PRINT_INTERVAL_MS)
    {
        printStatusReport();
        lastStatusPrint_ms = now_ms;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * processIMUEvent()
 *
 * Called when getSensorEvent() returns true.
 * Reads imu.sensorValue.sensorId and increments the
 * appropriate arrival counter.
 *
 * At this step we intentionally do NOT read data fields
 * (gyro.x, rotationVector.i, etc.) — that is Step 05.
 * We only verify the report ID stream is correct.
 */
void processIMUEvent(void)
{
    switch (imu.sensorValue.sensorId)
    {
        case SH2_ROTATION_VECTOR:
            countRotationVector++;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            countGyroscope++;
            break;

        case SH2_LINEAR_ACCELERATION:
            countLinearAccel++;
            break;

        default:
            // Unexpected report ID — count and flag for investigation
            countUnknown++;
            break;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * printStatusReport()
 *
 * Prints a 1Hz summary of how many events arrived per
 * report type. Expected at 100 Hz: ~100 counts each.
 *
 * Pass criteria:
 *   - All three counts are close to 100 (within ~5%)
 *   - countUnknown is 0
 *
 * Low counts suggest a report was not enabled correctly.
 * High countUnknown means an unexpected report is arriving.
 */
void printStatusReport(void)
{
    Serial.println("---- Event Count (last 1 sec) ----");

    Serial.print("  Rotation Vector   [0x");
    Serial.print(SH2_ROTATION_VECTOR, HEX);
    Serial.print("] : ");
    Serial.println(countRotationVector);

    Serial.print("  Gyroscope Cal     [0x");
    Serial.print(SH2_GYROSCOPE_CALIBRATED, HEX);
    Serial.print("] : ");
    Serial.println(countGyroscope);

    Serial.print("  Linear Accel      [0x");
    Serial.print(SH2_LINEAR_ACCELERATION, HEX);
    Serial.print("] : ");
    Serial.println(countLinearAccel);

    if (countUnknown > 0)
    {
        Serial.print("  *** Unknown ID    [--] : ");
        Serial.print(countUnknown);
        Serial.println("  <- investigate");
    }

    // Verdict
    bool allFlowing = (countRotationVector > 0) &&
                      (countGyroscope      > 0) &&
                      (countLinearAccel    > 0);

    Serial.println(allFlowing ? "  STATUS: All reports flowing. [PASS]"
                              : "  STATUS: One or more reports missing! [FAIL]");
    Serial.println();

    // Reset counters for next interval
    countRotationVector = 0;
    countGyroscope      = 0;
    countLinearAccel    = 0;
    countUnknown        = 0;
}

// ─────────────────────────────────────────────────────────
/*
 * handleIMUReset()
 * Re-enables all sensor reports after a spontaneous reset.
 */
void handleIMUReset(void)
{
    Serial.println();
    Serial.println("[IMU]  *** Spontaneous reset detected! ***");
    Serial.println("[IMU]  Re-enabling sensor reports...");

    if (!enableSensorReports())
    {
        Serial.println("[IMU]  CRITICAL: Could not recover. System halted.");
        while (1);
    }

    // Reset counters so the next status print is not polluted
    // by events from before the reset
    countRotationVector = 0;
    countGyroscope      = 0;
    countLinearAccel    = 0;
    countUnknown        = 0;

    Serial.println("[IMU]  Recovery complete.");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
bool initI2C(void)
{
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    Serial.print("[I2C]  SDA: GPIO "); Serial.println(PIN_SDA);
    Serial.print("[I2C]  SCL: GPIO "); Serial.println(PIN_SCL);
    Serial.print("[I2C]  Clock: ");
    Serial.print(I2C_CLOCK_HZ / 1000);
    Serial.println(" kHz");
    Serial.println();
    return true;
}

// ─────────────────────────────────────────────────────────
bool initIMU(void)
{
    Serial.println("[IMU]  Opening SH-2 session...");

    if (!imu.begin(IMU_I2C_ADDRESS, Wire))
    {
        Serial.println("[IMU]  ERROR: begin() failed.");
        Serial.println("[IMU]  Check wiring, address, power supply.");
        return false;
    }

    Serial.println("[IMU]  SH-2 session opened successfully.");
    return true;
}

// ─────────────────────────────────────────────────────────
bool enableSensorReports(void)
{
    bool ok = true;

    if (!imu.enableReport(SH2_ROTATION_VECTOR, RATE_ROTATION_VECTOR_US))
    {
        Serial.println("[IMU]  Rotation Vector        -> FAILED");
        ok = false;
    }

    if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, RATE_GYROSCOPE_US))
    {
        Serial.println("[IMU]  Gyroscope (calibrated) -> FAILED");
        ok = false;
    }

    if (!imu.enableReport(SH2_LINEAR_ACCELERATION, RATE_LINEAR_ACCEL_US))
    {
        Serial.println("[IMU]  Linear Acceleration    -> FAILED");
        ok = false;
    }

    if (ok)
        Serial.println("[IMU]  All reports enabled successfully.");

    return ok;
}

// ─────────────────────────────────────────────────────────
void printFirmwareSummary(void)
{
    Serial.println("[IMU]  SH-2 Firmware Summary:");
    Serial.println("[IMU]  -----------------------------------------------");
    Serial.println("[IMU]  Idx  Part Number   Version      Build");
    Serial.println("[IMU]  -----------------------------------------------");

    for (uint8_t i = 0; i < imu.prodIds.numEntries; i++)
    {
        Serial.print("[IMU]   ");
        Serial.print(i);
        Serial.print("   ");
        Serial.print(imu.prodIds.entry[i].swPartNumber);
        Serial.print("    v");
        Serial.print(imu.prodIds.entry[i].swVersionMajor);
        Serial.print(".");
        Serial.print(imu.prodIds.entry[i].swVersionMinor);
        Serial.print(".");
        Serial.print(imu.prodIds.entry[i].swVersionPatch);
        Serial.print("    Build ");
        Serial.println(imu.prodIds.entry[i].swBuildNumber);
    }

    Serial.println("[IMU]  -----------------------------------------------");
    Serial.print  ("[IMU]  Reset cause: ");
    Serial.println(resetCauseString(imu.getResetReason()));
    Serial.println();
}

// ─────────────────────────────────────────────────────────
const char* resetCauseString(uint8_t cause)
{
    switch (cause)
    {
        case 1:  return "Power-On Reset";
        case 2:  return "Internal reset (normal after begin())";
        case 3:  return "Watchdog";
        case 4:  return "External reset";
        case 5:  return "Other";
        default: return "Unknown";
    }
}

// ─────────────────────────────────────────────────────────
void printBanner(void)
{
    Serial.println();
    Serial.println("=================================");
    Serial.println(" Vehicle Dynamics IMU Project");
    Serial.println(" Step 03 - Verify Report IDs");
    Serial.println("=================================");
    Serial.println();
}