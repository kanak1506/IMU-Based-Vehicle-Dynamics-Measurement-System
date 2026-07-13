/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 04 — Verify Incoming Report IDs
 * =========================================================
 *
 *  Purpose:
 *    Print the raw SH-2 report ID of every event as it
 *    arrives. Permanently documents which report IDs the
 *    BNO085 is delivering before any data fields are read.
 *
 *  New in this step:
 *    - Every getSensorEvent() call prints its raw sensorId
 *    - Output rate-limited: one snapshot per second
 *    - Raw hex IDs cross-referenced to known SH-2 names
 *    - No data values extracted (that is Step 05)
 *
 *  Expected IDs from SH-2 reference manual:
 *    0x05 = SH2_ROTATION_VECTOR
 *    0x02 = SH2_GYROSCOPE_CALIBRATED
 *    0x04 = SH2_LINEAR_ACCELERATION
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
 *  Step:    04
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

// ── Snapshot Interval ────────────────────────────────────
// Collect incoming IDs for this many ms, then print a
// summary. Keeps Serial readable at 100 Hz event rate.
static const uint32_t SNAPSHOT_INTERVAL_MS = 1000;

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── ID Log Buffer ────────────────────────────────────────
// Stores the raw sensorId of every event in the snapshot
// window. Sized for 3 reports * 100 Hz * 1 sec + margin.
static const uint16_t LOG_BUFFER_SIZE = 400;
static uint8_t  idLog[LOG_BUFFER_SIZE];
static uint16_t idLogCount = 0;

// ── Timing ───────────────────────────────────────────────
static uint32_t lastSnapshot_ms = 0;

// ── Forward Declarations ─────────────────────────────────
void        printBanner(void);
bool        initI2C(void);
bool        initIMU(void);
bool        enableSensorReports(void);
void        handleIMUReset(void);
void        logEventID(uint8_t id);
void        printSnapshot(void);
const char* reportIDName(uint8_t id);
const char* resetCauseString(uint8_t cause);
void        printFirmwareSummary(void);

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

    lastSnapshot_ms = millis();

    Serial.println();
    Serial.println("Logging report IDs...");
    Serial.println("(Snapshot printed every 1 second)");
    Serial.println("=================================");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * loop()
 *
 * Drains all pending SH-2 events per cycle.
 * For each event, logs the raw sensorId into the buffer.
 * Every second, prints a snapshot of all IDs received.
 *
 * Why drain with while() not a single if():
 *   The SH-2 can queue multiple reports between loop()
 *   calls. Using while() ensures we process every queued
 *   event rather than falling behind. This pattern is
 *   carried into all future steps.
 */
void loop()
{
    // ── Reset Recovery ───────────────────────────────────
    if (imu.wasReset())
    {
        handleIMUReset();
    }

    // ── Drain event queue and log IDs ───────────────────
    while (imu.getSensorEvent())
    {
        logEventID(imu.sensorValue.sensorId);
    }

    // ── 1Hz snapshot print ───────────────────────────────
    uint32_t now_ms = millis();
    if ((now_ms - lastSnapshot_ms) >= SNAPSHOT_INTERVAL_MS)
    {
        printSnapshot();
        lastSnapshot_ms = now_ms;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * logEventID()
 *
 * Stores the raw report ID in the log buffer.
 * If the buffer fills (should not happen at 100 Hz / 1 sec
 * with a 400-slot buffer), the oldest entries are silently
 * dropped rather than corrupting memory.
 */
void logEventID(uint8_t id)
{
    if (idLogCount < LOG_BUFFER_SIZE)
    {
        idLog[idLogCount++] = id;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * printSnapshot()
 *
 * Counts unique IDs in the log buffer and prints:
 *   - The raw hex ID
 *   - The known SH-2 name (or UNKNOWN)
 *   - How many times it appeared in the snapshot window
 *
 * Then clears the buffer for the next window.
 *
 * This format makes it immediately obvious if:
 *   - An expected ID is missing (count = 0)
 *   - An unexpected ID is arriving (UNKNOWN)
 *   - One report is arriving at a different rate
 */
void printSnapshot(void)
{
    Serial.print("==== ID Snapshot [");
    Serial.print(idLogCount);
    Serial.println(" events] ====");

    // Count each unique ID in the buffer
    // We only expect a small set of IDs so a linear scan is fine
    uint8_t  seenIds[LOG_BUFFER_SIZE];
    uint16_t seenCounts[LOG_BUFFER_SIZE];
    uint8_t  uniqueCount = 0;

    for (uint16_t i = 0; i < idLogCount; i++)
    {
        uint8_t id = idLog[i];
        bool found = false;

        for (uint8_t j = 0; j < uniqueCount; j++)
        {
            if (seenIds[j] == id)
            {
                seenCounts[j]++;
                found = true;
                break;
            }
        }

        if (!found && uniqueCount < LOG_BUFFER_SIZE)
        {
            seenIds[uniqueCount]    = id;
            seenCounts[uniqueCount] = 1;
            uniqueCount++;
        }
    }

    // Print each unique ID with its count and name
    for (uint8_t i = 0; i < uniqueCount; i++)
    {
        Serial.print("  ID 0x");
        if (seenIds[i] < 0x10) Serial.print("0");
        Serial.print(seenIds[i], HEX);
        Serial.print("  (");
        Serial.print(reportIDName(seenIds[i]));
        Serial.print(")  count: ");
        Serial.println(seenCounts[i]);
    }

    // Verdict
    bool hasRV    = false;
    bool hasGyro  = false;
    bool hasAccel = false;

    for (uint8_t i = 0; i < uniqueCount; i++)
    {
        if (seenIds[i] == SH2_ROTATION_VECTOR)      hasRV    = true;
        if (seenIds[i] == SH2_GYROSCOPE_CALIBRATED) hasGyro  = true;
        if (seenIds[i] == SH2_LINEAR_ACCELERATION)  hasAccel = true;
    }

    bool allPresent = hasRV && hasGyro && hasAccel;
    Serial.println(allPresent
        ? "  VERDICT: All expected IDs present. [PASS]"
        : "  VERDICT: One or more expected IDs missing! [FAIL]");
    Serial.println();

    // Clear buffer for next snapshot window
    idLogCount = 0;
}

// ─────────────────────────────────────────────────────────
/*
 * reportIDName()
 *
 * Maps raw SH-2 sensor IDs to human-readable names.
 * IDs are defined in sh2.h (sh2_SensorId_e enum).
 * Only the three active reports are named here.
 * Any other ID is flagged as UNKNOWN for investigation.
 */
const char* reportIDName(uint8_t id)
{
    switch (id)
    {
        case SH2_ROTATION_VECTOR:      return "SH2_ROTATION_VECTOR";
        case SH2_GYROSCOPE_CALIBRATED: return "SH2_GYROSCOPE_CALIBRATED";
        case SH2_LINEAR_ACCELERATION:  return "SH2_LINEAR_ACCELERATION";
        default:                       return "UNKNOWN -- investigate";
    }
}

// ─────────────────────────────────────────────────────────
void handleIMUReset(void)
{
    Serial.println("[IMU]  *** Spontaneous reset detected! ***");
    idLogCount = 0;

    if (!enableSensorReports())
    {
        Serial.println("[IMU]  CRITICAL: Could not recover. System halted.");
        while (1);
    }
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
    { Serial.println("[IMU]  Rotation Vector        -> FAILED"); ok = false; }

    if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, RATE_GYROSCOPE_US))
    { Serial.println("[IMU]  Gyroscope (calibrated) -> FAILED"); ok = false; }

    if (!imu.enableReport(SH2_LINEAR_ACCELERATION, RATE_LINEAR_ACCEL_US))
    { Serial.println("[IMU]  Linear Acceleration    -> FAILED"); ok = false; }

    if (ok) Serial.println("[IMU]  All reports enabled successfully.");

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
        Serial.print("[IMU]   "); Serial.print(i);
        Serial.print("   ");     Serial.print(imu.prodIds.entry[i].swPartNumber);
        Serial.print("    v");   Serial.print(imu.prodIds.entry[i].swVersionMajor);
        Serial.print(".");       Serial.print(imu.prodIds.entry[i].swVersionMinor);
        Serial.print(".");       Serial.print(imu.prodIds.entry[i].swVersionPatch);
        Serial.print("    Build "); Serial.println(imu.prodIds.entry[i].swBuildNumber);
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
    Serial.println(" Step 04 - Verify Report IDs");
    Serial.println("=================================");
    Serial.println();
}