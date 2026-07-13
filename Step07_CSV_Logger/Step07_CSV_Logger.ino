/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 07 — CSV Logger
 * =========================================================
 *
 *  Purpose:
 *    Output a clean, well-formed CSV stream over Serial
 *    that Python can capture, parse, and save to disk.
 *
 *  New in this step:
 *    - Comma-separated output with CSV header line
 *    - All non-data lines prefixed with '#' (Python skips)
 *    - Serial.flush() after every row for reliable capture
 *    - Timestamp upgraded to microseconds (micros())
 *    - Output is ready for direct Python logging (Step 08)
 *
 *  CSV Format:
 *    Time_us,
 *    Quaternion_i, Quaternion_j, Quaternion_k, Quaternion_real,
 *    GyroX, GyroY, GyroZ,
 *    LinearAccelX, LinearAccelY, LinearAccelZ
 *
 *  Python parsing contract:
 *    - Lines starting with '#' are comments — skip them
 *    - First non-comment line is the CSV header
 *    - All subsequent non-comment lines are data rows
 *    - Every data row has exactly 11 comma-separated fields
 *    - No trailing comma on any row
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
 *  Step:    07
 * =========================================================
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ── I2C Pin Configuration ────────────────────────────────
static const int      PIN_SDA                 = 22;
static const int      PIN_SCL                 = 21;
static const uint32_t I2C_CLOCK_HZ            = 400000;

// ── BNO085 I2C Address ───────────────────────────────────
static const uint8_t  IMU_I2C_ADDRESS         = BNO08x_DEFAULT_ADDRESS;  // 0x4B

// ── Sensor Report Intervals ──────────────────────────────
static const uint32_t RATE_ROTATION_VECTOR_US = 10000;  // 100 Hz
static const uint32_t RATE_GYROSCOPE_US       = 10000;  // 100 Hz
static const uint32_t RATE_LINEAR_ACCEL_US    = 10000;  // 100 Hz

// ── Diagnostic Interval ──────────────────────────────────
static const uint32_t DIAGNOSTIC_INTERVAL_MS  = 5000;   // Every 5 seconds
                                                         // Less frequent than
                                                         // Step 06 to keep
                                                         // CSV stream clean

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── Data Structs ─────────────────────────────────────────
struct RotationVectorData {
    float   i, j, k, real;
    uint8_t accuracy;
    bool    fresh;
} rv = {};

struct GyroData {
    float   x, y, z;   // rad/s
    uint8_t status;
    bool    fresh;
} gyro = {};

struct LinearAccelData {
    float   x, y, z;   // m/s²
    uint8_t status;
    bool    fresh;
} accel = {};

// ── Acquisition Statistics ───────────────────────────────
static uint32_t syncedSamples    = 0;
static uint32_t totalSamples     = 0;
static uint32_t lastDiagnostic_ms = 0;

// ── Forward Declarations ─────────────────────────────────
void printBanner(void);
bool initI2C(void);
bool initIMU(void);
bool enableSensorReports(void);
void handleIMUReset(void);
void processIMUEvent(void);
void syncAcquisition(void);
void emitCSVRow(void);
void emitCSVHeader(void);
void printDiagnostic(void);
const char* resetCauseString(uint8_t cause);
void printFirmwareSummary(void);

// ─────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    // All startup prints are prefixed '#' so Python skips them
    printBanner();

    if (!initI2C())             { while (1); }
    if (!initIMU())             { while (1); }

    printFirmwareSummary();

    if (!enableSensorReports()) { while (1); }

    lastDiagnostic_ms = millis();

    Serial.println(F("# "));
    Serial.println(F("# Synchronized acquisition ready."));
    Serial.println(F("# Starting CSV stream..."));
    Serial.println(F("# "));

    // Emit CSV header — Python reads this to name columns
    emitCSVHeader();
}

// ─────────────────────────────────────────────────────────
void loop()
{
    // ── Reset Recovery ───────────────────────────────────
    if (imu.wasReset())
    {
        handleIMUReset();
    }

    // ── Drain event queue ────────────────────────────────
    while (imu.getSensorEvent())
    {
        processIMUEvent();
    }

    // ── Synchronized CSV output ──────────────────────────
    syncAcquisition();

    // ── 5-second diagnostic (comment line) ───────────────
    uint32_t now_ms = millis();
    if ((now_ms - lastDiagnostic_ms) >= DIAGNOSTIC_INTERVAL_MS)
    {
        printDiagnostic();
        lastDiagnostic_ms = now_ms;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * emitCSVHeader()
 *
 * Prints the CSV header row exactly once at startup.
 * Column names match the final project specification.
 * Python reads this line to create DataFrame column names.
 *
 * No '#' prefix — this is a real CSV line, not a comment.
 */
void emitCSVHeader(void)
{
    Serial.println(F("Time_us,"
                     "Quaternion_i,Quaternion_j,Quaternion_k,Quaternion_real,"
                     "GyroX,GyroY,GyroZ,"
                     "LinearAccelX,LinearAccelY,LinearAccelZ"));

    Serial.flush();  // Flush header only — ensures Python sees it before data
}

// ─────────────────────────────────────────────────────────
/*
 * processIMUEvent()
 *
 * Unchanged from Step 06. Populates structs from
 * imu.sensorValue and sets fresh flags.
 */
void processIMUEvent(void)
{
    switch (imu.sensorValue.sensorId)
    {
        case SH2_ROTATION_VECTOR:
            rv.i        = imu.sensorValue.un.rotationVector.i;
            rv.j        = imu.sensorValue.un.rotationVector.j;
            rv.k        = imu.sensorValue.un.rotationVector.k;
            rv.real     = imu.sensorValue.un.rotationVector.real;
            rv.accuracy = imu.sensorValue.status & 0x03;
            rv.fresh    = true;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            gyro.x      = imu.sensorValue.un.gyroscope.x;
            gyro.y      = imu.sensorValue.un.gyroscope.y;
            gyro.z      = imu.sensorValue.un.gyroscope.z;
            gyro.status = imu.sensorValue.status & 0x03;
            gyro.fresh  = true;
            break;

        case SH2_LINEAR_ACCELERATION:
            accel.x      = imu.sensorValue.un.linearAcceleration.x;
            accel.y      = imu.sensorValue.un.linearAcceleration.y;
            accel.z      = imu.sensorValue.un.linearAcceleration.z;
            accel.status = imu.sensorValue.status & 0x03;
            accel.fresh  = true;
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * syncAcquisition()
 *
 * Unchanged from Step 06. Emits one CSV row only when all
 * three structs are fresh, then clears all fresh flags.
 */
void syncAcquisition(void)
{
    if (rv.fresh && gyro.fresh && accel.fresh)
    {
        emitCSVRow();

        rv.fresh    = false;
        gyro.fresh  = false;
        accel.fresh = false;

        syncedSamples++;
        totalSamples++;
    }
}

// ─────────────────────────────────────────────────────────

/*
 * emitCSVRow()
 *
 * Pre-formats the entire CSV row into a single char buffer
 * and transmits in one Serial.print() call.
 *
 * 4 decimal places chosen deliberately:
 *   Quaternion : 0.0001 resolution = 0.006 deg — sufficient
 *   Gyro       : 0.0001 rad/s      — sufficient
 *   Accel      : 0.0001 m/s²       — sufficient
 *
 * Bytes per row at 4dp:
 *   ~12 (timestamp) + 10 fields * ~9 chars + commas ~ 110 bytes
 *   At 115200 baud: ~1050 rows/sec theoretical max
 *   At 81 Hz actual: 81 * 110 = ~8910 bytes/sec — well within budget
 *
 * Note: snprintf on ESP32 requires (double) cast for float
 * arguments with %f format specifier.
 */
void emitCSVRow(void)
{
    char buf[160];

    snprintf(buf, sizeof(buf),
             "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             micros(),
             (double)rv.i,    (double)rv.j,
             (double)rv.k,    (double)rv.real,
             (double)gyro.x,  (double)gyro.y,  (double)gyro.z,
             (double)accel.x, (double)accel.y, (double)accel.z);

    Serial.print(buf);
}

// ─────────────────────────────────────────────────────────
/*
 * printDiagnostic()
 *
 * Printed as a CSV comment line (# prefix).
 * Python skips it. Visible in Serial Monitor for monitoring.
 * Reduced to every 5 seconds to minimize CSV interruption.
 */
void printDiagnostic(void)
{
    Serial.print(F("# [DIAG] Sync rate: "));
    Serial.print(syncedSamples / (DIAGNOSTIC_INTERVAL_MS / 1000));
    Serial.print(F(" rows/sec | Total rows: "));
    Serial.println(totalSamples);

    syncedSamples = 0;
}

// ─────────────────────────────────────────────────────────
void handleIMUReset(void)
{
    Serial.println(F("# [IMU] *** Spontaneous reset detected! ***"));

    rv    = {};
    gyro  = {};
    accel = {};

    if (!enableSensorReports())
    {
        Serial.println(F("# [IMU] CRITICAL: Could not recover. Halted."));
        while (1);
    }

    syncedSamples = 0;
    Serial.println(F("# [IMU] Recovery complete."));
}

// ─────────────────────────────────────────────────────────
bool initI2C(void)
{
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    Serial.print(F("# [I2C] SDA: GPIO ")); Serial.println(PIN_SDA);
    Serial.print(F("# [I2C] SCL: GPIO ")); Serial.println(PIN_SCL);
    Serial.print(F("# [I2C] Clock: "));
    Serial.print(I2C_CLOCK_HZ / 1000);
    Serial.println(F(" kHz"));
    return true;
}

// ─────────────────────────────────────────────────────────
bool initIMU(void)
{
    Serial.println(F("# [IMU] Opening SH-2 session..."));

    if (!imu.begin(IMU_I2C_ADDRESS, Wire))
    {
        Serial.println(F("# [IMU] ERROR: begin() failed."));
        return false;
    }

    Serial.println(F("# [IMU] SH-2 session opened successfully."));
    return true;
}

// ─────────────────────────────────────────────────────────
bool enableSensorReports(void)
{
    bool ok = true;

    if (!imu.enableReport(SH2_ROTATION_VECTOR, RATE_ROTATION_VECTOR_US))
    { Serial.println(F("# [IMU] Rotation Vector   -> FAILED")); ok = false; }

    if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, RATE_GYROSCOPE_US))
    { Serial.println(F("# [IMU] Gyroscope Cal     -> FAILED")); ok = false; }

    if (!imu.enableReport(SH2_LINEAR_ACCELERATION, RATE_LINEAR_ACCEL_US))
    { Serial.println(F("# [IMU] Linear Accel      -> FAILED")); ok = false; }

    if (ok) Serial.println(F("# [IMU] All reports enabled successfully."));

    return ok;
}

// ─────────────────────────────────────────────────────────
void printFirmwareSummary(void)
{
    Serial.println(F("# [IMU] SH-2 Firmware:"));

    for (uint8_t i = 0; i < imu.prodIds.numEntries; i++)
    {
        Serial.print(F("# [IMU]  "));
        Serial.print(i);
        Serial.print(F("  Part "));
        Serial.print(imu.prodIds.entry[i].swPartNumber);
        Serial.print(F("  v"));
        Serial.print(imu.prodIds.entry[i].swVersionMajor);
        Serial.print(F("."));
        Serial.print(imu.prodIds.entry[i].swVersionMinor);
        Serial.print(F("."));
        Serial.print(imu.prodIds.entry[i].swVersionPatch);
        Serial.print(F("  Build "));
        Serial.println(imu.prodIds.entry[i].swBuildNumber);
    }

    Serial.print(F("# [IMU] Reset cause: "));
    Serial.println(resetCauseString(imu.getResetReason()));
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
    Serial.println(F("# "));
    Serial.println(F("# ================================="));
    Serial.println(F("# Vehicle Dynamics IMU Project"));
    Serial.println(F("# Step 07 - CSV Logger"));
    Serial.println(F("# ================================="));
    Serial.println(F("# "));
}