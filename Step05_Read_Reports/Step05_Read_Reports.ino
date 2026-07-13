/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 05 — Read Individual Sensor Reports
 * =========================================================
 *
 *  Purpose:
 *    Extract and display actual sensor data values from the
 *    three active SH-2 report streams for the first time.
 *    Verify numbers are physically plausible with sensor
 *    stationary on a bench.
 *
 *  New in this step:
 *    - Data fields extracted from imu.sensorValue
 *    - RotationVectorData, GyroData, LinearAccelData structs
 *      hold latest reading per stream (pattern for Step 06)
 *    - Quaternion magnitude computed as self-check
 *    - Accuracy/status byte decoded per report
 *    - Display throttled to 10 Hz (structs always current)
 *
 *  Bench sanity check (sensor stationary, flat):
 *    Rotation Vector : quaternion magnitude |q| ~ 1.0000
 *    Gyroscope       : all axes near 0.0 rad/s
 *    Linear Accel    : all axes near 0.0 m/s² (gravity removed)
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
 *  Step:    05
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

// ── Display Rate ─────────────────────────────────────────
// Sensor acquires at 100 Hz. We display at 10 Hz.
// The structs always hold the most recent sample — nothing
// is lost between prints, we simply display the latest value.
static const uint32_t DISPLAY_INTERVAL_MS     = 100;    // 10 Hz

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── Data Structs ─────────────────────────────────────────
/*
 * One struct per report type.
 * Updated every time a new event arrives in processIMUEvent().
 * Read by printData() at 10 Hz display rate.
 *
 * This decouples acquisition rate from display rate and is
 * the exact pattern Step 06 uses for synchronized CSV output.
 *
 * 'fresh' flag: true when a new value has arrived since the
 * last display cycle. Step 06 uses this to gate CSV output
 * on all three reports being current in the same cycle.
 */
struct RotationVectorData {
    float   i, j, k, real;
    uint8_t accuracy;   // 0=Unreliable 1=Low 2=Medium 3=High
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

// ── Timing ───────────────────────────────────────────────
static uint32_t lastDisplay_ms = 0;

// ── Forward Declarations ─────────────────────────────────
void        printBanner(void);
bool        initI2C(void);
bool        initIMU(void);
bool        enableSensorReports(void);
void        handleIMUReset(void);
void        processIMUEvent(void);
void        printData(void);
const char* accuracyString(uint8_t accuracy);
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

    lastDisplay_ms = millis();

    Serial.println();
    Serial.println("Reading sensor data (display: 10 Hz)...");
    Serial.println("=================================");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * loop()
 *
 * Two responsibilities:
 *   1. Drain all pending SH-2 events and populate structs
 *   2. Print a formatted data display at 10 Hz
 *
 * The while() drain ensures no events are missed between
 * loop() cycles regardless of how long printData() takes.
 */
void loop()
{
    // ── Reset Recovery ───────────────────────────────────
    if (imu.wasReset())
    {
        handleIMUReset();
    }

    // ── Drain event queue, populate structs ──────────────
    while (imu.getSensorEvent())
    {
        processIMUEvent();
    }

    // ── 10 Hz display ────────────────────────────────────
    uint32_t now_ms = millis();
    if ((now_ms - lastDisplay_ms) >= DISPLAY_INTERVAL_MS)
    {
        printData();
        lastDisplay_ms = now_ms;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * processIMUEvent()
 *
 * Reads imu.sensorValue and writes into the appropriate
 * data struct. The sensorId field determines which union
 * member is valid — never read a union member for a
 * different sensorId than it was written for.
 *
 * Field sources (from sh2_SensorValue.h):
 *   SH2_ROTATION_VECTOR:
 *     un.rotationVector.i / .j / .k / .real  (unitless)
 *     status & 0x03  -> accuracy (0–3)
 *
 *   SH2_GYROSCOPE_CALIBRATED:
 *     un.gyroscope.x / .y / .z  (rad/s)
 *     status & 0x03  -> calibration status (0–3)
 *
 *   SH2_LINEAR_ACCELERATION:
 *     un.linearAcceleration.x / .y / .z  (m/s²)
 *     status & 0x03  -> calibration status (0–3)
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
 * printData()
 *
 * Prints all three sensor readings at 10 Hz.
 *
 * Quaternion magnitude is computed as a continuous health
 * check. A correctly functioning SH-2 fusion always produces
 * a unit quaternion: |q| = sqrt(i²+j²+k²+real²) = 1.0
 * Deviation beyond ±0.002 indicates a fusion problem.
 *
 * 'fresh' flag shown per report — if false at display time,
 * the struct holds a value from a previous cycle. Acceptable
 * for display; Step 06 uses it to gate CSV output.
 */
void printData(void)
{
    float mag = sqrt(rv.i    * rv.i +
                     rv.j    * rv.j +
                     rv.k    * rv.k +
                     rv.real * rv.real);

    // ── Rotation Vector ──────────────────────────────────
    Serial.println("--- Rotation Vector (Quaternion) ---");
    Serial.print("  i:        "); Serial.println(rv.i,    5);
    Serial.print("  j:        "); Serial.println(rv.j,    5);
    Serial.print("  k:        "); Serial.println(rv.k,    5);
    Serial.print("  real:     "); Serial.println(rv.real, 5);
    Serial.print("  |q|:      "); Serial.print(mag, 5);
    Serial.println(abs(mag - 1.0f) < 0.002f ? "  [OK]" : "  [WARN: not unit quaternion]");
    Serial.print("  Accuracy: "); Serial.println(accuracyString(rv.accuracy));
    Serial.print("  Fresh:    "); Serial.println(rv.fresh    ? "yes" : "no (stale)");
    rv.fresh = false;

    // ── Gyroscope ────────────────────────────────────────
    Serial.println("--- Gyroscope (rad/s) ---");
    Serial.print("  X:        "); Serial.println(gyro.x, 5);
    Serial.print("  Y:        "); Serial.println(gyro.y, 5);
    Serial.print("  Z:        "); Serial.println(gyro.z, 5);
    Serial.print("  Status:   "); Serial.println(accuracyString(gyro.status));
    Serial.print("  Fresh:    "); Serial.println(gyro.fresh  ? "yes" : "no (stale)");
    gyro.fresh = false;

    // ── Linear Acceleration ──────────────────────────────
    Serial.println("--- Linear Acceleration (m/s²) ---");
    Serial.print("  X:        "); Serial.println(accel.x, 5);
    Serial.print("  Y:        "); Serial.println(accel.y, 5);
    Serial.print("  Z:        "); Serial.println(accel.z, 5);
    Serial.print("  Status:   "); Serial.println(accuracyString(accel.status));
    Serial.print("  Fresh:    "); Serial.println(accel.fresh ? "yes" : "no (stale)");
    accel.fresh = false;

    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * accuracyString()
 *
 * Decodes the lower 2 bits of the SH-2 status byte.
 * Defined in SH-2 reference manual section 1.3.5.2.
 *   0 = Unreliable — do not use this data
 *   1 = Low        — use with caution
 *   2 = Medium     — acceptable for most purposes
 *   3 = High       — fully calibrated
 */
const char* accuracyString(uint8_t accuracy)
{
    switch (accuracy & 0x03)
    {
        case 0:  return "0 - Unreliable";
        case 1:  return "1 - Low";
        case 2:  return "2 - Medium";
        case 3:  return "3 - High";
        default: return "Unknown";
    }
}

// ─────────────────────────────────────────────────────────
void handleIMUReset(void)
{
    Serial.println("[IMU]  *** Spontaneous reset detected! ***");

    rv    = {};
    gyro  = {};
    accel = {};

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
    Serial.println(" Step 05 - Read Individual Reports");
    Serial.println("=================================");
    Serial.println();
}