/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 06 — Synchronized Sensor Acquisition
 * =========================================================
 *
 *  Purpose:
 *    Emit one synchronized output row per acquisition cycle
 *    only when all three sensor report structs have been
 *    freshly updated. Adds millisecond timestamp to each row.
 *
 *  New in this step:
 *    - syncAcquisition() gates output on all 3 fresh flags
 *    - millis() timestamp on every output row
 *    - 1Hz diagnostic: actual synchronized sample rate
 *    - Output format is the direct precursor to Step 07 CSV
 *    - Human-readable display replaced with clean data lines
 *
 *  Synchronization strategy:
 *    The SH-2 delivers the three reports as independent
 *    events. After draining the event queue each loop(),
 *    we check whether all three structs are fresh. When
 *    all three are fresh, we emit one row and clear the
 *    fresh flags. This ensures every output row contains
 *    data from the same acquisition window with no stale
 *    values carried from a previous cycle.
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
 *  Step:    06
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

// ── Diagnostic Print Interval ────────────────────────────
static const uint32_t DIAGNOSTIC_INTERVAL_MS  = 1000;   // 1 Hz

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
static uint32_t syncedSamples   = 0;   // rows emitted this diagnostic window
static uint32_t totalSamples    = 0;   // total rows emitted since boot
static uint32_t lastDiagnostic_ms = 0;

// ── Forward Declarations ─────────────────────────────────
void        printBanner(void);
bool        initI2C(void);
bool        initIMU(void);
bool        enableSensorReports(void);
void        handleIMUReset(void);
void        processIMUEvent(void);
void        syncAcquisition(void);
void        emitSynchronizedRow(void);
void        printDiagnostic(void);
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

    lastDiagnostic_ms = millis();

    Serial.println();
    Serial.println("Synchronized acquisition running...");
    Serial.println("Format: Time | RV(i,j,k,real) | Gyro(x,y,z) | Accel(x,y,z)");
    Serial.println("=================================");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * loop()
 *
 * Three responsibilities:
 *   1. Drain all SH-2 events and populate structs
 *   2. Check synchronization and emit row if all fresh
 *   3. Print 1Hz diagnostic showing actual sample rate
 */
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

    // ── Synchronized output ──────────────────────────────
    syncAcquisition();

    // ── 1Hz diagnostic ───────────────────────────────────
    uint32_t now_ms = millis();
    if ((now_ms - lastDiagnostic_ms) >= DIAGNOSTIC_INTERVAL_MS)
    {
        printDiagnostic();
        lastDiagnostic_ms = now_ms;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * processIMUEvent()
 *
 * Identical to Step 05. Populates the three data structs
 * and sets fresh=true for each report as it arrives.
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
 * The core synchronization gate.
 *
 * Emits one output row only when ALL THREE structs are fresh.
 * This guarantees:
 *   - No row contains a value from a previous cycle
 *   - All three values in a row come from the same
 *     acquisition window (within one loop() period, ~1–2 ms)
 *   - The output rate naturally tracks the slowest report
 *     (Rotation Vector at ~80–92 Hz)
 *
 * After emitting, all fresh flags are cleared so the next
 * row cannot be emitted until all three update again.
 */
void syncAcquisition(void)
{
    if (rv.fresh && gyro.fresh && accel.fresh)
    {
        emitSynchronizedRow();

        rv.fresh    = false;
        gyro.fresh  = false;
        accel.fresh = false;

        syncedSamples++;
        totalSamples++;
    }
}

// ─────────────────────────────────────────────────────────
/*
 * emitSynchronizedRow()
 *
 * Prints one tab-separated data row to Serial.
 * Format is the direct precursor to Step 07 CSV output —
 * only the separator character changes (tab -> comma).
 *
 * Columns:
 *   TimeMs    : millis() at emit time (ms since boot)
 *   RV_i/j/k/real : rotation vector quaternion components
 *   Gyro_X/Y/Z    : calibrated angular rate (rad/s)
 *   Accel_X/Y/Z   : linear acceleration (m/s²)
 */
void emitSynchronizedRow(void)
{
    // Header is printed once in setup(). Data rows follow.
    Serial.print(millis());         Serial.print("\t");

    Serial.print(rv.i,    5);       Serial.print("\t");
    Serial.print(rv.j,    5);       Serial.print("\t");
    Serial.print(rv.k,    5);       Serial.print("\t");
    Serial.print(rv.real, 5);       Serial.print("\t");

    Serial.print(gyro.x,  5);       Serial.print("\t");
    Serial.print(gyro.y,  5);       Serial.print("\t");
    Serial.print(gyro.z,  5);       Serial.print("\t");

    Serial.print(accel.x, 5);       Serial.print("\t");
    Serial.print(accel.y, 5);       Serial.print("\t");
    Serial.println(accel.z, 5);
}

// ─────────────────────────────────────────────────────────
/*
 * printDiagnostic()
 *
 * Prints actual synchronized sample rate once per second.
 * This confirms the sync gate is working correctly and
 * tells us the effective output rate before CSV logging.
 *
 * Expected: 80–95 synced samples/sec (limited by RV rate).
 * If this reads 0: all three fresh flags never align —
 * check that all three enableReport() calls succeeded.
 */
void printDiagnostic()
{
    Serial.println();
    Serial.print(">>> Sync rate: ");
    Serial.print(syncedSamples);
    Serial.print(" rows/sec  |  Total rows: ");
    Serial.println(totalSamples);
    Serial.println();

    syncedSamples = 0;
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

    syncedSamples = 0;
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
    Serial.println(" Step 06 - Synchronized Acquisition");
    Serial.println("=================================");
    Serial.println();
}