/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 01 — BNO085 Detection & SH-2 Firmware Verification
 * =========================================================
 *
 *  Purpose:
 *    Initialize the BNO085 through the SparkFun library and
 *    verify the full SH-2 communication stack is operational
 *    by reading the sensor's product ID / firmware metadata.
 *
 *  What this proves:
 *    - I2C physical layer works         (Step 00)
 *    - SparkFun library can open SH-2   (this step)
 *    - SH-2 firmware responds correctly (this step)
 *    - enableReport() API works         (this step)
 *
 *  Hardware:
 *    ESP32 Dev Module
 *    SparkFun BNO085 breakout (SA0 pulled high -> address 0x4B)
 *
 *  Wiring (physical — SDA/SCL are intentionally swapped vs
 *  ESP32 defaults to match the installed hardware):
 *    BNO085 SDA -> ESP32 GPIO 22
 *    BNO085 SCL -> ESP32 GPIO 21
 *    BNO085 PS0 -> GND  (I2C mode)
 *    BNO085 PS1 -> GND  (I2C mode)
 *    BNO085 VCC -> 3.3V
 *    BNO085 GND -> GND
 *
 *  Library:
 *    SparkFun BNO08x Cortex Based IMU v1.0.6
 *
 *  Author:  Vehicle Dynamics IMU Project
 *  Step:    01
 * =========================================================
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ── I2C Pin Configuration ────────────────────────────────
// NOTE: Physical SDA/SCL are swapped vs ESP32 standard.
// GPIO 22 carries SDA. GPIO 21 carries SCL.
// This matches the installed hardware — do not change without
// rewiring the bench setup.
static const int PIN_SDA = 22;
static const int PIN_SCL = 21;

// ── BNO085 I2C Address ───────────────────────────────────
// SparkFun breakout: SA0 pulled HIGH -> 0x4B (default)
// If SA0 is pulled LOW the address is 0x4A.
static const uint8_t IMU_I2C_ADDRESS = BNO08x_DEFAULT_ADDRESS;  // 0x4B

// ── Report Interval ──────────────────────────────────────
// 10,000 us = 10 ms = 100 Hz
// Used here only to verify enableReport() accepts the command.
// Rate will be tuned in later steps.
static const uint32_t REPORT_INTERVAL_US = 10000;

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── Forward Declarations ─────────────────────────────────
void printBanner(void);
bool initI2C(void);
bool initIMU(void);
void printProductIDs(void);
void enableSensorReports(void);

// ─────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    printBanner();

    if (!initI2C())   { while (1); }
    if (!initIMU())   { while (1); }

    printProductIDs();
    enableSensorReports();

    Serial.println();
    Serial.println("Ready.");
    Serial.println("=================================");
}

// ─────────────────────────────────────────────────────────
void loop()
{
    // Step 01 does not read sensor data.
    // That begins in Step 03 (Enable Sensor Reports) and
    // Step 05 (Read Individual Reports).
}

// ─────────────────────────────────────────────────────────
/*
 * initI2C()
 *
 * Initialises the Wire bus with the correct physical pins.
 * Returns false if setup cannot proceed.
 */
bool initI2C(void)
{
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);  // 400 kHz Fast Mode — within BNO085 spec

    Serial.println("[I2C]  Bus initialised.");
    Serial.print  ("[I2C]  SDA: GPIO "); Serial.println(PIN_SDA);
    Serial.print  ("[I2C]  SCL: GPIO "); Serial.println(PIN_SCL);
    Serial.println("[I2C]  Clock: 400 kHz");
    Serial.println();

    return true;
}

// ─────────────────────────────────────────────────────────
/*
 * initIMU()
 *
 * Opens the SH-2 session through the SparkFun library.
 *
 * imu.begin() does the following internally:
 *   1. Sends a soft-reset packet to the BNO085
 *   2. Waits for the SH-2 hub to boot (~300 ms)
 *   3. Calls sh2_getProdIds() to confirm the hub is alive
 *   4. Registers the sensor event callback
 *
 * If begin() returns false, either the I2C address is wrong,
 * the wiring is bad, or the SH-2 firmware failed to boot.
 *
 * Returns true on success, false on failure.
 */
bool initIMU(void)
{
    Serial.println("[IMU]  Initializing BNO085...");

    if (!imu.begin(IMU_I2C_ADDRESS, Wire))
    {
        Serial.println("[IMU]  ERROR: begin() failed.");
        Serial.println("[IMU]  Check wiring, I2C address, and 3.3V supply.");
        return false;
    }

    Serial.println("[IMU]  BNO085 detected and SH-2 session opened.");
    return true;
}

// ─────────────────────────────────────────────────────────
/*
 * printProductIDs()
 *
 * Reads the SH-2 product ID records populated by begin().
 * These contain the firmware version, part number, and
 * reset reason — useful for confirming the exact firmware
 * revision running on the sensor.
 *
 * The reset cause codes are defined in the SH-2 reference:
 *   1 = Power-On Reset (normal first boot)
 *   2 = Internal reset
 *   3 = Watchdog
 *   4 = External reset
 *   5 = Other
 */
void printProductIDs(void)
{
    Serial.println("[IMU]  SH-2 Product IDs:");
    Serial.println("[IMU]  ------------------");

    for (uint8_t i = 0; i < imu.prodIds.numEntries; i++)
    {
        Serial.print  ("[IMU]  Entry ");
        Serial.print  (i);
        Serial.print  (": Part ");
        Serial.print  (imu.prodIds.entry[i].swPartNumber);
        Serial.print  ("  v");
        Serial.print  (imu.prodIds.entry[i].swVersionMajor);
        Serial.print  (".");
        Serial.print  (imu.prodIds.entry[i].swVersionMinor);
        Serial.print  (".");
        Serial.print  (imu.prodIds.entry[i].swVersionPatch);
        Serial.print  ("  Build ");
        Serial.println(imu.prodIds.entry[i].swBuildNumber);
    }

    uint8_t resetCause = imu.getResetReason();
    Serial.print("[IMU]  Reset cause: ");
    switch (resetCause)
    {
        case 1:  Serial.println("Power-On Reset (normal)"); break;
        case 2:  Serial.println("Internal reset");          break;
        case 3:  Serial.println("Watchdog");                break;
        case 4:  Serial.println("External reset");          break;
        case 5:  Serial.println("Other");                   break;
        default: Serial.print  ("Unknown ("); 
                 Serial.print  (resetCause); 
                 Serial.println(")");                       break;
    }

    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * enableSensorReports()
 *
 * Requests the three sensor reports needed for vehicle
 * dynamics analysis. Uses the v1.0.6 enableReport() API
 * directly with SH-2 sensor IDs.
 *
 * API note — why enableReport() and not the named helpers:
 *   The named helpers (enableGameRotationVector, etc.) are
 *   thin wrappers that convert milliseconds to microseconds
 *   then call enableReport() anyway. Using enableReport()
 *   directly is clearer, avoids unit confusion, and gives
 *   us exact microsecond control over report rates in later
 *   steps when we tune for consistent sampling.
 *
 * Sensors enabled:
 *   SH2_ROTATION_VECTOR      - Fused quaternion (uses magnetometer)
 *   SH2_GYROSCOPE_CALIBRATED - Angular rate (rad/s)
 *   SH2_LINEAR_ACCELERATION  - Accel with gravity removed (m/s^2)
 */
void enableSensorReports(void)
{
    Serial.println("[IMU]  Enabling sensor reports...");

    // Rotation Vector: absolute orientation quaternion
    if (imu.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US))
        Serial.println("[IMU]  Rotation Vector        -> OK  (100 Hz)");
    else
        Serial.println("[IMU]  Rotation Vector        -> FAILED");

    // Calibrated Gyroscope: angular rate in rad/s
    if (imu.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US))
        Serial.println("[IMU]  Gyroscope (calibrated) -> OK  (100 Hz)");
    else
        Serial.println("[IMU]  Gyroscope (calibrated) -> FAILED");

    // Linear Acceleration: acceleration with gravity vector removed
    if (imu.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US))
        Serial.println("[IMU]  Linear Acceleration    -> OK  (100 Hz)");
    else
        Serial.println("[IMU]  Linear Acceleration    -> FAILED");
}

// ─────────────────────────────────────────────────────────
void printBanner(void)
{
    Serial.println();
    Serial.println("=================================");
    Serial.println(" Vehicle Dynamics IMU Project");
    Serial.println(" Step 01 - BNO085 Detection");
    Serial.println("=================================");
    Serial.println();
}
