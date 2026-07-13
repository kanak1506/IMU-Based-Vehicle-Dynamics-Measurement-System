/*
 * =========================================================
 *  Vehicle Dynamics IMU Project
 *  Step 02 — SH-2 Initialization & Reset Recovery
 * =========================================================
 *
 *  Purpose:
 *    Establish the production-quality initialization
 *    architecture that all future steps will build on.
 *    Add spontaneous-reset detection and recovery in loop().
 *
 *  New in this step:
 *    - wasReset() check in loop() with automatic report
 *      re-enable (production robustness requirement)
 *    - Clean firmware version summary table at startup
 *    - All tunable constants named and grouped
 *    - Architecture ready for sensor data reading (Step 03+)
 *
 *  Hardware:
 *    ESP32 Dev Module
 *    SparkFun BNO085 breakout (SA0 pulled HIGH -> 0x4B)
 *
 *  Wiring (SDA/SCL intentionally swapped vs ESP32 standard
 *  to match installed hardware):
 *    BNO085 SDA -> ESP32 GPIO 22
 *    BNO085 SCL -> ESP32 GPIO 21
 *    BNO085 PS0 -> GND   (selects I2C mode)
 *    BNO085 PS1 -> GND   (selects I2C mode)
 *    BNO085 VCC -> 3.3V
 *    BNO085 GND -> GND
 *
 *  Library:
 *    SparkFun BNO08x Cortex Based IMU v1.0.6
 *
 *  Author:  Vehicle Dynamics IMU Project
 *  Step:    02
 * =========================================================
 */

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ── I2C Pin Configuration ────────────────────────────────
// Physical SDA/SCL are swapped vs ESP32 standard (GPIO21/22).
// GPIO 22 carries SDA. GPIO 21 carries SCL.
// This matches installed hardware. Do not change without
// rewiring.
static const int      PIN_SDA          = 22;
static const int      PIN_SCL          = 21;
static const uint32_t I2C_CLOCK_HZ     = 400000;   // 400 kHz Fast Mode

// ── BNO085 I2C Address ───────────────────────────────────
// SA0 pulled HIGH on SparkFun breakout -> 0x4B
static const uint8_t  IMU_I2C_ADDRESS  = BNO08x_DEFAULT_ADDRESS;  // 0x4B

// ── Sensor Report Intervals ──────────────────────────────
// All reports run at 100 Hz (10,000 us) for this step.
// Individual rates will be tuned in Step 06 (Synchronized
// Acquisition) based on sensor capabilities.
static const uint32_t RATE_ROTATION_VECTOR_US    = 10000;  // 100 Hz
static const uint32_t RATE_GYROSCOPE_US          = 10000;  // 100 Hz
static const uint32_t RATE_LINEAR_ACCEL_US       = 10000;  // 100 Hz

// ── IMU Object ───────────────────────────────────────────
BNO08x imu;

// ── Forward Declarations ─────────────────────────────────
void printBanner(void);
bool initI2C(void);
bool initIMU(void);
bool enableSensorReports(void);
void printFirmwareSummary(void);
void handleIMUReset(void);
const char* resetCauseString(uint8_t cause);

// ─────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    printBanner();

    if (!initI2C())            { while (1); }
    if (!initIMU())            { while (1); }

    printFirmwareSummary();

    if (!enableSensorReports()) { while (1); }

    Serial.println();
    Serial.println("Initialization complete. Monitoring...");
    Serial.println("=================================");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * loop()
 *
 * The only responsibility of loop() at this stage is to
 * detect spontaneous BNO085 resets and recover.
 *
 * Why this matters in automotive use:
 *   The BNO085 can reset due to:
 *     - Power supply glitch (vehicle crank, load dump)
 *     - Watchdog timeout inside SH-2 firmware
 *     - I2C bus errors causing SH-2 to restart
 *
 *   Without recovery, the system silently stops producing
 *   data. With wasReset(), we detect it within one loop()
 *   cycle and re-enable all reports automatically.
 *
 *   imu.wasReset() returns true once after a reset event,
 *   then clears its internal flag — so it fires only once
 *   per event.
 *
 * Sensor reading will be added in Step 05.
 */
void loop()
{
    if (imu.wasReset())
    {
        handleIMUReset();
    }

    // sh2_service() is called internally by getSensorEvent().
    // At this step we are not yet reading events, but the
    // wasReset() check above is sufficient to keep the
    // session alive.
    //
    // Step 05 will replace this comment with getSensorEvent()
    // calls and data extraction.
}

// ─────────────────────────────────────────────────────────
/*
 * initI2C()
 *
 * Initialises the Wire bus with the physical pin assignment
 * and sets clock to 400 kHz Fast Mode.
 */
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
/*
 * initIMU()
 *
 * Opens the SH-2 session via the SparkFun library.
 *
 * imu.begin() performs internally:
 *   1. Soft-reset the BNO085 via SHTP channel 1
 *   2. Wait for SH-2 hub boot (~300 ms typical)
 *   3. sh2_getProdIds() — verifies hub is alive and
 *      populates imu.prodIds
 *   4. Register sensorHandler callback for event delivery
 *
 * Failure modes:
 *   - Returns false if no I2C ACK at the given address
 *   - Returns false if sh2_open() times out (SH-2 did not
 *     boot — usually a power or wiring problem)
 */
bool initIMU(void)
{
    Serial.println("[IMU]  Opening SH-2 session...");

    if (!imu.begin(IMU_I2C_ADDRESS, Wire))
    {
        Serial.println("[IMU]  ERROR: begin() failed.");
        Serial.println("[IMU]  Possible causes:");
        Serial.println("[IMU]    - Wrong I2C address (check SA0 pin)");
        Serial.println("[IMU]    - SDA/SCL wiring fault");
        Serial.println("[IMU]    - BNO085 not receiving 3.3V");
        Serial.println("[IMU]    - PS0/PS1 not grounded (SPI mode active)");
        return false;
    }

    Serial.println("[IMU]  SH-2 session opened successfully.");
    return true;
}

// ─────────────────────────────────────────────────────────
/*
 * enableSensorReports()
 *
 * Requests all three sensor reports from the SH-2 hub.
 * Called once at startup and again after any spontaneous
 * reset, because reports do not survive a sensor reset.
 *
 * Uses imu.enableReport() directly with SH-2 sensor IDs
 * and microsecond intervals — the canonical v1.0.6 API.
 *
 * Returns false if any report fails to enable.
 */
bool enableSensorReports(void)
{
    Serial.println("[IMU]  Enabling sensor reports...");

    bool ok = true;

    if (imu.enableReport(SH2_ROTATION_VECTOR, RATE_ROTATION_VECTOR_US))
    {
        Serial.print("[IMU]  Rotation Vector        -> OK  @ ");
        Serial.print(1000000UL / RATE_ROTATION_VECTOR_US);
        Serial.println(" Hz");
    }
    else
    {
        Serial.println("[IMU]  Rotation Vector        -> FAILED");
        ok = false;
    }

    if (imu.enableReport(SH2_GYROSCOPE_CALIBRATED, RATE_GYROSCOPE_US))
    {
        Serial.print("[IMU]  Gyroscope (calibrated) -> OK  @ ");
        Serial.print(1000000UL / RATE_GYROSCOPE_US);
        Serial.println(" Hz");
    }
    else
    {
        Serial.println("[IMU]  Gyroscope (calibrated) -> FAILED");
        ok = false;
    }

    if (imu.enableReport(SH2_LINEAR_ACCELERATION, RATE_LINEAR_ACCEL_US))
    {
        Serial.print("[IMU]  Linear Acceleration    -> OK  @ ");
        Serial.print(1000000UL / RATE_LINEAR_ACCEL_US);
        Serial.println(" Hz");
    }
    else
    {
        Serial.println("[IMU]  Linear Acceleration    -> FAILED");
        ok = false;
    }

    return ok;
}

// ─────────────────────────────────────────────────────────
/*
 * handleIMUReset()
 *
 * Called when imu.wasReset() returns true.
 * Logs the event and re-enables all sensor reports.
 *
 * In a production system this would also:
 *   - Increment a fault counter
 *   - Write a fault record to SD card or CAN bus
 *   - Assert a warning output
 *
 * For now we log and recover cleanly.
 */
void handleIMUReset(void)
{
    Serial.println();
    Serial.println("[IMU]  *** Spontaneous reset detected! ***");
    Serial.println("[IMU]  Re-enabling sensor reports...");

    if (!enableSensorReports())
    {
        Serial.println("[IMU]  CRITICAL: Could not re-enable reports after reset.");
        Serial.println("[IMU]  System halted.");
        while (1);
    }

    Serial.println("[IMU]  Recovery complete.");
    Serial.println();
}

// ─────────────────────────────────────────────────────────
/*
 * printFirmwareSummary()
 *
 * Prints a formatted table of all SH-2 product ID entries.
 * Entry 0 is the SH-2 hub firmware — the most important one.
 * Additional entries are motion engine sub-components.
 */
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
/*
 * resetCauseString()
 *
 * Returns a human-readable string for the SH-2 reset cause
 * code. Defined in SH-2 reference manual section 6.4.
 */
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
    Serial.println(" Step 02 - SH-2 Initialization");
    Serial.println("=================================");
    Serial.println();
}