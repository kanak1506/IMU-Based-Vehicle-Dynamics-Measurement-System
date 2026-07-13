# The Automotive Inertial Measurement Unit & Vehicle Dynamics Handbook
## A Rigorous Engineering Guide from First Principles

This handbook provides an in-depth, first-principles guide to the hardware, circuits, protocols, firmware, mathematical models, and vehicle dynamics of the Inertial Measurement Unit (IMU) logging and analysis project.

---

## Table of Contents
1. **Chapter 1: Overall Project Goal, Architecture & Analysis Pipeline**
2. **Chapter 2: Hardware Components & Operating Principles**
3. **Chapter 3: Circuit Design, Electrical Wiring & Electrical Signals**
4. **Chapter 4: The I2C Protocol Deep-Dive (Clock Stretching, Start/Stop, Timing)**
5. **Chapter 5: BNO085 Sensor Architecture & SH-2 Sensor Fusion**
6. **Chapter 6: ESP32 Firmware Walkthrough: Steps 01 to 07**
7. **Chapter 7: C++ Programming Concepts (Pointers, Structs, Unions, Callbacks)**
8. **Chapter 8: Data Flow Packet Tracing**
9. **Chapter 9: Sensor Reports & Vehicle Dynamics Interpretation**
10. **Chapter 10: Mathematical Derivations**
11. **Chapter 11: Debugging & Troubleshooting**
12. **Chapter 12: Learning Roadmap & References**

---

# Chapter 1: Overall Project Goal, Architecture & Analysis Pipeline

### 1. Theory: The Engineering Objective
Understanding vehicle body motion relative to the road surface is crucial for chassis tuning, electronic stability control, and vehicle dynamics validation. 

A vehicle moving on a three-dimensional road plane operates in six degrees of freedom:
*   **Translational Acceleration**: Longitudinal ($a_x$, accelerating/braking), Lateral ($a_y$, cornering), and Vertical ($a_z$, suspension deflection/bumps).
*   **Rotational Orientation**: Roll ($\phi$, chassis tilt about longitudinal axis), Pitch ($\theta$, dive/squat about lateral axis), and Yaw ($\psi$, heading angle about vertical axis).

```
         Roll (Rotation about X)
              ┌───┐    
         ◄───►│   │◄───►  (X-axis: Longitudinal / Forward)
              └─┬─┘
  Pitch (Rotation about Y)   Yaw (Rotation about Z)
         ▲      │                 ▲
         │      ▼                 │ ◄───►
         ▼  (Y-axis: Lateral)     ▼ (Z-axis: Vertical)
```

The objective of this project is to construct a microsecond-precision, synchronized IMU logging system that measures these six degrees of freedom using the BNO085 sensor and ESP32 microcontroller, transmits them via a robust CSV stream, and analyzes the data under the **ISO 4138** standard for steady-state circular vehicle handling.

---

### 2. Application in this Project
The project is implemented across two main environments: the **embedded firmware (ESP32)** and the **analysis pipeline (Python & Streamlit)**.

```
+---------------------------------------------------------------------------------+
|                                EMBEDDED SYSTEM                                  |
|  [Physical Motion] -> [MEMS Transducers] -> [EKF Fusion] -> [SHTP Packets]      |
|                                                                    |            |
|                                                                    v            |
|  [PC Logger] <- [UART Serial] <- [ESP32 I2C Gate] <------ [BNO085 Chip]         |
+---------------------------------------------------------------------------------+
                                       |
                                       v (CSV Log File)
+---------------------------------------------------------------------------------+
|                                 PYTHON PIPELINE                                 |
|  [Raw Data Plotter] -> [Chassis Coordinate Mapping] -> [Butterworth LPF]        |
|                                                                    |            |
|                                                                    v            |
|  [Report Dashboard] <------- [Linear Regression] <------- [Steady State Window] |
+---------------------------------------------------------------------------------+
```

#### The Logging Pipeline
1.  **Physical Sensor**: The BNO085 samples its accelerometers, gyroscopes, and magnetometers, performing sensor fusion at 400 Hz.
2.  **I2C Bus Transmission**: The ESP32 polls the BNO085 at 100 Hz. The firmware operates a synchronization gate (Step 06/07) ensuring that a data row is only emitted when the Rotation Vector, Gyroscope, and Linear Acceleration are all fresh.
3.  **USB Serial Interface**: Data is formatted as a comma-separated string and sent to the host PC over UART at 115200 baud.
4.  **Python Logger (`step08_serial_logger.py`)**: Runs in the background, filters out comments, reads the header, and writes the stream to a timestamped CSV file.

#### The Analysis Pipeline
1.  **Vibration Filtering (`step10_vehicle_dynamics.py`)**: Applies a 4th-order Butterworth low-pass filter (10 Hz cutoff) to remove engine/chassis vibration while preserving suspension and body roll motions.
2.  **ISO Window Extraction (`step11_iso4138_analysis.py`)**: Scans the filtered data with a sliding window, verifying stability criteria: lateral acceleration variation ($\Delta a_y \le 0.02g$), roll variation ($\Delta \phi \le 0.5^\circ$), and yaw rate variation ($\Delta \dot{\psi} \le 2.0^\circ/\text{s}$) over a minimum 2.0-second period.
3.  **Handling Characterization**: Computes the **Roll Gradient** (chassis roll angle per unit of lateral acceleration, expressed in $^\circ/g$) and the **Yaw Rate Gradient** ($(\dot{\psi})/g$) using ordinary least squares (OLS) linear regression.

---

### 3. Code Walkthrough
The software components are structured as follows:

*   **`Step01_BNO085_Initialization`**: Sets up basic I2C communication and reads internal firmware version metadata from the BNO085's ROM registers.
*   **`Step02_Read_SH2`**: Implements spontaneous-reset recovery. If the sensor brownouts or resets due to vehicle engine cranking, the firmware catches the event via `imu.wasReset()` and re-enables sensor configuration on the fly.
*   **`Step03_Enable_Reports` & `Step04_Verify_Report_IDs`**: Implement SHTP packet reading, verifying that the unique report IDs (`0x05`, `0x02`, `0x04`) arrive at the target rate before decoding variables.
*   **`Step05_Read_Reports`**: Extracts raw payload values (quaternion components, angular velocities, and linear accelerations) and decodes accuracy status flags.
*   **`Step06_Synchronized_Acquisition`**: Synchronizes the asynchronous output of the individual sensors, ensuring that data is only printed when a complete snapshot of all three sensors is ready.
*   **`Step07_CSV_Logger`**: Formats the output as a clean, comma-separated stream with a microsecond timestamp, marking all diagnostic messages with a `#` prefix.
*   **`step08_serial_logger.py`**: A python script that reads the serial port, ignores lines starting with `#`, and logs data rows to a CSV.
*   **`step10_vehicle_dynamics.py`**: Computes Euler angles and transforms accelerations to the vehicle body coordinate system, exporting them to standard "g" forces.
*   **`step11_iso4138_analysis.py`**: Extracts steady-state windows and calculates the roll gradient.
*   **`app.py` & `dashboard/`**: Streamlit application UI, presets, and runners that automate execution of the python scripts.

---

### 4. Common Mistakes
*   **Stale Sensor Carry-over**: Reading a sensor value at $100\,\text{Hz}$ even if the physical sensor has not updated since the last cycle. This leads to duplicate data rows and distorts frequency analysis. The project fixes this by using a boolean `fresh` flag for each struct.
*   **Comment Collision**: Sending debug print statements to Serial without a comment delimiter (like `#`). This causes python parsers to crash when trying to convert raw text strings to floats.

---

### 5. Key Takeaways
*   **Modular Progression**: Building step-by-step from initialization to synchronized logging ensures that hardware connection issues are isolated before writing data-processing code.
*   **Consistent Sampling**: Gating the logging loop based on the slowest reporting rate (the Rotation Vector) guarantees that all elements in a single logged row correspond to the same physical moment.

---

# Chapter 2: Hardware Components & Operating Principles

Understanding the physical sensors and microcontrollers is essential for explaining how they convert movement into digital data.

```
+───────────────────────────────────────────────────────────────────────────+
|                           BNO085 CHIP SCHEMATIC                           |
|                                                                           |
|  +─────────────────────────────────────────────────────────────────────+  |
|  |                       Cortex-M0+ Core (SH-2)                        |  |
|  |  +───────────────────────────────────────────────────────────────+  |  |
|  |  |                 Extended Kalman Filter (EKF)                  |  |  |
|  |  +──────────────────────────────▲────────────────────────────────+  |  |
|  |                                 │ fused signals                     |  |
|  +─────────────────────────────────┼───────────────────────────────────+  |
|                                    │                                      |
|            +───────────────────────┼───────────────────────+              |
|            │                       │                       │              |
|      +─────┴─────+           +─────┴─────+           +─────┴─────+        |
|      |   MEMS    |           |   MEMS    |           |   MEMS    |        |
|      |  Accel    |           |   Gyro    |           |  Magnet.  |        |
|      +───────────+           +───────────+           +───────────+        |
+───────────────────────────────────────────────────────────────────────────+
```

---

### 1. Theory: Hardware Components & Principles

#### A. ESP32 Microcontroller
The ESP32 is a dual-core 32-bit Tensilica Xtensa LX6 microcontroller operating at $240\,\text{MHz}$. It acts as the local computing hub, executing C++ code, operating the I2C master bus, and handling high-speed Serial-to-USB communications.

#### B. BNO085 Intelligent IMU
The BNO085 is a 9-axis System-in-Package (SiP) containing three distinct Micro-Electro-Mechanical Systems (MEMS) sensors and an ARM Cortex-M0+ processor running Hillcrest Labs' **SH-2** firmware.

##### MEMS Accelerometer
The accelerometer measures translational acceleration by detecting the displacement of a micro-machined silicon proof mass. The mass is suspended on silicon springs between fixed capacitive fingers. 
When the sensor accelerates, inertia causes the proof mass to deflect, changing the distance ($d$) between the capacitive fingers. Since capacitance is given by:

$$C = \epsilon_0 \epsilon_r \frac{A}{d}$$

The deflection causes a change in capacitance. An internal ASIC measures this change and converts it to a voltage, which is then digitized into acceleration ($m/s^2$).

##### MEMS Gyroscope
The gyroscope measures angular rate using the **Coriolis Effect**. A vibrating silicon frame is driven electrostatically to oscillate back and forth at a high frequency. When the sensor rotates, the Coriolis force:

$$\vec{F}_c = -2m (\vec{\omega} \times \vec{v})$$

acts on the vibrating mass, causing it to deflect in a perpendicular direction. This deflection is measured capacitively and converted to angular velocity ($\text{rad/s}$).

##### MEMS Magnetometer
The magnetometer measures heading relative to the Earth's magnetic field. It uses Lorentz force micro-structures. A small current is sent through a suspended silicon beam. When an external magnetic field is present, it exerts a Lorentz force:

$$\vec{F}_L = I(\vec{L} \times \vec{B})$$

This force displaces the beam. The displacement is measured capacitively to resolve the orientation and magnitude of the magnetic field ($\mu\text{T}$).

##### Sensor Fusion & The Kalman Filter
Raw MEMS data has typical error profiles:
*   **Gyroscopes** are clean in the short term but drift over time because integrating their measurements accumulates bias errors ($\theta_t = \theta_{t-1} + \omega_t \Delta t$).
*   **Accelerometers** are noisy in the short term because they pick up road bumps, engine vibration, and suspension travel.
*   **Magnetometers** are slow and susceptible to interference from surrounding metal or electrical currents.

To resolve these errors, the BNO085's internal processor runs an **Extended Kalman Filter (EKF)**. The filter combines the sensors' outputs:
*   The gyroscope provides high-frequency orientation tracking.
*   The accelerometer provides a low-frequency reference for pitch and roll by tracking gravity.
*   The magnetometer provides a low-frequency reference for heading (yaw).

The EKF estimates and subtracts sensor biases on the fly, producing a stable, drift-free orientation quaternion.

---

### 2. Application in this Project
The ESP32 is configured as the I2C master. It initiates all transactions to poll the BNO085. The BNO085 acts as the slave, delivering processed orientations and accelerations. This architecture frees the ESP32 from complex mathematical computations.

---

### 3. Code Walkthrough
In `Step01_BNO085_Initialization.ino.ino`, the I2C peripheral is configured:
```cpp
// Lines 47-48: Pin mappings for custom wiring layout
static const int PIN_SDA = 22;
static const int PIN_SCL = 21;

// Line 107: Initialize I2C driver
Wire.begin(PIN_SDA, PIN_SCL);
```
The `Wire.begin(SDA, SCL)` function configures the ESP32's internal GPIO matrix to route SCL and SDA signals to physical GPIO pins 21 and 22.

---

### 4. Common Failure Modes
*   **MEMS Saturation**: Intense engine or chassis vibrations can exceed the accelerometer's measurement limit (clipping the output). When the raw data clips, the Kalman filter cannot accurately locate the gravity vector, causing orientation calculation errors.
*   **Magnetometer Distortion**: Placing the sensor near high-current batteries or motors creates localized magnetic fields. This distorts the magnetometer's readings, causing yaw drift.

---

### 5. Key Takeaways
*   **Intelligent Sensor**: The BNO085 does not require the host microcontroller to run sensor-fusion algorithms. The calculations are performed internally on its Cortex-M0+ processor.
*   **Bias Correction**: The BNO085 continually updates its internal calibration parameters to track and subtract sensor drift.

---

# Chapter 3: Circuit Design, Electrical Wiring & Electrical Signals

Proper electrical design and wiring are essential for stable data acquisition, especially in high-vibration automotive environments.

---

### 1. Theory: Circuit Design & Signal Integrity

```
                      +3.3V Power Rail
       ┌──────────────────────┬──────────────────────┬───────────┐
       │                      │                      │           │
       │                   [10kΩ]                 [10kΩ]         │
       │                   Pullup                 Pullup         │
       │                      │                      │           │
 ┌─────┴────────┐             │                      │     ┌─────┴────────┐
 │              │   SDA       ├──────────────────────┼────►│              │
 │    ESP32     │◄────────────┴──────────────────────┼────►│    BNO085    │
 │ (I2C Master) │   SCL                              └────►│  (I2C Slave) │
 │              │◄────────────────────────────────────────►│              │
 │              │                                          │  SA0 -> VCC  │ (Address 0x4B)
 │              │                                          │  PS0 -> GND  │ (I2C Mode)
 │              │                                          │  PS1 -> GND  │ (I2C Mode)
 └─────┬────────┘                                          └─────┬────────┘
       │                                                         │
       └──────────────────────┴──────────────────────────────────┘
                            GND (Common Ground)
```

#### A. Open-Drain Drivers
The I2C bus utilizes open-drain output drivers on the SDA and SCL lines. An open-drain pin is connected to the collector or drain of an internal transistor. The transistor can pull the electrical line to Ground (GND, logical low), but it cannot drive the line high.

#### B. Pull-Up Resistors
To pull the lines high when no device is transmitting, external pull-up resistors (typically $10\,\text{k}\Omega$) are tied between the data lines and the $3.3\,\text{V}$ power rail. 
Without pull-up resistors, the lines float electrically. Stray capacitance on the circuit board holds charge, resulting in logic level errors.
*   **Rise Time ($\tau$)**: The time it takes for a signal line to transition from a logical low to a logical high is governed by the RC time constant:

$$\tau = R \cdot C_{\text{bus}}$$

Where $R$ is the pull-up resistance and $C_{\text{bus}}$ is the parasitic capacitance of the wire. 
If the bus capacitance is high (due to long wires), strong pull-up resistors (lower $R$, e.g., $2.2\,\text{k}\Omega$) must be used to keep rise times fast enough for Fast Mode ($400\,\text{kHz}$) communications.

#### C. Interface Selection (PS0 & PS1)
The BNO085 selects its communication interface by sampling the voltage on the Protocol Select (PS0, PS1) pins during boot. Grounding both pins (PS0 = GND, PS1 = GND) configures the chip for I2C mode.

#### D. Slave Addressing (SA0)
The SA0 pin sets the lowest bit of the BNO085's I2C address. Pulling SA0 high sets the address to `0x4B`, while grounding it sets the address to `0x4A`.

#### E. Decoupling Capacitors
Decoupling capacitors ($0.1\,\mu\text{F}$ and $10\,\mu\text{F}$) act as local energy storage. They are placed close to the VCC and GND pins of the microchips to filter out voltage ripples caused by high-frequency switching.

---

### 2. Application in this Project
*   `SDA` and `SCL` pins are connected between the ESP32 and BNO085 with pull-up resistors.
*   `SA0` is pulled high, setting the slave address to `0x4B`.
*   `PS0` and `PS1` are tied to ground to enable I2C mode.
*   Custom pin mapping routes the I2C signals: SDA to GPIO 22, and SCL to GPIO 21.

---

### 3. Code Walkthrough
In `Step01_BNO085_Initialization.ino.ino`, the pin configurations are defined:
```cpp
// Lines 47-48: Pin mappings for custom wiring layout
static const int PIN_SDA = 22;
static const int PIN_SCL = 21;

// Line 53: Default address for SA0 pulled HIGH
static const uint8_t IMU_I2C_ADDRESS = BNO08x_DEFAULT_ADDRESS; // 0x4B
```
The firmware targets address `0x4B`. If the physical SA0 pin is grounded, the communication will fail with a connection error because the sensor will be listening on address `0x4A`.

---

### 4. Common Failure Modes
*   **Missing Pull-ups**: If no pull-up resistors are installed, SCL and SDA will float near $0\,\text{V}$. The microcontroller will hang during initialization because it cannot detect a high state on SCL.
*   **Long Wire Capacitance**: Using long, unshielded wires between the ESP32 and BNO085 increases the bus capacitance $C_{\text{bus}}$. This rounds off the edges of SCL clock pulses, leading to data corruption at $400\,\text{kHz}$.

---

### 5. Key Takeaways
*   **Passive Pull-up**: I2C depends on pull-up resistors to pull logic lines high.
*   **Addressing Configuration**: The software address must match the physical configuration of the SA0 pin.

---

# Chapter 4: The I2C Protocol Deep-Dive (Clock Stretching, Start/Stop, Timing)

Developing stable device drivers requires understanding the timing states of the physical I2C protocol.

---

### 1. Theory: The I2C Protocol

I2C is a synchronous, half-duplex serial protocol. The master controls the clock line (SCL) and initiates all communication.

```
       Start                                                          Stop
SDA ──┐      ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐    ┌───
      │      │   │   │   │   │   │   │   │   │   │   │   │   │   │    │
      └──────┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴────┘
SCL ────┐    ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐    ┌─────
        │    │   │   │   │   │   │   │   │   │   │   │   │   │   │    │
        └────┘   └───┘   └───┘   └───┘   └───┘   └───┘   └───┘   └────┘
         Start    Bit 7   Bit 6   Bit 5   Bit 4   Bit 3   Bit 2   Bit 1  ACK  Stop
```

#### A. Start and Stop Conditions
I2C logic transitions on the data line (SDA) when the clock line (SCL) is high:
*   **Start Condition (S)**: SDA transitions from **High to Low** while SCL is **High**. This alerts all slave devices on the bus to listen for incoming bytes.
*   **Stop Condition (P)**: SDA transitions from **Low to High** while SCL is **High**. This releases the bus.

#### B. Data Validity
Except for Start and Stop conditions, the data voltage on SDA must remain stable during the High phase of SCL. SDA transitions are only allowed when SCL is Low.

#### C. The ACK/NACK Bit
Following every 8 bits of data, the receiver must respond with a 9th bit:
*   **ACK (Acknowledge)**: The receiver pulls the SDA line low during the 9th clock pulse.
*   **NACK (Not Acknowledge)**: The receiver leaves SDA high. This signals that the device is busy, disconnected, or has finished receiving a read command.

#### D. Clock Stretching
If a slave device cannot process data fast enough to keep up with the master's clock speed, it can hold SCL low after receiving a byte. The master's hardware driver waits until the slave releases SCL before continuing the transfer.

---

### 2. Application in this Project
The ESP32 communicates with the BNO085 over I2C at $400\,\text{kHz}$ (Fast Mode). 

```
Write Sequence:
+---+---------------+---+---+───────────+---+---+
| S | Address (0x4B)| W | A | Data Byte | A | P |
+---+---------------+---+---+───────────+---+---+

Read Sequence:
+---+---------------+---+---+───────────+---+---+
| S | Address (0x4B)| R | A | Data Byte | N | P |
+---+---------------+---+---+───────────+---+---+
(Master pulls SDA low for ACK (A), and leaves it high for NACK (N) to end a read)
```

---

### 3. Code Walkthrough
In `Step01_BNO085_Initialization.ino.ino`, the I2C configuration is setup:
```cpp
// Line 108: Configure clock speed
Wire.setClock(400000);  // Set SCL clock speed to 400,000 Hz
```
This sets SCL to $400\,\text{kHz}$, which corresponds to a clock period ($T$) of:

$$T = \frac{1}{f} = \frac{1}{400\,\text{kHz}} = 2.5\,\mu\text{s}$$

---

### 4. Common Failure Modes
*   **Bus Lockup**: If the BNO085 resets or loses power mid-byte, it may leave SDA pulled low. The ESP32 cannot generate a Stop condition because SDA is stuck low. This locks up the entire I2C bus.
*   **Address Collision**: Attempting to connect to another I2C device that shares the same address (`0x4B`) will corrupt transmissions because both devices will try to drive SDA at the same time.

---

### 5. Key Takeaways
*   **Synchronous Operation**: SCL is driven by the master to synchronize data transmission.
*   **State-driven Logic**: Transitions on SDA when SCL is high are reserved for Start and Stop control conditions.

---

# Chapter 5: BNO085 Sensor Architecture & SH-2 Sensor Fusion

The BNO085 does not use a simple register map. It communicates using Hillcrest Labs' **Sensor Hub 2 (SH-2)** protocol.

---

### 1. Theory: SHTP & SH-2 Protocol Architecture

```
+───────────────────────────────────────────────────────────+
|                 SHTP PACKET ARCHITECTURE                  |
|                                                           |
|  +─────────────────────────────────────────────────────+  |
|  |                 4-Byte Packet Header                |  |
|  |  Byte 0: Length LSB       Byte 1: Length MSB        |  |
|  |  Byte 2: SHTP Channel     Byte 3: Sequence Number   |  |
|  |  (Channel 3 = Input Sensor Reports)                 |  |
|  +─────────────────────────┬───────────────────────────+  |
|                            │                              |
|                            v                              |
|  +─────────────────────────────────────────────────────+  |
|  |                 SH-2 Payload Struct                 |  |
|  |  Byte 0: Report ID (e.g. 0x05 for Rotation Vector)   |  |
|  |  Byte 1: Sequence Number                            |  |
|  |  Byte 2: Status & Accuracy                          |  |
|  |  Bytes 3-N: Data Payload Fields (Q-point Floats)    |  |
|  +─────────────────────────────────────────────────────+  |
+───────────────────────────────────────────────────────────+
```

#### A. Sensor Hub Transport Protocol (SHTP)
The BNO085 implements SHTP to transmit multiple logical data channels over I2C. SHTP channels organize data routing:
*   **Channel 0**: SHTP Control (bus configuration).
*   **Channel 1**: Executable commands (resets and status).
*   **Channel 2**: Control commands (sensor configurations).
*   **Channel 3**: Input sensor reports (data outputs).
*   **Channel 4**: Wake-up sensor reports.

Every SHTP transmission starts with a 4-byte header:
*   **Bytes 0-1**: Packet Length (LSB first, indicating total packet size including header).
*   **Byte 2**: Channel ID.
*   **Byte 3**: Sequence Number.

#### B. SH-2 Report Data Structure
When reading sensor data on Channel 3, the payload is structured as an SH-2 data report:
1.  **Report ID**: Identifies the sensor type (e.g., `0x05` for Rotation Vector, `0x02` for Gyroscope).
2.  **Sequence Number**: Tracks individual reports.
3.  **Status Byte**: Contains calibration status and accuracy level.
4.  **Delay/Timestamp**: Time delta since the last sample.
5.  **Data Fields**: The actual sensor readings, formatted as scaled integers.

##### Q-Point Format
To avoid floating-point calculations on its low-power core, the BNO085 processes values using fixed-point math called **Q-point** formatting. 
A value in Q-point format is represented as:

$$\text{Float Value} = \text{Raw Integer} \times 2^{-\text{Q}}$$

For example, if the Rotation Vector has a Q-point scale of 14, the raw 16-bit integer must be divided by $2^{14} = 16384$ to convert it to a floating-point value. The SparkFun library handles this conversion automatically using scaling definitions from `sh2_SensorValue.c`.

---

### 2. Application in this Project
The firmware configures three report IDs on Channel 3:
1.  `SH2_ROTATION_VECTOR` (Report ID: `0x05`, Q-point: 14)
2.  `SH2_GYROSCOPE_CALIBRATED` (Report ID: `0x02`, Q-point: 9)
3.  `SH2_LINEAR_ACCELERATION` (Report ID: `0x04`, Q-point: 8)

---

### 3. Code Walkthrough
In `Step05_Read_Reports.ino`, the firmware processes incoming SH-2 reports:
```cpp
// Lines 200-205: Reading Rotation Vector fields
rv.i        = imu.sensorValue.un.rotationVector.i;
rv.j        = imu.sensorValue.un.rotationVector.j;
rv.k        = imu.sensorValue.un.rotationVector.k;
rv.real     = imu.sensorValue.un.rotationVector.real;
rv.accuracy = imu.sensorValue.status & 0x03;
```
The `un.rotationVector.i` struct elements are populated by the SparkFun library. It reads the raw bytes from the I2C transfer, extracts the payload fields, and divides them by the appropriate Q-point divisor.

---

### 4. Common Failure Modes
*   **Packet Loss**: If the host microcontroller ignores the interrupt line or runs too slowly, the BNO085's output queue can overflow. The sensor drops packets, which causes sequence number skips.
*   **Uncalibrated Orientation**: If the status byte returns `0` (Unreliable), the Kalman filter has not calibrated itself. The output quaternion will drift under vehicle dynamics.

---

### 5. Key Takeaways
*   **Packet Encapsulation**: The BNO085 encapsulates data in SHTP and SH-2 layers, rather than using raw hardware register addresses.
*   **Fixed-Point Scaling**: Q-point scaling is used to transmit high-resolution physical quantities as integer fields.

---

# Chapter 6: ESP32 Firmware Walkthrough: Steps 01 to 07

This chapter details the progression of the C++ firmware, tracing how the code initializes communication, handles sensor resets, and implements synchronized logging.

---

### 1. Step 01: Detection & Firmware Verification
#### Purpose
Establishes the physical I2C connection, initializes the BNO085 breakout board, and prints ROM firmware version records.

```cpp
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
```
*   `Wire.h`: The standard Arduino I2C communication library.
*   `SparkFun_BNO08x_Arduino_Library.h`: The driver interface that wraps SHTP and SH-2 commands.

```cpp
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    
    if (!imu.begin(IMU_I2C_ADDRESS, Wire)) {
        Serial.println("[IMU] ERROR: begin() failed.");
        while (1);
    }
}
```
*   `Wire.begin(22, 21)` configures the I2C peripheral pin routing.
*   `imu.begin(0x4B, Wire)` does the following:
    1.  Sends a soft-reset packet to the sensor on SHTP Channel 1.
    2.  Waits for the sensor to boot ($300\,\text{ms}$).
    3.  Queries product IDs using the `sh2_getProdIds()` command to verify communication.
    4.  Configures internal callbacks for event processing.

---

### 2. Step 02: Spontaneous Reset Recovery
#### Purpose
Automotive electrical systems are prone to voltage drops during engine cranking. This step implements recovery logic to configure the sensor if it resets mid-run.

```cpp
void loop() {
    if (imu.wasReset()) {
        handleIMUReset();
    }
}
```
*   `imu.wasReset()` checks an internal status flag. If the BNO085 power dips and the sensor resets, the flag is set.
*   `handleIMUReset()` re-enables all configured sensor reports, as report selections do not survive a hardware reset.

---

### 3. Step 03 & 04: Report Parsing & ID Verification
#### Purpose
Reads raw SHTP packets from the input queue and maps their ID bytes to human-readable strings to confirm configuration rates.

```cpp
void loop() {
    while (imu.getSensorEvent()) {
        processIMUEvent();
    }
}
```
*   `imu.getSensorEvent()` polls the I2C bus. If data is available, it reads the packet header, verifies the channel, decodes the payload, and copies it to `imu.sensorValue`. It returns `true` if a packet was successfully parsed.
*   A `while` loop is used to drain the queue. The sensor can buffer multiple reports, so the loop reads all pending packets until the queue is empty.

---

### 4. Step 05: Data Decoding & Plausibility Check
#### Purpose
Extracts sensor values from the internal payload structures and calculates the magnitude of the rotation quaternion to verify sensor health.

```cpp
void processIMUEvent() {
    switch (imu.sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR:
            rv.i = imu.sensorValue.un.rotationVector.i;
            rv.j = imu.sensorValue.un.rotationVector.j;
            rv.k = imu.sensorValue.un.rotationVector.k;
            rv.real = imu.sensorValue.un.rotationVector.real;
            rv.fresh = true;
            break;
    }
}
```
*   `imu.sensorValue.sensorId` identifies which sensor report has arrived.
*   The raw data fields are extracted from the `un` union, and the `fresh` flag is set to true.

```cpp
float mag = sqrt(rv.i*rv.i + rv.j*rv.j + rv.k*rv.k + rv.real*rv.real);
```
*   This line calculates the quaternion magnitude. A valid rotation quaternion must have a magnitude of exactly $1.0$.

---

### 5. Step 06: Sensor Synchronization
#### Purpose
Resolves the time offset between different sensor reports. Since the accelerometer, gyroscope, and rotation vector update at slightly different times, we use a logic gate to synchronize the logged rows.

```cpp
void syncAcquisition() {
    if (rv.fresh && gyro.fresh && accel.fresh) {
        emitSynchronizedRow();
        rv.fresh = false;
        gyro.fresh = false;
        accel.fresh = false;
    }
}
```
*   Data is only emitted when all three sensor reports have been updated.
*   Once printed, the `fresh` flags are reset to false. This prevents duplicate or mismatched data rows.

---

### 6. Step 07: CSV Logging Stream
#### Purpose
Generates a structured CSV stream on the serial port, using `#` to mark all diagnostic and startup logs so the Python data-logger can ignore them.

```cpp
void emitCSVRow() {
    char buf[160];
    snprintf(buf, sizeof(buf),
             "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             micros(),
             (double)rv.i, (double)rv.j, (double)rv.k, (double)rv.real,
             (double)gyro.x, (double)gyro.y, (double)gyro.z,
             (double)accel.x, (double)accel.y, (double)accel.z);
    Serial.print(buf);
}
```
*   `micros()` provides microsecond resolution for accurate sample rate analysis.
*   `snprintf` formats the values into a single character array buffer. This reduces printing overhead and ensures the entire row is sent in a single transaction.
*   Values are cast to `double` because the ESP32's implementation of `snprintf` requires doubles for the `%f` format specifier.

---

### 7. Common Mistakes
*   **Blocking delay() calls**: Using `delay()` in the loop halts the execution thread. The I2C bus will stall, causing packet drops and sequence overflows on the sensor.
*   **Direct Struct Copying**: Copying the memory of `imu.sensorValue` directly without checking the `sensorId` can result in reading invalid data because the fields share the same memory location in a union.

---

### 8. Key Takeaways
*   **Reset Handling**: Power recovery is essential for reliable operation on a physical vehicle.
*   **Output Buffering**: Formatting data into a single string buffer and flushing the serial line ensures consistent logging throughput.

---

# Chapter 7: C++ Programming Concepts

Embedded firmware requires careful memory management and a solid understanding of C++ structures.

---

### 1. Theory: Core Programming Concepts

#### A. Objects and Classes
*   **Class**: A blueprint that defines the variables (data members) and functions (methods) of an entity.
*   **Object**: An instance of a class.
*   In our project, `BNO08x` is a C++ class defined by the SparkFun library, and `imu` is the physical object we declare in memory:
    ```cpp
    BNO08x imu; // Instantiates the IMU object
    ```

#### B. Pointers and References
*   **Pointer (`*`)**: A variable that holds the raw memory address of another variable.
*   **Reference (`&`)**: An alias or alternative name for an existing variable in memory.
*   Passing arguments by reference avoids copying data structures in memory:
    ```cpp
    bool begin(uint8_t address, TwoWire &wirePort);
    ```
    `TwoWire &wirePort` is a reference to the physical `Wire` peripheral object, rather than a copy.

#### C. Structs vs. Unions
*   **Struct**: A data structure where each member variable has its own unique location in memory.
    ```cpp
    struct GyroData {
        float x; // 4 bytes
        float y; // 4 bytes
        float z; // 4 bytes
    }; // Total memory size = 12 bytes
    ```
*   **Union**: A data structure where all member variables share the same memory space. The size of the union is the size of its largest member.
    ```cpp
    union SensorValues {
        sh2_Acceleration_t acceleration;
        sh2_Gyroscope_t gyroscope;
    }; // Memory size matches the larger of the two structs.
    ```
    *Safety Rule*: You must only read the union member that corresponds to the active type field (like `sensorId`). Reading other fields returns garbled data.

---

### 2. Application in this Project
The SparkFun driver uses structs and unions to manage SHTP packets:
```cpp
// From SparkFun_BNO08x_Arduino_Library.h
typedef struct {
    uint8_t sensorId;
    uint8_t status;
    union {
        sh2_RotationVector_t rotationVector;
        sh2_Gyroscope_t gyroscope;
        sh2_LinearAcceleration_t linearAcceleration;
    } un;
} sh2_SensorValue_t;
```
When `sensorId` is `SH2_ROTATION_VECTOR`, the fields in `un.rotationVector` are valid. If you read `un.gyroscope` at that time, you will get incorrect values because they point to the same memory location.

---

### 3. Code Walkthrough
In `Step05_Read_Reports.ino`, data is copied from the library union into our local structs:
```cpp
// Lines 209-214: Accessing union members
gyro.x      = imu.sensorValue.un.gyroscope.x;
gyro.y      = imu.sensorValue.un.gyroscope.y;
gyro.z      = imu.sensorValue.un.gyroscope.z;
gyro.status = imu.sensorValue.status & 0x03;
```
This copies values from the shared memory space of the union (`un.gyroscope.x`) into the dedicated memory variables of our local `GyroData` struct.

---

### 4. Common Mistakes
*   **Null Pointer Dereferencing**: Attempting to read or write to a pointer that is set to `nullptr` or has not been initialized. This causes a processor crash (Guru Meditation Error) on the ESP32.
*   **Union Field Corruption**: Accessing an incorrect union field (e.g., reading `un.gyroscope.x` when the report is an acceleration update) returns garbled values.

---

### 5. Key Takeaways
*   **Unions save memory** by reusing the same memory address for different report types.
*   **Passing references** saves CPU cycles by preventing unnecessary data copying in loop execution.

---

# Chapter 8: Data Flow Packet Tracing

Let's trace the path of a single measurement from the physical sensor to the final vehicle dynamics plot.

---

### 1. Step-by-Step Data Flow

```
+───────────────────────────────────────────────────────────+
|                 DATA PIPELINE PACKET TRACE                |
+───────────────────────────────────────────────────────────+
  1. PHYSICAL MOVEMENT
     - Vehicle rolls, creating lateral acceleration.
     │
  2. MEMS TRANSDUCTION
     - Silicon proof mass deflects, changing capacitance.
     │
  3. DIGITAL CONVERSION
     - The sensor's ASIC converts capacitance changes to digital values.
     │
  4. SENSOR FUSION
     - The EKF EKF combines the signals to calculate orientation.
     │
  5. SHTP PACKET WRAPPING
     - Data is formatted as a Q-point payload with an SHTP header.
     │
  6. I2C TRANSMISSION
     - ESP32 reads the data packets over physical I2C lines.
     │
  7. FIRMWARE SYNCHRONIZATION
     - The ESP32 waits until all three sensor updates are fresh.
     │
  8. SERIAL OUTPUT
     - Emits a comma-separated row: "Time_us,Quat,Gyro,Accel".
     │
  9. PYTHON CAPTURE
     - The logging script reads the serial stream and writes it to a CSV.
     │
  10. ANALYSIS PIPELINE
      - Butterworth filters remove high-frequency vibration.
      - Quaternions are converted to Euler Roll, Pitch, and Yaw.
      │
  11. DYNAMICS PLOT
      - Streamlit displays the roll gradient regression line.
+───────────────────────────────────────────────────────────+
```

#### Detailed Path Analysis
1.  **Chassis Movement**: The vehicle enters a constant radius curve. The chassis rolls outward, generating lateral acceleration.
2.  **Transduction**: Inertial forces displace the accelerometer's silicon proof mass, while Coriolis forces deflect the vibrating gyroscope elements.
3.  **A/D Conversion**: The BNO085's internal analog-to-digital converter converts the capacitive changes to digital values.
4.  **Fusion Execution**: The SH-2 Kalman filter processes these inputs to calculate the vehicle's orientation quaternion.
5.  **Payload Packaging**: The orientation is formatted as Q-point scaled integers in an SH-2 packet. An SHTP header is appended to route the packet to Channel 3.
6.  **I2C Transfer**: The ESP32 pulls SDA and SCL to read the packet from the sensor's output queue.
7.  **Library Parsing**: The SparkFun driver decodes the Q-point scaling to populate `imu.sensorValue`.
8.  **Synchronization Gate**: The firmware verifies that all three sensor records are updated, then writes a CSV row to the serial port.
9.  **Storage logging**: The host python script captures the serial line and appends the row to the CSV file.
10. **Signal Processing**: The analysis pipeline reads the CSV, filters out high-frequency noise, and converts the quaternions to Euler angles.
11. **Visualization**: The Streamlit dashboard plots the roll angle vs. lateral acceleration to calculate the vehicle's roll gradient.

---

### 2. Verification
You can verify the data path by running the logging script and inspecting the output values:
*   **Stationary Check**: Accelerations on X and Y should be close to $0.0\,\text{m/s}^2$ (since gravity is mathematically removed), and the quaternion magnitude ($|q|$) should be $1.0000$.
*   **Calibration Check**: The status byte should read `3` (High Accuracy) for all sensors when they are fully calibrated.

---

# Chapter 9: Sensor Reports & Vehicle Dynamics Interpretation

To extract meaningful vehicle parameters, we must connect the sensor's coordinate system to the physical behavior of the vehicle chassis.

---

### 1. Theory: Coordinates & Motion Physics

```
                    Longitudinal Axis (X-axis, Forward)
                                   ▲
                                   │  +Roll (Clockwise tilt)
                                   │   ┌───┐
                                   │  ◄│   │►
                                   │   └───┘
  Lateral Axis ◄───────────────────┼───────────────────► Lateral Axis
  (Y-axis, Left)                   │                   (Y-axis, Right)
                                   │
                                   │  +Pitch (Dive forward)
                                   ▼
                             Vertical Axis (Z-axis, Up)
                             +Yaw (Counter-clockwise rotation)
```

#### A. Chassis Coordinate Standards
In vehicle dynamics, we coordinate the sensor's axes with the vehicle's chassis frame (ISO standard):
*   **X-axis (Longitudinal)**: Positive forward. Measures acceleration during launching and braking.
*   **Y-axis (Lateral)**: Positive to the left. Measures cornering forces.
*   **Z-axis (Vertical)**: Positive upward. Measures ride bumps, suspension compression, and rebound.
*   **Roll ($\phi$)**: Rotation about the X-axis (body roll during cornering).
*   **Pitch ($\theta$)**: Rotation about the Y-axis (dive during braking, squat during acceleration).
*   **Yaw ($\psi$)**: Rotation about the Z-axis (heading changes and drift).

#### B. Lateral Acceleration & Weight Transfer
When a vehicle corners at speed $v$ along a radius $R$, it experiences lateral acceleration $a_y$:

$$a_y = \frac{v^2}{R}$$

This acceleration acts through the vehicle's Center of Gravity (CG). It generates a lateral roll moment that transfers load from the inside tires to the outside tires:

$$\Delta F_{z,\text{lateral}} = \frac{m \cdot a_y \cdot h}{T}$$

Where $m$ is the vehicle mass, $h$ is the CG height above the roll center, and $T$ is the track width. 

This lateral load transfer reduces the overall cornering capability of the tires. Minimizing CG height and controlling the chassis roll gradient are critical for maximizing grip.

```
       Chassis Roll Centered at CG
            ┌───────────────┐
            │       CG      │ ──► Lateral Force (m * ay)
            └───────┬───────┘
              ◄─────┼─────►  Chassis Rolls φ
                    │ h (CG Height)
          ──────────┴──────────  Roll Center
           ▲                 ▼
       Inside Tire       Outside Tire
      Load Decreases    Load Increases
```

#### C. Roll Gradient (RG)
The roll gradient is the rate of chassis roll tilt per unit of lateral acceleration:

$$\text{RG} = \frac{d\phi}{da_y}$$

It is expressed in degrees per g ($^\circ/g$). A lower roll gradient (e.g., $3.5^\circ/g$ for a sports car compared to $8.0^\circ/g$ for an SUV) indicates stiffer anti-roll bars or suspension springs, which helps keep the tires flatter to the road surface during cornering.

#### H. Understeer Gradient (UG)
The understeer gradient describes how the steering angle ($\delta$) changes with lateral acceleration during steady-state cornering:

$$\delta = \frac{L}{R} + \text{UG} \cdot a_y$$

Where $L$ is the wheelbase, $R$ is the turn radius, and $a_y$ is the lateral acceleration in g.
*   **UG > 0 (Understeer)**: The driver must turn the steering wheel further as speed increases to maintain the same cornering radius. This is the default setup for passenger cars because it is stable and predictable.
*   **UG = 0 (Neutral steer)**: The steering angle remains constant as speed increases.
*   **UG < 0 (Oversteer)**: The steering wheel must be turned back toward center as speed increases. This can lead to instability and spin-outs.

---

### 2. Application in this Project
The analysis script `step11_iso4138_analysis.py` extracts steady-state window samples and applies linear regression to calculate the Roll Gradient (RG) and the Yaw Rate Gradient (YRG) for the vehicle.

---

### 3. Code Walkthrough
In `step10_vehicle_dynamics.py`, the raw linear accelerations are converted to standard g-forces:
```python
# Lines 93-95: Convert raw acceleration (m/s²) to g forces
df['LongAccel_g'] = df['LinearAccelX'] / G
df['LatAccel_g']  = df['LinearAccelY'] / G
df['VertAccel_g'] = df['LinearAccelZ'] / G
```
Here, `G` is defined as $9.81\,\text{m/s}^2$. Dividing the raw IMU values by `G` converts the accelerations into standard dimensionless gravitational force units ($g$), which are standard in vehicle dynamics logging.

---

### 4. Common Mistakes
*   **Incorrect Mounting Orientation**: Mounting the sensor sideways without updating the axis directions in software. This maps longitudinal acceleration to the lateral axis, causing incorrect roll and pitch calculations.
*   **Ignoring Suspended Pitch**: When measuring lateral acceleration, chassis roll tilts the sensor relative to the gravity vector. If this tilt is not compensated, a component of gravity will project onto the lateral sensor axis, distorting the acceleration measurements. The BNO085's `SH2_LINEAR_ACCELERATION` report handles this by mathematically removing the gravity vector before outputting the readings.

---

### 5. Key Takeaways
*   **Weight Transfer**: Lateral acceleration creates a roll moment that transfers weight to the outside tires, changing the balance of grip.
*   **Steady-State Metrics**: The Roll Gradient ($^\circ/g$) and Understeer Gradient are key metrics used to quantify vehicle handling and stability.

---

# Chapter 10: Mathematical Derivations

This chapter contains mathematical derivations for quaternions, euler conversions, low-pass filtering, and coordinate transformations.

---

### 1. Quaternions and Orientation Representing
A quaternion is a four-dimensional complex number used to represent 3D rotations without suffering from gimbal lock (which occurs when using Euler angles, where two rotation axes align and lock out a degree of freedom).

A unit quaternion is written as:

$$\mathbf{q} = q_r + q_i\mathbf{i} + q_j\mathbf{j} + q_k\mathbf{k}$$

Where $q_r$ is the scalar (real) component, and $q_i, q_j, q_k$ are the vector (imaginary) components. The imaginary units satisfy the relations:

$$\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{i}\mathbf{j}\mathbf{k} = -1$$

A rotation of angle $\theta$ about a unit vector axis $\vec{u} = [u_x, u_y, u_z]^T$ is represented by the quaternion:

$$\mathbf{q} = \cos\left(\frac{\theta}{2}\right) + \sin\left(\frac{\theta}{2}\right)\left(u_x\mathbf{i} + u_y\mathbf{j} + u_z\mathbf{k}\right)$$

For a quaternion to represent a valid rotation, its magnitude must be exactly $1.0$:

$$\|\mathbf{q}\| = \sqrt{q_r^2 + q_i^2 + q_j^2 + q_k^2} = 1.0$$

---

### 2. Derivation: Quaternion to ZYX Euler Angles
Euler angles express rotation as a sequence of three rotations about orthogonal axes. We use the aerospace standard **ZYX sequence** (Yaw $\psi$, then Pitch $\theta$, then Roll $\phi$).

The rotation matrix $R_{ZYX}$ is constructed by multiplying the individual rotation matrices:

$$R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}, \quad R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

$$R_{ZYX} = R_z(\psi) R_y(\theta) R_x(\phi)$$

$$R_{ZYX} = \begin{bmatrix} 
\cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\
\sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}$$

We can also express this rotation matrix in terms of quaternion elements:

$$R(\mathbf{q}) = \begin{bmatrix}
1 - 2(q_j^2 + q_k^2) & 2(q_i q_j - q_r q_k) & 2(q_i q_k + q_r q_j) \\
2(q_i q_j + q_r q_k) & 1 - 2(q_i^2 + q_k^2) & 2(q_j q_k - q_r q_i) \\
2(q_i q_k - q_r q_j) & 2(q_j q_k + q_r q_i) & 1 - 2(q_i^2 + q_j^2)
\end{bmatrix}$$

By equating the elements of the two matrices, we derive the conversion formulas:

#### Roll ($\phi$)
Equating the elements in row 3, columns 2 and 3:

$$\frac{R_{32}}{R_{33}} = \frac{\cos\theta\sin\phi}{\cos\theta\cos\phi} = \tan\phi$$

$$\phi = \text{atan2}\left(2(q_r q_i + q_j q_k), 1 - 2(q_i^2 + q_j^2)\right)$$

#### Pitch ($\theta$)
Equating the element in row 3, column 1:

$$R_{31} = -\sin\theta \implies \theta = \arcsin(-R_{31})$$

$$\theta = \arcsin\left(2(q_r q_j - q_i q_k)\right)$$

We clip the input of the arcsin function to $[-1, 1]$ to prevent mathematical errors due to numerical rounding.

#### Yaw ($\psi$)
Equating the elements in row 2, column 1 and row 1, column 1:

$$\frac{R_{21}}{R_{11}} = \frac{\sin\psi\cos\theta}{\cos\psi\cos\theta} = \tan\psi$$

$$\psi = \text{atan2}\left(2(q_r q_k + q_i q_j), 1 - 2(q_j^2 + q_k^2)\right)$$

---

### 3. Derivation: Butterworth Low-Pass Filter
To remove chassis vibration while preserving handling motions, we apply a digital low-pass filter. The continuous-time transfer function of a 2nd-order Butterworth filter is:

$$H(s) = \frac{\omega_c^2}{s^2 + \sqrt{2}\omega_c s + \omega_c^2}$$

Where $\omega_c = 2\pi f_c$ is the cutoff frequency in radians per second.
To implement this filter in software, we map it to the discrete-time Z-domain using the **Bilinear Transform**:

$$s \leftarrow \frac{2}{T_s} \frac{z-1}{z+1}$$

Where $T_s$ is the sampling interval ($1/\text{sample rate}$). This substitution converts the continuous transfer function $H(s)$ into a discrete transfer function $H(z)$:

$$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{a_0 + a_1 z^{-1} + a_2 z^{-2}}$$

Expanding this transfer function yields the difference equation implemented in software:

$$a_0 y[n] = b_0 x[n] + b_1 x[n-1] + b_2 x[n-2] - a_1 y[n-1] - a_2 y[n-2]$$

$$y[n] = \frac{b_0}{a_0} x[n] + \frac{b_1}{a_0} x[n-1] + \frac{b_2}{a_0} x[n-2] - \frac{a_1}{a_0} y[n-1] - \frac{a_2}{a_0} y[n-2]$$

Where:
*   $x[n]$ is the current raw sensor input.
*   $y[n]$ is the current filtered output.
*   $x[n-1], x[n-2]$ are past raw inputs.
*   $y[n-1], y[n-2]$ are past filtered outputs.
*   $a_i, b_i$ are the filter coefficients computed from the cutoff frequency and sample rate.

---

### 4. Code Walkthrough
In `imu_utils.py`, the quaternion-to-euler conversions are implemented:
```python
# Lines 40-56: Quaternion conversion function
def quaternion_to_euler(qi, qj, qk, qr):
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll      = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    sinp  = np.clip(2.0 * (qr * qj - qk * qi), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw       = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw
```
The python function uses `numpy` vector operations to convert lists of quaternion values. The `np.clip` function restricts the input of the arcsin to $[-1.0, 1.0]$ to prevent mathematical domain errors.

---

### 5. Common Mistakes
*   **Gimbal Lock**: Attempting to use Euler angles instead of quaternions to combine rotations. If pitch reaches $\pm 90^\circ$, yaw and roll align, losing a degree of freedom. Quaternions avoid this lockup.
*   **Filter Phase Shift**: Straining data through a causal real-time filter shifts the phase of the filtered signal, delaying it in time. To prevent phase shift during analysis, `scipy.signal.filtfilt` applies the filter forward and backward, cancelling out the delay.

---

### 6. Key Takeaways
*   **Quaternions** are 4D representations of rotation that prevent gimbal lock.
*   **The Bilinear Transform** maps continuous-time filters to discrete-time difference equations.

---

# Chapter 11: Debugging & Troubleshooting

This chapter details debugging procedures for common hardware, protocol, and firmware issues.

---

### 1. Debugging Matrix

| Symptom | Root Cause | Debugging Procedure | Fix |
| :--- | :--- | :--- | :--- |
| **ESP32 hangs at `imu.begin()`** | 1. Missing pull-ups on I2C.<br>2. Incorrect I2C Address.<br>3. Sensor not powered. | 1. Measure voltage on SDA/SCL (should be 3.3V).<br>2. Run I2C Scanner to find active address.<br>3. Verify 3.3V rail. | 1. Install 10kΩ pull-up resistors.<br>2. Set address in software to match SA0.<br>3. Check power connections. |
| **Gradients return low $R^2$** | 1. Driver vibration noise.<br>2. Insufficient steady-state data. | 1. Check frequency spectrum of raw acceleration.<br>2. Check length of steady-state driving windows. | 1. Apply a 2Hz cutoff low-pass filter.<br>2. Drive longer circles during testing. |
| **`syncAcquisition` returns rate of 0** | 1. One or more sensor reports failed to enable. | 1. Check return status of `imu.enableReport()` calls.<br>2. Verify memory allocation of data structs. | 1. Ensure BNO085 firmware version matches library expectations. |
| **Spontaneous resets** | 1. Power dips during engine crank.<br>2. SHTP buffer overflows. | 1. Measure VCC with oscilloscope during cranking.<br>2. Verify loop cycle execution time. | 1. Install decoupling capacitors.<br>2. Remove blocking print statements. |

---

### 2. Expected Serial Output Diagnostics
When working correctly, the serial output should display:
*   During initialization:
    ```text
    # 
    # =================================
    # Vehicle Dynamics IMU Project
    # Step 07 - CSV Logger
    # =================================
    # 
    # [I2C] SDA: GPIO 22
    # [I2C] SCL: GPIO 21
    # [I2C] Clock: 400 kHz
    # [IMU] Opening SH-2 session...
    # [IMU] SH-2 session opened successfully.
    # [IMU] SH-2 Firmware:
    # [IMU]  0  Part 10003608  v3.2.13  Build 6
    # [IMU] All reports enabled successfully.
    # [IMU] Reset cause: Power-On Reset
    # 
    # Synchronized acquisition ready.
    # Starting CSV stream...
    # 
    Time_us,Quaternion_i,Quaternion_j,Quaternion_k,Quaternion_real,GyroX,GyroY,GyroZ,LinearAccelX,LinearAccelY,LinearAccelZ
    ```
*   During operation:
    ```text
    2405068,0.0024,-0.0041,0.7071,0.7071,-0.0012,0.0004,0.0051,0.0120,-0.0045,0.0089
    ```

---

### 3. Key Takeaways
*   **Voltage Verification**: Verify voltages on signal and power lines before debugging software.
*   **Diagnostic Markers**: Use `#` prefixes to keep debugging logs from interfering with data parsers.

---

# Chapter 12: Learning Roadmap & References

This roadmap outlines key concepts, reference books, and papers for continuing your education in embedded systems and vehicle dynamics.

---

### 1. Conceptual Progression

```
  [EMBEDDED SYSTEM FUNDAMENTALS]
  - C/C++ memory management and structures (Structs, Unions)
  - Hardware buses (I2C, SPI, UART)
  │
  ▼
  [SIGNAL PROCESSING]
  - Quaternion transformations
  - Digital filters (Butterworth difference equations)
  - Extended Kalman Filters (EKF)
  │
  ▼
  [AUTOMOTIVE CONTROLS]
  - Coordinate system mapping (ISO standards)
  - Accelerometer gravity compensation
  │
  ▼
  [VEHICLE DYNAMICS]
  - Load transfer models
  - Steady-state testing (ISO 4138)
  - Understeer/Oversteer gradients
```

---

### 2. Recommended Resources

#### Books
1.  **"Race Car Vehicle Dynamics"** (William F. Milliken and Douglas L. Milliken)
    *   The standard reference for vehicle handling mechanics. Focuses on tire characteristics and lateral load transfer.
2.  **"Fundamentals of Vehicle Dynamics"** (Thomas D. Gillespie)
    *   A first-principles guide to suspension geometries, roll centers, and understeer gradients.
3.  **"Introduction to Embedded Systems"** (Edward A. Lee and Sanjit A. Seshia)
    *   Covers digital I/O, synchronous communication buses, and task scheduling.

#### Technical Papers
1.  **ISO 4138:2021** - *"Passenger cars — Steady-state circular test-procedures"*
    *   Documents the testing standards and validation criteria used in Chapter 11.
2.  **"An Introduction to the Kalman Filter"** (Greg Welch and Gary Bishop)
    *   A practical guide to the mathematics behind the sensor fusion algorithms running inside the BNO085.

#### Videos & Online Material
1.  **"Understand Understeer and Oversteer Gradients"** (SAE International Videos)
    *   Visualizes the steering angle relationships derived in Chapter 9.
2.  **"CEVA Hillcrest Labs SH-2 Reference Manual"**
    *   Details the structure of SHTP packet layouts and sensor report formatting.

---

# Chapter 13: PCB Design — `esp32_bno085_shield` (KiCad)

Rather than wiring the ESP32 and BNO085 with loose bench jumpers, a dedicated carrier PCB was designed in **KiCad 10.0.4 (Pcbnew)** to make the sensor assembly rigid and repeatable for in-vehicle testing.

---

### 1. Theory: Why a Custom Board Instead of Jumper Wires
Loose dupont wires add uncontrolled length to the SDA/SCL traces, which increases bus capacitance $C_{\text{bus}}$ (Chapter 3/4) and slows signal rise time. On a moving vehicle, wire connectors are also a vibration-induced intermittent-connection risk. A fixed PCB solves both: trace lengths (and therefore capacitance) are fixed and short, and there are no crimped connectors between the two chips to shake loose.

---

### 2. Application in this Project
The board, named **`esp32_bno085_shield`**, mounts the ESP32 Dev Module and the BNO085 breakout together and implements the I2C interface circuit from Chapter 3 directly in copper: 10 kΩ SDA/SCL pull-ups to 3.3 V, 0.1 µF / 10 µF decoupling at the BNO085's power pins, SA0 strapped high (address `0x4B`), PS0/PS1 strapped to ground (I2C mode), and routing that matches the firmware's GPIO 22 (SDA) / GPIO 21 (SCL) pin mapping from Step 01.

**Board specification** (from the exported Gerber job file, `Gerber_PCB/esp32_bno085_shield-job.gbrjob`):

| Property | Value |
|---|---|
| Board outline | 77.55 mm × 65.55 mm |
| Layers | 2 (F.Cu / B.Cu) |
| Board thickness | 1.6 mm |
| Copper weight | 0.035 mm (1 oz) per layer |
| Dielectric core | FR4, 1.51 mm |
| Solder mask | 0.01 mm, top and bottom |
| Surface finish | None specified |
| Min track / clearance | 0.2 mm (pad-to-pad, pad-to-track, track-to-track), outer layers |
| Design date | 2026-07-06 |

**Stack-up (top to bottom):**
```
Top Silkscreen
Top Solder Paste
Top Solder Mask     (0.01 mm)
F.Cu                (0.035 mm)   ← Layer 1
FR4 core            (1.51 mm)
B.Cu                (0.035 mm)   ← Layer 2
Bottom Solder Mask  (0.01 mm)
Bottom Solder Paste
Bottom Silkscreen
```

---

### 3. Code Walkthrough — Fabrication Outputs
`Gerber_PCB/` (also zipped as `Gerber_PCB.zip`) holds the complete, board-house-ready export:

```text
esp32_bno085_shield-F_Cu.gbr / -B_Cu.gbr             Copper layers
esp32_bno085_shield-F_Mask.gbr / -B_Mask.gbr         Solder mask
esp32_bno085_shield-F_Paste.gbr / -B_Paste.gbr       Solder paste stencil
esp32_bno085_shield-F_Silkscreen.gbr / -B_Silkscreen.gbr   Legend
esp32_bno085_shield-Edge_Cuts.gbr                    Board outline
esp32_bno085_shield-PTH.drl / -NPTH.drl              Plated / non-plated drills
esp32_bno085_shield-job.gbrjob                       Machine-readable stack-up (source of table above)
```

---

### 4. Common Mistakes
*   **Losing the editable source**: only the exported Gerber/drill set exists in this project workspace — the editable `.kicad_pro` / `.kicad_sch` / `.kicad_pcb` files were not found here. Gerbers can be manufactured from but not edited; if the board ever needs a revision (e.g. moving to Understeer Gradient hardware in Chapter 15), the original KiCad project should be located and archived alongside the Gerbers.
*   **Wire-swap mismatch**: because SDA/SCL are intentionally routed opposite to the ESP32's silkscreen defaults (Chapter 2), any future board revision must keep the firmware's `PIN_SDA=22, PIN_SCL=21` in sync with whatever the copper actually routes to.

---

### 5. Key Takeaways
*   **Fixed-geometry wiring** removes the two dominant field-failure modes of a breadboard IMU rig: connector vibration and uncontrolled bus capacitance.
*   **The board strapping (SA0/PS0/PS1) is baked into copper**, not jumpers, so the firmware's hardcoded I2C address (`0x4B`) and mode assumptions are guaranteed to match the hardware every time it's assembled.

---

# Chapter 14: The Streamlit Dashboard — User Interface

`app.py` plus the `dashboard/` package wrap the entire firmware→analysis pipeline (Chapters 1, 6, 9) in a single browser page, so a full test cycle — start logging, wait, view Roll Gradient — never requires a terminal.

---

### 1. Theory: Why a Background-Thread UI
Streamlit re-runs the whole script top-to-bottom on every interaction, which is incompatible with a serial logger or a multi-second analysis pipeline running inline (the UI would freeze or lose the running process on the next rerun). The dashboard instead runs the serial logger and the analysis pipeline each on a **daemon background thread**, storing progress in a small state object (`LogState`, `PipelineState`) that lives in `st.session_state`. The UI thread polls that object and calls `st.rerun()` every 0.5 s while busy, so the page updates live without blocking on I/O.

---

### 2. Application in this Project
The package is split by responsibility:

| Module | Responsibility |
|---|---|
| `dashboard/ui.py` | Page layout and all widgets |
| `dashboard/state.py` | Session-state key constants; single source of truth mapping widget keys to preset/vehicle JSON keys |
| `dashboard/runner.py` | Background-thread serial logger — opens the COM port, waits up to 30 s for the exact Step 07 CSV header, streams rows to `imu_log_*.csv`, tracks elapsed time / row count / sample rate |
| `dashboard/pipeline.py` | Background-thread analysis runner — validates the CSV's columns, then runs Step09 → Step10 → Step11 in order, collecting each step's numeric summary and figure |
| `dashboard/presets.py` | Saves/loads named vehicle configurations as JSON under `presets/` (e.g. `presets/HandTest.json`) |

**Page layout, top to bottom:**
1. **Vehicle Configuration** (left column) — name, mass, wheelbase, CG height, CG-to-front, track width, front-axle mass, with validation (all > 0, front mass < total mass, CG-to-front < wheelbase) and Save/Load preset controls.
2. **Test Control** (right column) — file name (optional), COM port (default `COM12`), test radius (default 10 m), and a single **START TEST / STOP TEST** button whose label and action swap based on live logger state.
3. **Results** (bottom, full width) — three tabs: **Raw Sensor** (Step09 plot + row count/duration/rate/quaternion-health metrics), **Vehicle Dynamics** (Step10 plot + Roll/Pitch/Yaw and filtered accel min/max), and **ISO 4138** (Step11 plot, a live-rendered regression chart built from the pipeline's numbers rather than a static image, an in-app "Method & Formulae" expander with the LaTeX equations from Chapter 9/10, and PASS/FAIL badges per direction against the R² ≥ 0.98 threshold).

The moment logging finishes (`LogState.status == "done"`), the UI **auto-starts** the analysis pipeline — there is no separate "Analyze" button to click.

---

### 3. Code Walkthrough
In `dashboard/ui.py`, the auto-trigger logic:
```python
if (
    ls is not None
    and ls.status == "done"
    and ls.csv_path
    and (ps is None or ps.status == "idle")
):
    ps = PipelineState()
    st.session_state[state.PIPELINE_STATE] = ps
    pipeline.start(csv_path=ls.csv_path, vehicle=_collect_vehicle(),
                   radius=..., ps=ps, run_name=Path(ls.csv_path).stem)
```
This checks the logger's state on every rerun and starts the pipeline exactly once per completed log.

In `dashboard/pipeline.py`, each test run's plots are archived under a run-specific name instead of overwriting the previous run's fixed-name output:
```python
def _archive_image(src, run_name, suffix):
    safe_name = re.sub(r'[\\/:*?"<>|]', '', run_name).strip()
    dest = plots_path(f"{safe_name}_{suffix}.png")
    shutil.copyfile(src, dest)
    return dest
```
and one summary row is appended per session to a per-vehicle results CSV (`results/{vehicle_name}_results.csv`) that already reserves `ug_*` columns for Understeer Gradient once a steering sensor is added (Chapter 9).

---

### 4. Common Mistakes
*   **Blocking the UI thread**: running the serial read loop or the analysis pipeline directly inside `ui.py` would freeze the page on every Streamlit rerun. Both are pushed onto daemon threads specifically to avoid this.
*   **Losing results on restart**: because results only live in `st.session_state`, restarting the Streamlit server clears them. The UI compensates by falling back to the last-saved PNG in `plots/` with a "stale data" caption rather than showing a blank tab.

---

### 5. Key Takeaways
*   **Thread + poll, not inline execution**, is the pattern that lets a Streamlit single-page app drive a long-running serial/analysis job.
*   **Per-run archiving** (plots and results CSV) turns each test into permanent history instead of a value that gets overwritten by the next run.

---

# Chapter 15: Project Status & Recorded Test Data

---

### 1. Data Collected
Real hardware logging sessions on record (`imu_log_*.csv`), 2026-06-29 through 2026-07-09: 8 sessions on 06-29, 2 on 06-30, 1 on 07-07, and 4 on 07-09 (most recent). One synthetic file, `imu_log_synthetic_EV656_ISO4138.csv`, was generated by `generatedata.py` with a known ground-truth Roll Gradient of 9.5 deg/g specifically to validate the Chapter 9/10 math independent of real sensor noise, before trusting it on vehicle data.

---

### 2. Current Result: 0 Valid Steady-State Windows
`results/unnamed_results.csv` shows every real-vehicle session analyzed so far returned `n_windows_total = 0` — no segment of any recorded drive has yet satisfied all four ISO 4138 acceptance criteria from Chapter 9/§Chapter 6 application (lateral-accel std < 0.02 g, roll std < 0.5°, yaw-rate std < 2.0°/s, sustained ≥ 2.0 s, within 0.05–0.85 g). All logged rows also show a blank `vehicle_name`, meaning no saved preset was loaded for these runs.

Because the pipeline itself is already validated against the synthetic ground-truth dataset (Roll Gradient recovers to 9.5 deg/g on `imu_log_synthetic_EV656_ISO4138.csv`), this points to the **test-execution side**, not the software: the recorded drives need longer, steadier constant-radius segments — driven per ISO 4138's actual procedure — for a valid window to be detected.

---

### 3. Outstanding Work
*   Run a real test with a longer, steadier held-radius phase and a loaded vehicle preset, to produce the first real-world Roll Gradient / Yaw Rate Gradient result.
*   Add a steering-angle sensor to unblock Understeer Gradient (`ug_*` columns already reserved in the results schema, per Chapter 9 §2 and Chapter 14 §2).
*   Locate and archive the editable KiCad project source (Chapter 13 §4) — only the fabrication Gerbers currently exist in this workspace.
