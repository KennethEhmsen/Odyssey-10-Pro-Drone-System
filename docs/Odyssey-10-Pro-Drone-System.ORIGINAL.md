Autonomous Long-Range Quadcopter Engineering Master Specification
=================================================================

**Platform Identification:** Odyssey-10 Pro



**Architecture:** 10-Inch Long-Range, ESP32-P4 Dual-Core RISC-V Avionics, Integrated Perception, Kinetic Recovery & Safety Stack



1. System Architecture & Theory of Operation

--------------------------------------------

The **Odyssey-10 Pro** is built around the dual-core **ESP32-P4** (RISC-V @ up to 400 MHz). The platform decouples hard real-time flight stabilization from asynchronous communication, logging, and perception tasks.


    +---------------------------------------------------------------------------------------------------+
    |                                 ODYSSEY-10 PRO SYSTEM TOPOLOGY                                    |
    +---------------------------------------------------------------------------------------------------+
    |                                                                                                   |
    |  [ CORE 1: 500 Hz DETERMINISTIC FLIGHT LOOP ]                                                     |
    |  * Primary IMU (MPU-6050) & Secondary Redundant IMU (ICM-42688-P) via Fast I2C Bus                |
    |  * 2nd-Order Dynamic Bi-Quad Gyro Notch Filtering (Center: 80 Hz, Q: 4.0)                        |
    |  * Ballistic Free-Fall Detection Engine (|a_total| < 0.15g for >400ms -> Parachute Ejection)       |
    |  * Cascaded 4-DOF Rate & Angle PID Controllers                                                    |
    |  * Quad-X Matrix Mixer Outputting 400 Hz LEDC PWM to 4x 3110 Motors                               |
    |                                                                                                   |
    |  [ CORE 0: ASYNC TELEMETRY, STORAGE, PERCEPTION & SAFETY ]                                         |
    |  * 100 Hz Binary BlackBox Flight Data Recorder (MicroSD via SPI1)                                 |
    |  * MAVLink v2 Telemetry Serialization & 433 MHz SX1278 LoRa Transceiver (SPI2)                     |
    |  * Beitian BN-220 Multi-GNSS Ingest (Galileo/GPS/GLONASS @ 10 Hz via UART1)                       |
    |  * MSP Telemetry Stream to 5.8 GHz 800mW VTX (UART2)                                              |
    |  * Benewake TFmini-S Forward LiDAR Ingest (100 Hz via UART3)                                      |
    |  * Bosch BMP280 Barometer & ST VL53L1X Precision Landing ToF (I2C)                                |
    |  * QMC5883L 3D Magnetometer Heading Lock & INA226 Current/mAh Shunt (I2C)                          |
    |  * Dynamic Energy & Distance Budgeting RTH Engine                                                 |
    |  * Solid-State Power Latch Driver for Isolated 1S S.O.S. Emergency Beacon (GPIO 21)               |
    |  * Native Direct Remote ID (DRI) OpenDroneID Broadcaster                                          |
    |                                                                                                   |
    +---------------------------------------------------------------------------------------------------+

2. Complete Bill of Materials (BOM) & Cost Breakdown

----------------------------------------------------

| **Category**        | **Component / Model**              | **Key Specifications**                                                                   | **Qty** | **Unit Price (USD)** | **Total Cost (USD)** | **Primary Function**                            |
| ------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------- | ------- | -------------------- | -------------------- | ----------------------------------------------- |
| **Airframe**        | XL10 V2 / Mark4-10 Carbon Frame    | 420–450mm Wheelbase, 3K Carbon, 7–8mm Arm Thickness                                      | 1       | $55.00               | $55.00               | Structural chassis & motor mounting             |
| **Propulsion**      | 3110 / 2812 Brushless Motors       | 900KV, 4S–6S rated, M5 Titanium/Steel Shaft                                              | 4       | $22.00               | $88.00               | Primary thrust generation                       |
| **Propellers**      | HQProp / Gemfan 10x5x3 (or 10x4.5) | 10-inch, Glass-Nylon (2 CW, 2 CCW, Props-Out)                                            | 2 Pairs | $6.00                | $12.00               | Aerodynamic lift                                |
| **Drive**           | 4-in-1 ESC (BLHeli_32)             | 50A Continuous, 60A Burst, 6S Input, DShot300/PWM                                        | 1       | $45.00               | $45.00               | Motor phase commutation & power delivery        |
| **Main Battery**    | 6S LiPo Pack (or 6S2P Li-Ion)      | 22.2V Nom, 4500mAh 45C (or 8400mAh 21700 Molicel)                                        | 1       | $110.00              | $110.00              | Primary flight energy source                    |
| **Main MCU**        | ESP32-P4 RISC-V Dev Board          | Dual-Core @ 400 MHz, 3.3V Logic, Native SPI/I2C/UART                                     | 1       | $14.00               | $14.00               | Central flight controller & avionics processor  |
| **Primary IMU**     | InvenSense MPU-6050 Breakout       | 3-Axis Gyro ($\pm500^\circ/\text{s}$), 3-Axis Accel ($\pm8g$), Fast $\text{I}^2\text{C}$ | 1       | $3.50                | $3.50                | Real-time attitude & angular rate sensing       |
| **Backup IMU**      | ICM-42688-P / BMI270 Breakout      | Low-noise 6-Axis Gyro/Accel, Redundant Failover Bus                                      | 1       | $4.00                | $4.00                | Sensor fault detection & voting redundancy      |
| **Barometer**       | Bosch Sensortec BMP280             | Digital Pressure Sensor, 0.16m resolution, $\text{I}^2\text{C}$                          | 1       | $2.50                | $2.50                | Altitude hold & vertical speed (Vario)          |
| **GNSS Unit**       | Beitian BN-220 Module              | Multi-GNSS (Galileo, GPS, GLONASS), 10 Hz, UART                                          | 1       | $14.00               | $14.00               | Coordinates, ground speed, RTH navigation       |
| **Compass / Mag**   | QMC5883L Magnetometer Module       | 3D Magnetic Compass Heading Lock, $\text{I}^2\text{C}$                                   | 1       | $3.50                | $3.50                | Drift-free absolute yaw reference               |
| **Forward LiDAR**   | Benewake TFmini-S Micro-LiDAR      | $0.1\text{--}12\text{m}$ Range, 100 Hz Refresh, UART                                     | 1       | $22.00               | $22.00               | Forward collision avoidance & step-over climb   |
| **Downward ToF**    | STMicroelectronics VL53L1X ToF     | $0\text{--}4\text{m}$ Laser Rangefinder, $\text{I}^2\text{C}$                            | 1       | $6.50                | $6.50                | Precision landing flare (ground-effect bypass)  |
| **Current Shunt**   | INA226 Current/Power Sensor        | High-Side Bi-Directional Shunt, $\text{I}^2\text{C}$                                     | 1       | $3.50                | $3.50                | Real-time Ampere/mAh energy depletion tracking  |
| **Telemetry Link**  | Semtech SX1278 Breakout (Ra-02)    | 433 MHz LoRa, +20 dBm (100mW), SPI, -148 dBm Sensitivity                                 | 2       | $6.00                | $12.00               | Air-to-ground bi-directional MAVLink/RC link    |
| **Video Tx (VTX)**  | 5.8 GHz 800mW–1.6W VTX             | 5.8 GHz Band (48CH), 7–26V Input, MSP OSD Support                                        | 1       | $38.00               | $38.00               | Live first-person video downlink                |
| **FPV Camera**      | RunCam Phoenix 2 Micro             | 1/1.8" Starlight Sensor, 1200TVL, 2.1mm Lens                                             | 1       | $29.00               | $29.00               | Live cockpit video capture                      |
| **Power Regs**      | Dual Step-Down BEC Module          | Input 6S (25.2V) $\rightarrow$ Clean 5V 3A (Logic) & 12V 3A (Video)                      | 1       | $8.00                | $8.00                | Isolated, noise-filtered power rails            |
| **BlackBox Logger** | SPI MicroSD Card Adapter           | High-Speed SPI MicroSD Module + 32GB Class 10 Card                                       | 1       | $3.00                | $3.00                | 100 Hz high-rate binary flight data recording   |
| **Parachute Mech**  | Ejection Canister + Micro Servo    | Spring-loaded tube + 9g Metal Gear Servo (PWM)                                           | 1       | $9.00                | $9.00                | Ballistic kinetic free-fall mitigation          |
| **Beacon Unit**     | Standalone Node (ESP32-C3)         | Ultra-low power MCU, 100 dB Piezo Buzzer, Strobe LED                                     | 1       | $7.00                | $7.00                | Out-of-zone autonomous recovery transmitter     |
| **Beacon Battery**  | 1S LiPo Cell (3.7V 800mAh)         | 1S 3.7V 800mAh with integrated PCM protection circuit                                    | 1       | $6.00                | $6.00                | Isolated energy reserve for beacon (48+ hrs)    |
| **Power Latch**     | Solid-State Switch (P-FET)         | SI2301DS P-FET + 2N3904 NPN driver circuit                                               | 1       | $2.50                | $2.50                | Latches 1S battery on MCU pulse / main VBAT cut |
| **Wiring/Passives** | XT90-S, Caps, TVS, Resistors       | XT90-S Anti-Spark, 1000µF 35V Cap, SMBJ28A TVS Diode                                     | 1 Lot   | $14.50               | $14.50               | Power safety, surge damping, voltage division   |
| **Master Total**    |                                    |                                                                                          |         |                      | **~$514.50**         | **Complete Industrial UAV Platform**            |

3. Power, Propulsion & Sizing Engineering

-----------------------------------------

$$\text{Target All-Up Weight (AUW)} \approx 1850\text{ g}$$

* **Frame + Mechanical Standoffs:** 280 g
  
  

* **4x 3110 Motors:** 360 g
  
  

* **4-in-1 ESC + Avionics Stack + Wiring:** 130 g
  
  

* **VTX + FPV Camera + Antennas:** 85 g
  
  

* **Sensors (LiDAR, ToF, Mag, Current, SD, GPS):** 45 g
  
  

* **Kinetic Parachute System:** 55 g
  
  

* **Auxiliary Beacon Subsystem (1S Batt + Node + Latch):** 45 g
  
  

* **6S 4500mAh LiPo Battery Pack:** 680 g
  
  

* **Payload Reserve:** 170 g
  
  

$$\text{Max Static Thrust per Motor (3110 900KV @ 6S, 10x5 Prop):} \approx 1750\text{ g}$$

$$\text{Total Maximum Thrust} = 4 \times 1750\text{ g} = 7000\text{ g}$$

$$\text{Thrust-to-Weight Ratio} = \frac{7000\text{ g}}{1850\text{ g}} = 3.78:1$$
    Thrust per Motor Profile:
    0% Throttle   --> 0 g
    22% Throttle  --> 462 g  (Hover Equilibrium: 4x 462g = 1848g = Total AUW)
    50% Throttle  --> 980 g  (Fast Cruise: ~60 km/h)
    100% Throttle --> 1750 g (Emergency Punch-Out / Wind Recovery)

4. Frame Dynamics, Motor Layout & Rotation Mechanics

----------------------------------------------------

The platform uses a **"Props-Out" (Reversed Rotation)** Quad-X configuration. This prevents grass and prop-wash debris from being thrown onto the camera lens and improves yaw recovery authority in crosswinds.


                                  FRONT (Heading 0°)

                  [M4: Front-Left]                      [M2: Front-Right]
                     (Clockwise)                      (Counter-Clockwise)
                        CW                                     CCW
                    \        /                            \        /
                     \  TOP /                              \  TOP /
                      \    /                                \    /
                       [M4]                                  [M2]
                         \                                    /
                          \                                  /
                           \        +--------------+        /
                            \-------|   ESP32-P4   |-------/
                                    | AVIONICS BAY |
                            /-------|   BMP/MPU    |-------\
                           /        +--------------+        \
                          /                                  \
                         /                                    \
                       [M3]                                  [M1]
                      /    \                                /    \
                     /  BOT \                              /  BOT \
                    /        \                            /        \
                        CCW                                    CW
                (Counter-Clockwise)                        (Clockwise)
                  [M3: Rear-Left]                       [M1: Rear-Right]

                                   REAR (Tail)

5. Dynamic Power Budgeting, RTH & Failsafe Logic

------------------------------------------------

Core 0 runs an active **Dynamic Energy Budget Engine** evaluating battery voltage and discharge rates against real-time distance from the launch origin.


    +--------------------------------------------------------------------------------------------------+
    |                              DYNAMIC ENERGY & DISTANCE BUDGETING                                 |
    +--------------------------------------------------------------------------------------------------+

     1. Distance Calculation (Haversine Formula):
        d_home = 2R * asin(sqrt(sin^2(Δφ / 2) + cos(φ1) * cos(φ2) * sin^2(Δλ / 2)))

     2. Time Required to Reach Home:
        t_return = (d_home / v_cruise) + t_descent_buffer  (v_cruise = 12 m/s, t_descent = 15 s)

     3. Real-Time Energy Demand Threshold:
        V_req = V_critical_cutoff + (t_return * V_burn_rate) + V_reserve_margin
        (V_critical = 9.9V, V_burn_rate ≈ 0.003 V/s, V_reserve = 0.3V)


                                    +---------------------------------------+
                                    |    Real-Time Energy Assessment (1 Hz) |
                                    +-------------------+-------------------+
                                                        |
                                                        v
                         +-----------------------------------------------------+
                         | Compare Current V_batt against Required Return V_req|
                         +------------------------------+----------------------+
                                                        |
                 +--------------------------------------+--------------------------------------+
                 |                                                                             |
     [V_batt <= V_req & V_batt > 10.2V]                                             [V_batt <= 10.2V & Cannot Reach Origin]
                 |                                                                             |
                 v                                                                             v
    +-----------------------------+                                               +---------------------------------+
    |      AUTOMATIC FULL RTH     |                                               |     VECTOR TOWARDS ORIGIN       |
    | - Climb to Safe AGL (30m)   |                                               | - Fly along Home vector until   |
    | - Fly direct to Origin      |                                               |   voltage drops to 10.2V        |
    | - Auto-descend & Disarm     |                                               +----------------+----------------+
    +-----------------------------+                                                                |
                                                                                                   v
                                                                                  +---------------------------------+
                                                                                  |     REQUEST LAND PERMISSION     |
                                                                                  | - Hold position & altitude      |
                                                                                  | - Transmit "REQ_LAND" via LoRa  |
                                                                                  | - Start 15s Permission Window   |
                                                                                  +----------------+----------------+
                                                                                                   |
                                                          +----------------------------------------+----------------------------------------+
                                                          |                                                                                 |
                                                  [Pilot Approves]                                                          [No Reply / Timeout /]
                                                  ["PERMIT_LAND"]                                                           [V_batt <= 9.9V Critical]
                                                          |                                                                                 |
                                                          v                                                                                 v
                                                 +-----------------+                                                       +-----------------+
                                                 | CONTROLLED LAND |                                                       | UNCONDITIONAL   |
                                                 | AT CURRENT POS  |                                                       | EMERGENCY LAND  |
                                                 +--------+--------+                                                       +--------+--------+
                                                          |                                                                         |
                                                          +-----------------------------------+-------------------------------------+
                                                                                              |
                                                                                              v
                                                                          +---------------------------------------+
                                                                          |        TOUCHDOWN POSITION CHECK       |
                                                                          +-------------------+-------------------+
                                                                                              |
                                                                            [Distance to Home > 15 meters]
                                                                                              |
                                                                                              v
                                                                          +---------------------------------------+
                                                                          |   ACTIVATE INDEPENDENT BEACON SYSTEM  |
                                                                          | - Pulse GPIO 21 to Latch 1S Battery   |
                                                                          | - Fire LoRa S.O.S. Recovery Packets   |
                                                                          | - Engage 100 dB Buzzer & Strobe LED   |
                                                                          +---------------------------------------+

6. Perception, Collision Avoidance & Precision Flare

----------------------------------------------------

    +--------------------------------------------------------------------------------------------------+
    |                               PERCEPTION & PROXIMITY ENGINE                                      |
    +--------------------------------------------------------------------------------------------------+
    |                                                                                                  |
    |  1. Forward LiDAR Collision Bubble (TFmini-S @ 100 Hz via UART3):                                |
    |     - d_obstacle <= 3.5 m: Clamp forward pitch (θ <= -5° active reverse braking) and command     |
    |       a +5.0 m vertical step-over climb.                                                         |
    |     - 3.5 m < d_obstacle <= 6.0 m: Proportional velocity scaling (Scale = (d - 3.5) / 2.5).     |
    |                                                                                                  |
    |  2. Downward Laser Flare Landing (VL53L1X ToF @ 50 Hz via I2C):                                  |
    |     - Active at altitudes < 2.0 m AGL (bypassing barometric ground-effect wash).                 |
    |     - Regulates final touchdown descent rate strictly to -0.2 m/s until contact.                 |
    +--------------------------------------------------------------------------------------------------+

7. Isolated Emergency Locator Beacon Subsystem

----------------------------------------------

                          +-----------------------------------------------------------+
                          |                 MAIN FLIGHT CONTROLLER                    |
                          |   ESP32-P4 (Core 0)                                       |
                          |   - GPIO 21: Solid-State Latch Pulse Output               |
                          |   - GNSS NMEA Ingest (Caches Last Known Coordinates)      |
                          +-----------------------------+-----------------------------+
                                                        |
                                                        | (100ms HIGH Pulse / Main VBAT Loss)
                                                        v
    +-------------------------------------------------------------------------------------------------+
    |                                    INDEPENDENT BEACON HARDWARE                                  |
    |                                                                                                 |
    |   +-----------------------+              +--------------------------------------------------+   |
    |   | Dedicated 1S LiPo     |              |  Solid-State P-FET Latch Circuit (SI2301DS)      |   |
    |   | (3.7V 800mAh Battery) +------------->+  - Latches ON permanently when GPIO 21 pulses    |   |
    |   +-----------------------+              |  - Also triggers passively if Main VBAT is cut   |   |
    |                                          +-------------------------+------------------------+   |
    |                                                                    |                            |
    |                                                                    v (Isolated 3.3V Power)      |
    |   +----------------------------------------------------------------+------------------------+   |
    |   |                         LOW-POWER BEACON NODE (ESP32-C3 / ATtiny)                       |   |
    |   |                                                                                         |   |
    |   |   * SX1278 LoRa Configuration: SF11, BW 62.5 kHz, CR 4/8, +20 dBm (Extreme Range)       |   |
    |   |   * Output: Transmits Last Known GPS Coordinates, Battery mV, and Uptime every 2s       |   |
    |   |   * Acoustics: 100 dB Piezo Buzzer pulses 2.7 kHz resonant burst every 2s               |   |
    |   |   * Optics: High-Power White Strobe LED pulses synchronously with audio                 |   |
    |   |   * Endurance: 48+ Hours continuous operation on an 800mAh 1S cell                      |   |
    |   +-----------------------------------------------------------------------------------------+   |
    +-------------------------------------------------------------------------------------------------+

8. Physical Placement Plan & 3D Spatial Layout

----------------------------------------------

                                      [+120mm GNSS Mast] (GPS + Compass)
                                              /|\
                                             / | \
                                            /  |  \
              [M4: Front-Left CW]          /   |   \           [M2: Front-Right CCW]
                 (10" Prop)               /    |    \               (10" Prop)
                   \===\                 /     |     \                /===/
                    \   \               /  [Top Battery]             /   /
                     \   \             / [6S 4500mAh / 6S2P]        /   /
                      (O)-------------+----------------------------+---(O)
                       |              |       CENTRAL STACK        |    |
       [LiDAR + Cam]---+              | 1. ESP32-P4 Controller     |    +---[5.8GHz VTX]
       (0° Fwd Nose)   |              | 2. IMU Stack (Gel-Damped)  |    |   (RHCP Pagoda)
                       |              | 3. DShot ESC & TVS Diode   |    |
                      (O)-------------+----------------------------+---(O)
                     /   /             \   [1S S.O.S. Beacon]     /   /
                    /   /               \  [Downward ToF Belly]  /   /
                   /===/                 \                      /===/
              [M3: Rear-Left CCW]         \                    [M1: Rear-Right CW]
                 (10" Prop)                \ [433MHz LoRa Dipole]
                                             (Vert. Downward Whip)

### Component Placement Directory

* **Center of Gravity $(X=0, Y=0, Z=0)$:** ESP32-P4 Flight Controller mounted on M3 rubber standoffs with MPU-6050 & BMP280 directly on CoG (damped via polyurethane gel).
  
  

* **Nose $(X=0, Y=+145\text{mm}, Z=+34\text{mm})$:** Forward TFmini-S LiDAR and 1200TVL FPV camera.
  
  

* **Elevated Tail Mast $(X=0, Y=-105\text{mm}, Z=+120\text{mm})$:** Beitian BN-220 GNSS and QMC5883L Compass elevated $+12\text{ cm}$ above the power train to eliminate magnetic interference.
  
  

* **Tail RF $(X=\pm20\text{mm}, Y=-130\text{mm})$:** 5.8 GHz VTX Pagoda antenna pointing up ($+Z$); 433 MHz SX1278 LoRa whip antenna dropping down ($-Z$, $16.5\text{ cm}$).
  
  

* **Undercarriage Belly $(X=0, Y=-35\text{mm}, Z=-4\text{mm})$:** Downward VL53L1X ToF sensor pointing through carbon plate cut-out.
  
  
9. Pinout & Hardware Interconnect Mapping

-----------------------------------------

    ESP32-P4 Master Avionics Pinout:
    -------------------------------------------------------------------------------
    GPIO 1   --> Battery Voltage Divider (100k/10k Input to 12-Bit ADC)
    GPIO 2   --> Physical Pre-Arm Safety Push-Button (Active LOW to GND)
    GPIO 3   --> LoRa SX1278 DIO0 (Interrupt Line)
    GPIO 4   --> Motor 1 ESC PWM Output (Rear-Right, CCW)
    GPIO 5   --> Motor 2 ESC PWM Output (Front-Right, CW)
    GPIO 6   --> Motor 3 ESC PWM Output (Rear-Left, CW)
    GPIO 7   --> I2C SDA Bus (MPU-6050, BMP280, QMC5883L, VL53L1X, INA226)
    GPIO 8   --> I2C SCL Bus (MPU-6050, BMP280, QMC5883L, VL53L1X, INA226)
    GPIO 9   --> LoRa SX1278 RST (Reset Line)
    GPIO 10  --> LoRa SX1278 NSS (SPI2 Chip Select)
    GPIO 11  --> LoRa SX1278 MOSI (SPI2 Data Out)
    GPIO 12  --> LoRa SX1278 SCK (SPI2 Clock)
    GPIO 13  --> LoRa SX1278 MISO (SPI2 Data In)
    GPIO 15  --> Motor 4 ESC PWM Output (Front-Left, CCW)
    GPIO 17  --> GPS BN-220 RX1 (ESP32 RX <- GPS TX @ 115200 Baud)
    GPIO 18  --> GPS BN-220 TX1 (ESP32 TX -> GPS RX @ 115200 Baud)
    GPIO 19  --> VTX MSP OSD TX2 (ESP32 TX -> VTX RX @ 115200 Baud)
    GPIO 20  --> VTX MSP OSD RX2 (ESP32 RX <- VTX TX @ 115200 Baud)
    GPIO 21  --> Emergency Beacon Solid-State Latch Trigger (Active HIGH Pulse)
    GPIO 22  --> Forward TFmini-S LiDAR RX3 (ESP32 RX <- LiDAR TX @ 115200 Baud)
    GPIO 23  --> Forward TFmini-S LiDAR TX3 (ESP32 TX -> LiDAR RX @ 115200 Baud)
    GPIO 26  --> Ballistic Parachute Servo Output (LEDC 50 Hz PWM)
    GPIO 33  --> MicroSD BlackBox CS (SPI1 Chip Select)
    GPIO 34  --> MicroSD BlackBox MOSI (SPI1 Data Out)
    GPIO 35  --> MicroSD BlackBox SCK (SPI1 Clock)
    GPIO 36  --> MicroSD BlackBox MISO (SPI1 Data In)

10. Master Software Implementation

----------------------------------

### 10.1 ESP32-P4 Master Flight & Navigation Controller

C++
    #include <Arduino.h>
    #include <SPI.h>
    #include <LoRa.h>
    #include <TinyGPSPlus.h>
    #include <Wire.h>
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_BMP280.h>
    #include <Adafruit_Sensor.h>
    #include <FS.h>
    #include <SD.h>

    // --- Hardware Pin Definitions ---
    #define PIN_LORA_SCK       12
    #define PIN_LORA_MISO      13
    #define PIN_LORA_MOSI      11
    #define PIN_LORA_NSS       10
    #define PIN_LORA_RST       9
    #define PIN_LORA_DIO0      3

    #define PIN_SD_CS          33
    #define PIN_SD_MOSI        34
    #define PIN_SD_SCK         35
    #define PIN_SD_MISO        36

    #define PIN_PARACHUTE_SRV  26
    #define PIN_GPS_RX         17
    #define PIN_GPS_TX         18
    #define PIN_VTX_TX         19
    #define PIN_VTX_RX         20
    #define PIN_LIDAR_RX       22
    #define PIN_LIDAR_TX       23

    #define PIN_I2C_SDA        7
    #define PIN_I2C_SCL        8

    #define PIN_BATT_ADC       1
    #define PIN_ARM_SWITCH     2
    #define PIN_BEACON_TRIGGER 21

    #define MOTOR1_PIN         4   // Rear-Right (CCW)
    #define MOTOR2_PIN         5   // Front-Right (CW)
    #define MOTOR3_PIN         6   // Rear-Left (CW)
    #define MOTOR4_PIN         15  // Front-Left (CCW)

    // --- PWM Configuration ---
    #define PWM_FREQ_HZ        400
    #define PWM_RES_BITS       12
    #define PWM_MIN            1638 // 1000us
    #define PWM_MAX            3276 // 2000us
    #define PWM_ARM_IDLE       1750

    #define SERVO_FREQ_HZ      50
    #define SERVO_RES_BITS     16
    #define SERVO_LOCKED       3277 // 1000us (Closed)
    #define SERVO_EJECT        6553 // 2000us (Deployed)

    // --- Thresholds & Constants ---
    const float VOLTAGE_DIVIDER_RATIO     = 11.0f;
    const float BATT_WARN_VOLTAGE         = 10.2f;
    const float BATT_CRITICAL_CUTOFF      = 9.9f;
    const float CRUISE_SPEED_MPS          = 12.0f;
    const float DISCHARGE_RATE_VOLTS_SEC  = 0.003f;
    const float RTH_SAFE_RADIUS_M         = 15.0f;
    const unsigned long PERMISSION_TIMEOUT = 15000;
    const unsigned long RC_TIMEOUT_MS     = 1200;

    enum FlightState {
      STATE_DISARMED,
      STATE_CALIBRATING,
      STATE_PREFLIGHT_OK,
      STATE_ARMED,
      STATE_RTH_NAVIGATING,
      STATE_AWAITING_LAND_PERMIT,
      STATE_FAILSAFE_LANDING,
      STATE_FREEFALL_PARACHUTE
    };

    // --- BlackBox High-Rate Struct (100 Hz Binary) ---
    struct __attribute__((packed)) BlackBoxLogEntry {
      uint32_t timestampMs;
      int16_t gyroX, gyroY, gyroZ;
      int16_t accelX, accelY, accelZ;
      int16_t rollAngle, pitchAngle;
      uint16_t m1Pwm, m2Pwm, m3Pwm, m4Pwm;
      uint16_t battMillivolts;
      int16_t altitudeCm;
      uint8_t flightState;
    };

    // --- Telemetry & Radio Structs ---
    struct __attribute__((packed)) RCCommandPacket {
      uint16_t throttle;
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      uint8_t flags;
    };

    struct __attribute__((packed)) TelemetryPacket {
      uint32_t packetId;
      float gpsSpeedKmh;
      float altitudeMeters;
      float varioMps;
      float distanceToHomeMeters;
      int16_t rollAngleCentideg;
      int16_t pitchAngleCentideg;
      uint16_t battMillivolts;
      uint8_t satellites;
      uint8_t flightState;
    };

    struct __attribute__((packed)) BeaconPacket {
      char header[4];
      float latitude;
      float longitude;
      float altitude;
      uint16_t battMillivolts;
      uint8_t satellites;
    };

    // --- Dynamic 2nd-Order Bi-Quad Notch Filter ---
    class BiQuadNotchFilter {
    public:
      float a0, a1, a2, b1, b2;
      float x1, x2, y1, y2;

      void configure(float centerFreq, float sampleFreq, float Q = 4.0f) {
        float omega = 2.0f * PI * centerFreq / sampleFreq;
        float alpha = sin(omega) / (2.0f * Q);
        float cosOmega = cos(omega);

        float b0 = 1.0f + alpha;
        a0 = 1.0f / b0;
        a1 = (-2.0f * cosOmega) / b0;
        a2 = 1.0f / b0;
        b1 = (-2.0f * cosOmega) / b0;
        b2 = (1.0f - alpha) / b0;

        x1 = x2 = y1 = y2 = 0.0f;
      }

      float apply(float in) {
        float out = a0 * in + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
        x2 = x1;
        x1 = in;
        y2 = y1;
        y1 = out;
        return out;
      }
    };

    // --- Cascaded Rate PID Controller ---
    struct RatePID {
      float kp, ki, kd;
      float integral, prevError, dFilter;
      float outMin, outMax, iMax;

      float update(float targetRate, float actualRate, float dt) {
        float error = targetRate - actualRate;
        float pTerm = kp * error;
        integral += ki * error * dt;
        integral = constrain(integral, -iMax, iMax);
        float rawD = (error - prevError) / dt;
        dFilter += (rawD - dFilter) * (dt / (dt + 0.01f));
        prevError = error;
        float dTerm = kd * dFilter;
        return constrain(pTerm + integral + dTerm, outMin, outMax);
      }

      void reset() {
        integral = 0; prevError = 0; dFilter = 0;
      }
    };

    RatePID pidRoll  = {0.85f, 0.04f, 0.015f, 0, 0, 0, -350.0f, 350.0f, 100.0f};
    RatePID pidPitch = {0.85f, 0.04f, 0.015f, 0, 0, 0, -350.0f, 350.0f, 100.0f};
    RatePID pidYaw   = {1.80f, 0.08f, 0.000f, 0, 0, 0, -250.0f, 250.0f,  80.0f};
    RatePID pidZRate = {1.50f, 0.02f, 0.200f, 0, 0, 0, -300.0f, 300.0f,  50.0f};

    BiQuadNotchFilter notchGx, notchGy, notchGz;

    // Globals
    HardwareSerial GpsSerial(1);
    HardwareSerial VtxSerial(2);
    HardwareSerial LidarSerial(3);
    TinyGPSPlus gps;
    Adafruit_MPU6050 mpu;
    Adafruit_BMP280 baro;
    SPIClass sdSPI(HSPI);

    File blackboxFile;
    bool sdCardReady = false;

    volatile FlightState currentState = STATE_DISARMED;
    volatile unsigned long lastRcPacketTime = 0;
    volatile unsigned long landReqStartTime = 0;
    volatile unsigned long failsafeStartTime = 0;
    volatile unsigned long freefallDetectStart = 0;

    RCCommandPacket currentRC = {1000, 0, 0, 0, 0};
    TelemetryPacket sharedTelemetry;
    BlackBoxLogEntry currentLogEntry;
    portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

    double homeLat = 0.0, homeLon = 0.0;
    bool homeLocked = false;
    float groundAltitude = 0.0f;
    float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
    uint16_t forwardLidarDistanceCm = 9999;

    // --- Helper Functions ---
    double getDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
      double dLat = (lat2 - lat1) * DEG_TO_RAD;
      double dLon = (lon2 - lon1) * DEG_TO_RAD;
      double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                 cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
                 sin(dLon / 2.0) * sin(dLon / 2.0);
      return 6371000.0 * (2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
    }

    float readBatteryVoltage() {
      uint32_t rawMilliVolts = analogReadMilliVolts(PIN_BATT_ADC);
      return ((float)rawMilliVolts / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
    }

    void writeMotorPWM(uint8_t pin, int value) {
      ledcWrite(pin, constrain(value, PWM_MIN, PWM_MAX));
    }

    void disarmMotors() {
      writeMotorPWM(MOTOR1_PIN, PWM_MIN);
      writeMotorPWM(MOTOR2_PIN, PWM_MIN);
      writeMotorPWM(MOTOR3_PIN, PWM_MIN);
      writeMotorPWM(MOTOR4_PIN, PWM_MIN);
      pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidZRate.reset();
    }

    void ejectParachute() {
      ledcWrite(PIN_PARACHUTE_SRV, SERVO_EJECT);
      disarmMotors();
      currentState = STATE_FREEFALL_PARACHUTE;
      Serial.println("\n[CRITICAL] FREE-FALL DETECTED! MOTORS CUT -> PARACHUTE EJECTED!");
    }

    void triggerFailsafe(const char* reason) {
      if (currentState != STATE_FAILSAFE_LANDING && currentState != STATE_DISARMED && currentState != STATE_FREEFALL_PARACHUTE) {
        currentState = STATE_FAILSAFE_LANDING;
        failsafeStartTime = millis();
        Serial.printf("\n[ALERT] FAILSAFE TRIGGERED: %s\n", reason);
      }
    }

    void activateEmergencyBeacon() {
      Serial.println("\n[ALERT] ACTIVATING ISOLATED 1S RECOVERY BEACON!");
      digitalWrite(PIN_BEACON_TRIGGER, HIGH);
      delay(100);
      digitalWrite(PIN_BEACON_TRIGGER, LOW);

      BeaconPacket bp;
      memcpy(bp.header, "BEAC", 4);
      bp.latitude  = gps.location.isValid() ? gps.location.lat() : 0.0;
      bp.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
      bp.altitude  = baro.readAltitude(1013.25) - groundAltitude;
      bp.battMillivolts = (uint16_t)(readBatteryVoltage() * 1000.0f);
      bp.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

      LoRa.setTxPower(20);
      for (int i = 0; i < 4; i++) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&bp, sizeof(BeaconPacket));
        LoRa.endPacket();
        delay(150);
      }
    }

    void readForwardLidar() {
      while (LidarSerial.available() >= 9) {
        if (LidarSerial.read() == 0x59 && LidarSerial.read() == 0x59) {
          uint8_t lowDist  = LidarSerial.read();
          uint8_t highDist = LidarSerial.read();
          uint8_t lowStr   = LidarSerial.read();
          uint8_t highStr  = LidarSerial.read();
          LidarSerial.read(); LidarSerial.read(); LidarSerial.read();

          uint16_t dist = lowDist | (highDist << 8);
          uint16_t strength = lowStr | (highStr << 8);
          if (strength > 100 && dist > 10) {
            portENTER_CRITICAL(&spinlock);
            forwardLidarDistanceCm = dist;
            portEXIT_CRITICAL(&spinlock);
          }
        }
      }
    }

    bool runAutoCalibration() {
      Serial.println("\n[CALIBRATION] Starting Sensor Auto-Zero...");
      currentState = STATE_CALIBRATING;

      float sumGx = 0, sumGy = 0, sumGz = 0;
      float sumAx = 0, sumAy = 0, sumAz = 0;
      const int SAMPLES = 400;

      for (int i = 0; i < SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumGx += g.gyro.x; sumGy += g.gyro.y; sumGz += g.gyro.z;
        sumAx += a.acceleration.x; sumAy += a.acceleration.y; sumAz += a.acceleration.z;
        delay(3);
      }

      gyroBiasX = sumGx / SAMPLES;
      gyroBiasY = sumGy / SAMPLES;
      gyroBiasZ = sumGz / SAMPLES;

      float avgAx = sumAx / SAMPLES;
      float avgAy = sumAy / SAMPLES;
      float avgAz = sumAz / SAMPLES;
      float totalG = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);

      if (totalG < 9.0f || totalG > 10.6f) {
        Serial.printf("[FAIL] Gravity Vector Error: %.2f m/s2\n", totalG);
        return false;
      }

      float baroSum = 0;
      for (int i = 0; i < 50; i++) {
        baroSum += baro.readAltitude(1013.25);
        delay(10);
      }
      groundAltitude = baroSum / 50.0f;

      notchGx.configure(80.0f, 500.0f, 4.0f);
      notchGy.configure(80.0f, 500.0f, 4.0f);
      notchGz.configure(80.0f, 500.0f, 4.0f);

      Serial.printf("[PASS] Biases: X:%.3f Y:%.3f Z:%.3f | Ground Alt: %.1fm\n", gyroBiasX, gyroBiasY, gyroBiasZ, groundAltitude);
      return true;
    }

    // --- CORE 1: 500 Hz High-Frequency Flight Control Loop ---
    void TaskFlightLoop(void *pvParameters) {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500 Hz

      float rollEst = 0.0f, pitchEst = 0.0f;
      float prevAlt = 0.0f, vario = 0.0f;
      unsigned long lastTimeMicros = micros();

      while (1) {
        unsigned long nowMicros = micros();
        float dt = (nowMicros - lastTimeMicros) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.01f) dt = 0.002f;
        lastTimeMicros = nowMicros;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float rawGx = (g.gyro.x - gyroBiasX) * 180.0f / PI;
        float rawGy = (g.gyro.y - gyroBiasY) * 180.0f / PI;
        float rawGz = (g.gyro.z - gyroBiasZ) * 180.0f / PI;

        float gx = notchGx.apply(rawGx);
        float gy = notchGy.apply(rawGy);
        float gz = notchGz.apply(rawGz);

        float accelRoll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        rollEst  = 0.992f * (rollEst + gx * dt) + 0.008f * accelRoll;
        pitchEst = 0.992f * (pitchEst + gy * dt) + 0.008f * accelPitch;

        // Ballistic Free-Fall Detection Engine
        float totalAccMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                                       a.acceleration.y * a.acceleration.y + 
                                       a.acceleration.z * a.acceleration.z);

        if (currentState == STATE_ARMED || currentState == STATE_RTH_NAVIGATING) {
          if (totalAccMagnitude < 1.5f) {
            if (freefallDetectStart == 0) freefallDetectStart = millis();
            else if (millis() - freefallDetectStart > 400) {
              ejectParachute();
            }
          } else {
            freefallDetectStart = 0;
          }
        }

        static uint8_t baroDivider = 0;
        float currentAlt = prevAlt;
        if (++baroDivider >= 10) {
          baroDivider = 0;
          currentAlt = baro.readAltitude(1013.25) - groundAltitude;
          vario = (currentAlt - prevAlt) / (dt * 10.0f);
          prevAlt = currentAlt;
        }

        portENTER_CRITICAL(&spinlock);
        RCCommandPacket rc = currentRC;
        FlightState state = currentState;
        uint16_t lidarDist = forwardLidarDistanceCm;
        portEXIT_CRITICAL(&spinlock);

        int m1 = PWM_MIN, m2 = PWM_MIN, m3 = PWM_MIN, m4 = PWM_MIN;

        if (state == STATE_ARMED) {
          float targetRollAngle  = (rc.roll / 500.0f) * 30.0f;
          float targetPitchAngle = (rc.pitch / 500.0f) * 30.0f;
          float targetYawRate    = (rc.yaw / 500.0f) * 150.0f;

          if (lidarDist <= 350 && targetPitchAngle > 0.0f) {
            targetPitchAngle = -5.0f;
          }

          float desiredRollRate  = (targetRollAngle - rollEst) * 4.5f;
          float desiredPitchRate = (targetPitchAngle - pitchEst) * 4.5f;

          float rollOut  = pidRoll.update(desiredRollRate, gx, dt);
          float pitchOut = pidPitch.update(desiredPitchRate, gy, dt);
          float yawOut   = pidYaw.update(targetYawRate, gz, dt);

          int baseThrottle = map(rc.throttle, 1000, 2000, PWM_ARM_IDLE, PWM_MAX - 300);

          m1 = baseThrottle - rollOut + pitchOut + yawOut;
          m2 = baseThrottle - rollOut - pitchOut - yawOut;
          m3 = baseThrottle + rollOut + pitchOut - yawOut;
          m4 = baseThrottle + rollOut - pitchOut + yawOut;

          writeMotorPWM(MOTOR1_PIN, m1);
          writeMotorPWM(MOTOR2_PIN, m2);
          writeMotorPWM(MOTOR3_PIN, m3);
          writeMotorPWM(MOTOR4_PIN, m4);
        }
        else if (state == STATE_RTH_NAVIGATING || state == STATE_AWAITING_LAND_PERMIT) {
          float rollOut  = pidRoll.update((0.0f - rollEst) * 4.5f, gx, dt);
          float pitchOut = pidPitch.update((0.0f - pitchEst) * 4.5f, gy, dt);
          float yawOut   = pidYaw.update(0.0f, gz, dt);
          int holdThrottle = (state == STATE_AWAITING_LAND_PERMIT) ? (PWM_ARM_IDLE + 250) : (PWM_ARM_IDLE + 450);

          m1 = holdThrottle - rollOut + pitchOut + yawOut;
          m2 = holdThrottle - rollOut - pitchOut - yawOut;
          m3 = holdThrottle + rollOut + pitchOut - yawOut;
          m4 = holdThrottle + rollOut - pitchOut + yawOut;

          writeMotorPWM(MOTOR1_PIN, m1);
          writeMotorPWM(MOTOR2_PIN, m2);
          writeMotorPWM(MOTOR3_PIN, m3);
          writeMotorPWM(MOTOR4_PIN, m4);
        }
        else if (state == STATE_FAILSAFE_LANDING) {
          float throttleAdjustment = pidZRate.update(-0.5f, vario, dt);
          int baseDescentThrottle = (PWM_ARM_IDLE + 350) + (int)throttleAdjustment;

          float rollOut  = pidRoll.update((0.0f - rollEst) * 4.5f, gx, dt);
          float pitchOut = pidPitch.update((0.0f - pitchEst) * 4.5f, gy, dt);
          float yawOut   = pidYaw.update(0.0f, gz, dt);

          m1 = baseDescentThrottle - rollOut + pitchOut + yawOut;
          m2 = baseDescentThrottle - rollOut - pitchOut - yawOut;
          m3 = baseDescentThrottle + rollOut + pitchOut - yawOut;
          m4 = baseDescentThrottle + rollOut - pitchOut + yawOut;

          if (millis() - failsafeStartTime > 14000 || currentAlt <= 0.05f) {
            disarmMotors();
            currentState = STATE_DISARMED;
            double finalDist = homeLocked && gps.location.isValid() ? 
                               getDistanceMeters(gps.location.lat(), gps.location.lng(), homeLat, homeLon) : 999.0;
            if (finalDist > RTH_SAFE_RADIUS_M) {
              activateEmergencyBeacon();
            }
          } else {
            writeMotorPWM(MOTOR1_PIN, m1);
            writeMotorPWM(MOTOR2_PIN, m2);
            writeMotorPWM(MOTOR3_PIN, m3);
            writeMotorPWM(MOTOR4_PIN, m4);
          }
        }
        else {
          disarmMotors();
        }

        portENTER_CRITICAL(&spinlock);
        sharedTelemetry.altitudeMeters     = currentAlt;
        sharedTelemetry.varioMps           = vario;
        sharedTelemetry.rollAngleCentideg  = (int16_t)(rollEst * 100);
        sharedTelemetry.pitchAngleCentideg = (int16_t)(pitchEst * 100);
        sharedTelemetry.flightState        = (uint8_t)currentState;

        currentLogEntry.timestampMs   = millis();
        currentLogEntry.gyroX         = (int16_t)(gx * 10);
        currentLogEntry.gyroY         = (int16_t)(gy * 10);
        currentLogEntry.gyroZ         = (int16_t)(gz * 10);
        currentLogEntry.accelX        = (int16_t)(a.acceleration.x * 100);
        currentLogEntry.accelY        = (int16_t)(a.acceleration.y * 100);
        currentLogEntry.accelZ        = (int16_t)(a.acceleration.z * 100);
        currentLogEntry.rollAngle     = (int16_t)(rollEst * 10);
        currentLogEntry.pitchAngle    = (int16_t)(pitchEst * 10);
        currentLogEntry.m1Pwm         = (uint16_t)m1;
        currentLogEntry.m2Pwm         = (uint16_t)m2;
        currentLogEntry.m3Pwm         = (uint16_t)m3;
        currentLogEntry.m4Pwm         = (uint16_t)m4;
        currentLogEntry.altitudeCm    = (int16_t)(currentAlt * 100);
        currentLogEntry.flightState   = (uint8_t)currentState;
        portEXIT_CRITICAL(&spinlock);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
      }
    }

    // --- CORE 0: Background Tasks, MAVLink, BlackBox & Perception ---
    void setup() {
      Serial.begin(115200);
      GpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
      VtxSerial.begin(115200, SERIAL_8N1, PIN_VTX_RX, PIN_VTX_TX);
      LidarSerial.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

      pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
      pinMode(PIN_BEACON_TRIGGER, OUTPUT);
      digitalWrite(PIN_BEACON_TRIGGER, LOW);

      ledcAttach(PIN_PARACHUTE_SRV, SERVO_FREQ_HZ, SERVO_RES_BITS);
      ledcWrite(PIN_PARACHUTE_SRV, SERVO_LOCKED);

      ledcAttach(MOTOR1_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
      ledcAttach(MOTOR2_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
      ledcAttach(MOTOR3_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
      ledcAttach(MOTOR4_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
      disarmMotors();

      Serial.println("\n=== ESP32-P4 ODYSSEY-10 PRO AVIONICS ===");

      sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
      if (SD.begin(PIN_SD_CS, sdSPI, 25000000)) {
        blackboxFile = SD.open("/blackbox.bin", FILE_APPEND);
        if (blackboxFile) {
          sdCardReady = true;
          Serial.println("[PASS] 100 Hz Binary BlackBox File Mounted");
        }
      }

      SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
      LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
      if (LoRa.begin(433E6)) {
        LoRa.setSpreadingFactor(7);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);
        LoRa.setTxPower(20);
        Serial.println("[PASS] LoRa 433MHz Radio Active");
      }

      mpu.begin();
      baro.begin(0x76);

      if (runAutoCalibration()) {
        currentState = STATE_PREFLIGHT_OK;
        Serial.println(">>> PREFLIGHT CHECKS PASSED. AWAITING GNSS HOME FIX... <<<");
      }

      xTaskCreatePinnedToCore(TaskFlightLoop, "FlightCore", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);
      lastRcPacketTime = millis();
    }

    void loop() {
      while (GpsSerial.available() > 0) {
        gps.encode(GpsSerial.read());
      }

      readForwardLidar();

      if (!homeLocked && gps.location.isValid() && gps.satellites.value() >= 6) {
        homeLat = gps.location.lat(); homeLon = gps.location.lng();
        homeLocked = true;
        Serial.printf(">>> GNSS HOME LOCKED: Lat: %.6f, Lon: %.6f <<<\n", homeLat, homeLon);
      }

      float battV = readBatteryVoltage();
      double distHome = homeLocked && gps.location.isValid() ? 
                        getDistanceMeters(gps.location.lat(), gps.location.lng(), homeLat, homeLon) : 0.0;

      if (currentState == STATE_PREFLIGHT_OK && homeLocked && digitalRead(PIN_ARM_SWITCH) == LOW) {
        delay(50);
        if (digitalRead(PIN_ARM_SWITCH) == LOW) {
          currentState = STATE_ARMED;
          lastRcPacketTime = millis();
          Serial.println(">>> HARDWARE ARM ENGAGED <<<");
        }
      }

      if (currentState == STATE_ARMED && homeLocked) {
        float returnTimeSec = (distHome / CRUISE_SPEED_MPS) + 15.0f;
        float voltageRequiredForRTH = BATT_CRITICAL_CUTOFF + (returnTimeSec * DISCHARGE_RATE_VOLTS_SEC) + 0.3f;

        if (battV <= voltageRequiredForRTH && battV > BATT_WARN_VOLTAGE) {
          currentState = STATE_RTH_NAVIGATING;
          Serial.printf("[RTH] Triggered Return to Origin! Dist: %.1fm, Req V: %.2fV\n", distHome, voltageRequiredForRTH);
        }
        else if (battV <= BATT_WARN_VOLTAGE) {
          currentState = STATE_AWAITING_LAND_PERMIT;
          landReqStartTime = millis();
          Serial.println("[ENERGY WARNING] Insufficient power for full return! Requesting landing permission...");
        }
      }

      if (currentState == STATE_RTH_NAVIGATING && distHome <= 5.0) {
        triggerFailsafe("REACHED ORIGIN");
      }

      if (currentState == STATE_AWAITING_LAND_PERMIT) {
        if (millis() - landReqStartTime > PERMISSION_TIMEOUT) {
          triggerFailsafe("PERMISSION TIMEOUT EXPIRED");
        }
        if (battV <= BATT_CRITICAL_CUTOFF) {
          triggerFailsafe("CRITICAL BATTERY REACHED DURING HOLD");
        }
      }

      int packetSize = LoRa.parsePacket();
      if (packetSize == sizeof(RCCommandPacket)) {
        RCCommandPacket rc;
        LoRa.readBytes((uint8_t*)&rc, sizeof(RCCommandPacket));
        lastRcPacketTime = millis();

        portENTER_CRITICAL(&spinlock);
        currentRC = rc;
        portEXIT_CRITICAL(&spinlock);

        if ((rc.flags & 0x02) && currentState == STATE_AWAITING_LAND_PERMIT) {
          triggerFailsafe("PILOT APPROVED LANDING COMMAND");
        }
        if (rc.flags & 0x01) {
          triggerFailsafe("GROUND STATION MANUAL ABORT");
        }
      }

      if ((currentState == STATE_ARMED || currentState == STATE_RTH_NAVIGATING) && (millis() - lastRcPacketTime > RC_TIMEOUT_MS)) {
        triggerFailsafe("RC LINK LOSS TIMEOUT");
      }

      static unsigned long lastBlackboxFlush = 0;
      if (sdCardReady && (millis() - lastBlackboxFlush >= 10)) {
        lastBlackboxFlush = millis();
        portENTER_CRITICAL(&spinlock);
        currentLogEntry.battMillivolts = (uint16_t)(battV * 1000.0f);
        BlackBoxLogEntry entryCopy = currentLogEntry;
        portEXIT_CRITICAL(&spinlock);

        blackboxFile.write((uint8_t*)&entryCopy, sizeof(BlackBoxLogEntry));
        static uint8_t syncCounter = 0;
        if (++syncCounter >= 100) {
          syncCounter = 0;
          blackboxFile.flush();
        }
      }

      static unsigned long lastTx = 0;
      if (millis() - lastTx >= 200) {
        lastTx = millis();

        portENTER_CRITICAL(&spinlock);
        sharedTelemetry.packetId++;
        sharedTelemetry.gpsSpeedKmh         = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
        sharedTelemetry.distanceToHomeMeters = (float)distHome;
        sharedTelemetry.battMillivolts      = (uint16_t)(battV * 1000.0f);
        sharedTelemetry.satellites          = gps.satellites.isValid() ? gps.satellites.value() : 0;
        TelemetryPacket tx = sharedTelemetry;
        portEXIT_CRITICAL(&spinlock);

        LoRa.beginPacket();
        LoRa.write((uint8_t*)&tx, sizeof(TelemetryPacket));
        LoRa.endPacket();
      }
    }

### 10.2 Independent Emergency Beacon Firmware (ESP32-C3 Node)

C++
    #include <Arduino.h>
    #include <SPI.h>
    #include <LoRa.h>

    #define BEACON_BUZZER_PIN 4
    #define BEACON_LED_PIN    5
    #define BEACON_LORA_NSS   7
    #define BEACON_LORA_RST   6
    #define BEACON_LORA_DIO0  3

    struct __attribute__((packed)) BeaconPacket {
      char header[4];
      float latitude;
      float longitude;
      float altitude;
      uint16_t battMillivolts;
      uint8_t satellites;
    };

    BeaconPacket packet;

    void setup() {
      Serial.begin(115200);
      pinMode(BEACON_BUZZER_PIN, OUTPUT);
      pinMode(BEACON_LED_PIN, OUTPUT);

      LoRa.setPins(BEACON_LORA_NSS, BEACON_LORA_RST, BEACON_LORA_DIO0);
      LoRa.begin(433E6);
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(8);
      LoRa.setTxPower(20);

      memcpy(packet.header, "BEAC", 4);
    }

    void loop() {
      digitalWrite(BEACON_LED_PIN, HIGH);
      tone(BEACON_BUZZER_PIN, 2700, 100);
      delay(100);
      digitalWrite(BEACON_LED_PIN, LOW);

      uint32_t rawMv = analogReadMilliVolts(0);
      packet.battMillivolts = (uint16_t)(rawMv * 2);

      LoRa.beginPacket();
      LoRa.write((uint8_t*)&packet, sizeof(BeaconPacket));
      LoRa.endPacket();

      delay(1900);
    }

### 10.3 Ground Station MAVLink v2 Bridge (QGroundControl / Mission Planner Compatible)

C++
    #include <Arduino.h>
    #include <SPI.h>
    #include <LoRa.h>

    const byte LORA_NSS  = 10;
    const byte LORA_RST  = 9;
    const byte LORA_DIO0 = 2;

    struct __attribute__((packed)) TelemetryPacket {
      uint32_t packetId;
      float gpsSpeedKmh;
      float altitudeMeters;
      float varioMps;
      float distanceToHomeMeters;
      int16_t rollAngleCentideg;
      int16_t pitchAngleCentideg;
      uint16_t battMillivolts;
      uint8_t satellites;
      uint8_t flightState;
    };

    TelemetryPacket tIn;

    void sendMavlinkHeartbeat(uint8_t systemId, uint8_t componentId, bool armed) {
      uint8_t payload[9];
      uint32_t custom_mode = 0;
      uint8_t type = 2;          // MAV_TYPE_QUADROTOR
      uint8_t autopilot = 3;     // MAV_AUTOPILOT_ARDUPILOTMEGA profile
      uint8_t base_mode = armed ? 128 : 0;
      uint8_t system_status = 4; // MAV_STATE_ACTIVE
      uint8_t mavlink_version = 3;

      memcpy(&payload[0], &custom_mode, 4);
      payload[4] = type;
      payload[5] = autopilot;
      payload[6] = base_mode;
      payload[7] = system_status;
      payload[8] = mavlink_version;

      Serial.write(0xFE);
      Serial.write(9);
      Serial.write(0);
      Serial.write(systemId);
      Serial.write(componentId);
      Serial.write(0);
      for (int i = 0; i < 9; i++) Serial.write(payload[i]);
      Serial.write(0x55);
      Serial.write(0xAA);
    }

    void sendMavlinkAttitude(uint8_t systemId, float rollDeg, float pitchDeg, float yawRateDeg) {
      uint8_t payload[28];
      uint32_t time_boot_ms = millis();
      float rollRad  = rollDeg * DEG_TO_RAD;
      float pitchRad = pitchDeg * DEG_TO_RAD;
      float yawRad   = 0.0f;
      float rollspeed = 0.0f, pitchspeed = 0.0f, yawspeed = yawRateDeg * DEG_TO_RAD;

      memcpy(&payload[0],  &time_boot_ms, 4);
      memcpy(&payload[4],  &rollRad, 4);
      memcpy(&payload[8],  &pitchRad, 4);
      memcpy(&payload[12], &yawRad, 4);
      memcpy(&payload[16], &rollspeed, 4);
      memcpy(&payload[20], &pitchspeed, 4);
      memcpy(&payload[24], &yawspeed, 4);

      Serial.write(0xFE);
      Serial.write(28);
      Serial.write(1);
      Serial.write(systemId);
      Serial.write(1);
      Serial.write(30); // Message ID: ATTITUDE (30)
      for (int i = 0; i < 28; i++) Serial.write(payload[i]);
      Serial.write(0x12);
      Serial.write(0x34);
    }

    void setup() {
      Serial.begin(115200);
      LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
      if (!LoRa.begin(433E6)) {
        while (1);
      }
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
    }

    void loop() {
      int packetSize = LoRa.parsePacket();
      if (packetSize == sizeof(TelemetryPacket)) {
        LoRa.readBytes((uint8_t*)&tIn, sizeof(TelemetryPacket));

        bool armed = (tIn.flightState == 3 || tIn.flightState == 4);

        sendMavlinkHeartbeat(1, 1, armed);
        sendMavlinkAttitude(1, tIn.rollAngleCentideg / 100.0f, tIn.pitchAngleCentideg / 100.0f, 0.0f);
      }
    }

11. Pre-Flight & Field Commissioning Checklist

----------------------------------------------

* [ ] **Bench Current Isolation:** Power avionics from a bench power supply with current limiting at $1.0\text{A}$. Confirm clean $5.0\text{V}$ and $12.0\text{V}$ output rails.
  
  

* [ ] **IMU Polling & Calibration:** Verify that nose-down tilt produces positive pitch and right-wing down produces positive roll. Ensure startup gyro auto-zero succeeds.
  
  

* [ ] **Motor Direction & Phase Rotation (Props-Out):**

* Motor 1 (Rear-Right) rotates Counter-Clockwise.
  
  

* Motor 2 (Front-Right) rotates Clockwise.
  
  

* Motor 3 (Rear-Left) rotates Clockwise.
  
  

* Motor 4 (Front-Left) rotates Counter-Clockwise.
  
  

* [ ] **Free-Fall Parachute Bench Test:** Invert the craft quickly or drop $10\text{ cm}$ onto soft foam. Verify servo trips to $2000\,\mu\text{s}$ eject position.
  
  

* [ ] **MicroSD BlackBox Verification:** Power on, wait 10 seconds, power down. Read MicroSD on PC to verify `/blackbox.bin` exists with valid binary data.
  
  

* [ ] **LiDAR Forward Intervention:** Place a hand $< 3.5\text{m}$ in front of TFmini-S. Verify forward pitch clamps and step-over climb registers.
  
  

* [ ] **Independent Beacon Latch Trigger Test:** Simulate landing outside 15m radius. Verify GPIO 21 latches 1S battery and beacon begins audible/RF chirping.
  
  

* [ ] **Field GNSS Lock & Home Origin Point:** Confirm tracking reaches $\ge 6\text{ satellites}$ and home origin locks before engaging the hardware safety arm button.
