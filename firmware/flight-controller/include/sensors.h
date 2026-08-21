// =====================================================================================
//  Odyssey-10 Pro -- Sensor hub
//  ------------------------------------------------------------------------------------
//  FIXES FINDINGS 14, 15 AND 16, and implements the three sensors that the original
//  specification listed in the BOM and the architecture diagram but never referenced
//  anywhere in the firmware:
//
//    * VL53L1X downward ToF  -- precision landing flare and, critically, the veto on
//                               the touchdown disarm that finding 14 is about
//    * INA226 current shunt  -- coulomb counting, so the energy budget does not depend
//                               entirely on a voltage reading that sags under throttle
//    * QMC5883L magnetometer -- absolute heading, without which the RTH controller in
//                               navigation.h has no way to know which way it is facing
//
//  The ICM-42688-P backup IMU is also read and cross-checked against the primary,
//  which is what the "sensor fault detection & voting redundancy" line in the BOM
//  promised.
//
//  Bus discipline: the MPU-6050 is the only device the 500 Hz flight task touches.
//  Every other I2C device is polled from the 50 Hz telemetry task on core 0, and all
//  I2C traffic is serialised behind i2cMutex so the two cores cannot interleave
//  transactions on the shared bus.
// =====================================================================================

#ifndef ODY_SENSORS_H
#define ODY_SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "config.h"
#include "types.h"

struct CalibrationResult {
  bool  passed          = false;
  float gyroBias[3]     = {0, 0, 0};
  float gravityMagnitude = 0.0f;
  float groundAltitudeM  = 0.0f;
  float magDeclinationDeg = 0.0f;
  char  failure[64]      = {0};
};

class SensorHub {
public:
  bool begin();

  // --- 500 Hz path (core 1) --------------------------------------------------------
  // Reads the primary IMU only. Returns false if the read failed, which the flight
  // loop treats as a lost sample rather than as zeroes.
  bool readPrimaryImu(Vec3& gyroDps, Vec3& accelMps2);

  // --- 50 Hz path (core 0) ---------------------------------------------------------
  void serviceSlowSensors(uint32_t nowMs);
  void serviceGnss(uint32_t nowMs);
  void serviceLidar(uint32_t nowMs);

  CalibrationResult calibrate();

  // --- Accessors -------------------------------------------------------------------
  // Every one of these is age-checked. There is no way to read a stale value by
  // accident, which is the whole point of finding 15's fix.
  bool     baroAgl(uint32_t nowMs, float& outM) const;
  bool     tofAgl(uint32_t nowMs, float& outM) const;
  bool     forwardObstacle(uint32_t nowMs, uint16_t& outCm) const;
  bool     magneticHeading(uint32_t nowMs, float roll, float pitch, float& outDeg) const;
  bool     gnssFix(uint32_t nowMs, GnssFix& out) const;
  const BatteryState& battery() const { return battery_; }

  // Composite health mask, recomputed every telemetry tick.
  uint16_t healthMask(uint32_t nowMs) const;

  // Reference altitude captured at arm time, so baro AGL is relative to the launch
  // point of THIS flight rather than to wherever the aircraft was powered on.
  void     latchGroundReference();
  float    groundReferenceM() const { return groundReferenceM_; }

  float    consumedMah() const { return battery_.consumedMah; }
  void     resetCoulombCount() { battery_.consumedMah = 0.0f; }

  // Redundancy: true when the primary and backup gyros disagree beyond tolerance,
  // which is a strong indication that one of them has failed.
  bool     imuDisagreement() const { return imuDisagree_; }

  // GNSS baud negotiation. Returns the rate the module actually answered on.
  uint32_t configureGnssLink();

private:
  // Low-level register helpers, all taken behind i2cMutex_.
  bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val);
  bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);
  bool i2cProbe(uint8_t addr);

  bool initMpu6050();
  bool initIcm42688();
  bool initBmp280();
  bool initQmc5883l();
  bool initVl53l1x();
  bool initIna226();

  bool readIcm42688(Vec3& gyroDps);
  bool readQmc5883l(float& mx, float& my, float& mz);
  bool readIna226(float& busVolts, float& amps);
  bool readVl53l1x(uint16_t& mm);

  void sendUbxCfgRate(uint16_t measRateMs);
  void sendUbxCfgPrt(uint32_t baud);

  // Sensor presence, established once at boot.
  bool havePrimaryImu_ = false;
  bool haveBackupImu_  = false;
  bool haveBaro_       = false;
  bool haveMag_        = false;
  bool haveTof_        = false;
  bool haveCurrent_    = false;

  // Timestamped readings.
  TimedValue<float>    baroAltMsl_;
  TimedValue<float>    tofRangeM_;
  TimedValue<uint16_t> lidarCm_;
  TimedValue<Vec3>     magRaw_;
  TimedValue<GnssFix>  gnss_;
  TimedValue<float>    currentA_;
  TimedValue<Vec3>     backupGyro_;
  TimedValue<Vec3>     primaryGyro_;
  // The accelerometer is read at IMU_ACCEL_READ_HZ rather than at the loop rate, so it
  // carries a timestamp like every other slower-than-loop sensor in this file. Nothing
  // should use it without knowing how old it is.
  TimedValue<Vec3>     primaryAccel_;
  uint16_t             accelDivider_ = 0;

  BatteryState battery_;
  float        groundReferenceM_ = 0.0f;
  bool         groundRefLatched_ = false;
  bool         imuDisagree_      = false;
  uint32_t     lastCoulombMs_    = 0;

  // Hard-iron offsets from the compass calibration in docs section 11.
  float magOffset_[3] = {0.0f, 0.0f, 0.0f};
  float magScale_[3]  = {1.0f, 1.0f, 1.0f};

  // TFmini-S frame assembly state, kept across calls so a frame split across two
  // service ticks is not dropped.
  uint8_t lidarBuf_[9];
  uint8_t lidarIdx_ = 0;
};

extern SensorHub    sensors;
extern TinyGPSPlus  gnssParser;
extern SemaphoreHandle_t i2cMutex;

#endif // ODY_SENSORS_H
