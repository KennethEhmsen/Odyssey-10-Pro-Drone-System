// =====================================================================================
//  Odyssey-10 Pro -- Sensor hub implementation
// =====================================================================================

#include "sensors.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

SensorHub          sensors;
TinyGPSPlus        gnssParser;
SemaphoreHandle_t  i2cMutex = nullptr;

static Adafruit_MPU6050 mpu;
static Adafruit_BMP280  baro;

extern HardwareSerial GnssSerial;
extern HardwareSerial LidarSerial;

// Scoped I2C lock. Every register access in this file goes through one of these.
namespace {
struct I2cLock {
  bool held;
  explicit I2cLock(TickType_t wait = pdMS_TO_TICKS(10))
      : held(i2cMutex && xSemaphoreTake(i2cMutex, wait) == pdTRUE) {}
  ~I2cLock() { if (held) xSemaphoreGive(i2cMutex); }
};
} // namespace

// -------------------------------------------------------------------------------------
//  Register-level helpers
// -------------------------------------------------------------------------------------
bool SensorHub::i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool SensorHub::i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

bool SensorHub::i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// -------------------------------------------------------------------------------------
//  Bring-up
//
//  Every init() return value is checked and recorded. The original firmware called
//  mpu.begin() and baro.begin(0x76) and discarded both results, so a barometer that
//  failed to initialise still passed preflight and fed garbage into the altitude,
//  vario and landing-termination logic.
// -------------------------------------------------------------------------------------
bool SensorHub::begin() {
  if (!i2cMutex) i2cMutex = xSemaphoreCreateMutex();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_HZ);

  I2cLock lk(pdMS_TO_TICKS(1000));
  havePrimaryImu_ = initMpu6050();
  haveBackupImu_  = initIcm42688();
  haveBaro_       = initBmp280();
  haveMag_        = initQmc5883l();
  haveTof_        = initVl53l1x();
  haveCurrent_    = initIna226();

  Serial.printf("[SENSORS] IMU:%d BACKUP:%d BARO:%d MAG:%d TOF:%d INA:%d\n",
                havePrimaryImu_, haveBackupImu_, haveBaro_,
                haveMag_, haveTof_, haveCurrent_);

  // The primary IMU is the only sensor whose absence is unconditionally fatal.
  return havePrimaryImu_;
}

bool SensorHub::initMpu6050() {
  if (!mpu.begin(I2C_ADDR_MPU6050, &Wire)) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // The DLPF corner comes from FRAME_SIZE_IN, because it has to sit ABOVE the hover
  // fundamental the notch is aimed at and BELOW the loop's Nyquist. A 7-inch runs its
  // fundamental near 180 Hz, so a 94 Hz corner would attenuate the very peak the notch
  // is there to remove -- and add phase lag doing it.
#if IMU_DLPF_HZ >= 260
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
#elif IMU_DLPF_HZ >= 184
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
#else
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
#endif
  return true;
}

bool SensorHub::initIcm42688() {
  uint8_t who = 0;
  if (!i2cRead(I2C_ADDR_ICM42688, 0x75, &who, 1)) return false;
  if (who != 0x47) return false;                       // ICM-42688-P WHO_AM_I
  i2cWrite8(I2C_ADDR_ICM42688, 0x4E, 0x0F);            // PWR_MGMT0: gyro+accel low-noise
  delay(50);
  i2cWrite8(I2C_ADDR_ICM42688, 0x4F, 0x06);            // GYRO_CONFIG0: +/-500 dps, 1 kHz
  return true;
}

bool SensorHub::initBmp280() {
  if (!baro.begin(I2C_ADDR_BMP280)) return false;
  baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,     // temperature
                   Adafruit_BMP280::SAMPLING_X16,    // pressure
                   Adafruit_BMP280::FILTER_X16,
                   Adafruit_BMP280::STANDBY_MS_1);
  return true;
}

bool SensorHub::initQmc5883l() {
  if (!i2cProbe(I2C_ADDR_QMC5883L)) return false;
  i2cWrite8(I2C_ADDR_QMC5883L, 0x0B, 0x01);   // SET/RESET period
  // CTRL1: continuous mode, 200 Hz ODR, +/-8 G range, 512 OSR
  i2cWrite8(I2C_ADDR_QMC5883L, 0x09, 0x1D);
  return true;
}

bool SensorHub::initVl53l1x() {
  if (!i2cProbe(I2C_ADDR_VL53L1X)) return false;
  uint8_t id[2];
  if (!i2cRead(I2C_ADDR_VL53L1X, 0x010F, id, 2)) {
    // 16-bit register addressing; fall back to the two-byte form.
    Wire.beginTransmission(I2C_ADDR_VL53L1X);
    Wire.write(0x01); Wire.write(0x0F);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)I2C_ADDR_VL53L1X, 2) != 2) return false;
    id[0] = Wire.read(); id[1] = Wire.read();
  }
  if (id[0] != 0xEA) return false;             // model ID
  // Short distance mode, 20 ms timing budget -> comfortably faster than the 50 Hz we
  // poll it at, and short mode is far more robust in daylight than long mode.
  Wire.beginTransmission(I2C_ADDR_VL53L1X);
  Wire.write(0x00); Wire.write(0x87);          // SYSTEM__MODE_START = ranging
  Wire.endTransmission();
  return true;
}

bool SensorHub::initIna226() {
  uint8_t id[2];
  if (!i2cRead(I2C_ADDR_INA226, 0xFF, id, 2)) return false;
  if (!(id[0] == 0x22 && id[1] == 0x60)) return false;   // die ID 0x2260

  // CONFIG: averaging 16, bus and shunt conversion 1.1 ms, continuous.
  Wire.beginTransmission(I2C_ADDR_INA226);
  Wire.write(0x00); Wire.write(0x45); Wire.write(0x27);
  if (Wire.endTransmission() != 0) return false;

  // CALIBRATION = 0.00512 / (current_LSB * R_shunt). With a 1 mOhm shunt and an
  // 80 A full scale, current_LSB = 80/32768 = 2.44 mA.
  const float currentLsb = INA226_MAX_CURRENT_A / 32768.0f;
  const uint16_t cal = (uint16_t)(0.00512f / (currentLsb * INA226_SHUNT_OHMS));
  Wire.beginTransmission(I2C_ADDR_INA226);
  Wire.write(0x05); Wire.write((uint8_t)(cal >> 8)); Wire.write((uint8_t)(cal & 0xFF));
  return Wire.endTransmission() == 0;
}

// -------------------------------------------------------------------------------------
//  500 Hz primary IMU read
// -------------------------------------------------------------------------------------
bool SensorHub::readPrimaryImu(Vec3& gyroDps, Vec3& accelMps2) {
  if (!havePrimaryImu_) return false;

  I2cLock lk(pdMS_TO_TICKS(2));
  if (!lk.held) return false;          // bus busy; report a lost sample, not zeroes

  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) return false;

  const float radToDeg = 180.0f / (float)M_PI;
  gyroDps.x   = g.gyro.x * radToDeg;
  gyroDps.y   = g.gyro.y * radToDeg;
  gyroDps.z   = g.gyro.z * radToDeg;
  accelMps2.x = a.acceleration.x;
  accelMps2.y = a.acceleration.y;
  accelMps2.z = a.acceleration.z;

  primaryGyro_.set(gyroDps, millis());
  return true;
}

// -------------------------------------------------------------------------------------
//  50 Hz slow sensor service (core 0)
// -------------------------------------------------------------------------------------
void SensorHub::serviceSlowSensors(uint32_t nowMs) {
  // ---- Barometer -------------------------------------------------------------------
  if (haveBaro_) {
    I2cLock lk;
    if (lk.held) {
      const float alt = baro.readAltitude(1013.25f);
      // A BMP280 that has fallen off the bus returns implausible values rather than an
      // error, so sanity-bound the reading before it is allowed to become "fresh".
      if (isfinite(alt) && alt > -500.0f && alt < 9000.0f) {
        baroAltMsl_.set(alt, nowMs);
      }
    }
  }

  // ---- Downward ToF ----------------------------------------------------------------
  if (haveTof_) {
    I2cLock lk;
    uint16_t mm = 0;
    if (lk.held && readVl53l1x(mm)) {
      if (mm > 0 && mm < 4000) tofRangeM_.set((float)mm / 1000.0f, nowMs);
    }
  }

  // ---- Magnetometer ----------------------------------------------------------------
  if (haveMag_) {
    I2cLock lk;
    float mx, my, mz;
    if (lk.held && readQmc5883l(mx, my, mz)) {
      Vec3 m;
      m.x = (mx - magOffset_[0]) * magScale_[0];
      m.y = (my - magOffset_[1]) * magScale_[1];
      m.z = (mz - magOffset_[2]) * magScale_[2];
      magRaw_.set(m, nowMs);
    }
  }

  // ---- Backup IMU cross-check ------------------------------------------------------
  if (haveBackupImu_) {
    I2cLock lk;
    Vec3 g;
    if (lk.held && readIcm42688(g)) {
      backupGyro_.set(g, nowMs);
      if (primaryGyro_.isFresh(nowMs, IMU_MAX_AGE_MS)) {
        const Vec3& p = primaryGyro_.value;
        // 25 deg/s of disagreement on any axis is far outside sensor noise and
        // mounting misalignment; it means one of the two has failed.
        imuDisagree_ = fabsf(p.x - g.x) > 25.0f
                    || fabsf(p.y - g.y) > 25.0f
                    || fabsf(p.z - g.z) > 25.0f;
      }
    }
  }

  // ---- Battery: voltage + coulomb counting -----------------------------------------
  {
    uint32_t accum = 0;
    for (int i = 0; i < BATT_ADC_SAMPLES; ++i) accum += analogReadMilliVolts(PIN_BATT_ADC);
    const float rawV = ((float)accum / (float)BATT_ADC_SAMPLES / 1000.0f)
                     * VOLTAGE_DIVIDER_RATIO * VOLTAGE_DIVIDER_TRIM;
    battery_.packVoltsRaw = rawV;

    static SlewLimitedEma vFilter(BATT_FILTER_ALPHA, 0.50f);
    if (!vFilter.primed()) vFilter.reset(rawV);
    battery_.packVolts = vFilter.apply(rawV);

    if (haveCurrent_) {
      I2cLock lk;
      float busV, amps;
      if (lk.held && readIna226(busV, amps)) {
        currentA_.set(amps, nowMs);
        battery_.currentAmps     = amps;
        battery_.currentSensorOk = true;

        if (lastCoulombMs_ != 0) {
          const float dtHours = (float)(nowMs - lastCoulombMs_) / 3600000.0f;
          battery_.consumedMah += amps * 1000.0f * dtHours;
        }
        lastCoulombMs_ = nowMs;
      }
    } else {
      battery_.currentSensorOk = false;
    }

    // Remaining capacity: prefer the coulomb count, fall back to a voltage curve.
    if (battery_.currentSensorOk) {
      const float used = battery_.consumedMah / PACK_USABLE_MAH;
      battery_.remainingPercent = constrain((1.0f - used) * 100.0f, 0.0f, 100.0f);
    } else {
      const float span = PACK_FULL_V - PACK_CRITICAL_V;
      battery_.remainingPercent =
          constrain((battery_.packVolts - PACK_CRITICAL_V) / span * 100.0f, 0.0f, 100.0f);
    }
  }
}

bool SensorHub::readIcm42688(Vec3& gyroDps) {
  uint8_t b[6];
  if (!i2cRead(I2C_ADDR_ICM42688, 0x25, b, 6)) return false;   // GYRO_DATA_X1
  const float scale = 500.0f / 32768.0f;                        // +/-500 dps
  gyroDps.x = (int16_t)((b[0] << 8) | b[1]) * scale;
  gyroDps.y = (int16_t)((b[2] << 8) | b[3]) * scale;
  gyroDps.z = (int16_t)((b[4] << 8) | b[5]) * scale;
  return true;
}

bool SensorHub::readQmc5883l(float& mx, float& my, float& mz) {
  uint8_t status = 0;
  if (!i2cRead(I2C_ADDR_QMC5883L, 0x06, &status, 1)) return false;
  if (!(status & 0x01)) return false;          // DRDY not set: no new sample
  uint8_t b[6];
  if (!i2cRead(I2C_ADDR_QMC5883L, 0x00, b, 6)) return false;
  mx = (float)(int16_t)(b[0] | (b[1] << 8));
  my = (float)(int16_t)(b[2] | (b[3] << 8));
  mz = (float)(int16_t)(b[4] | (b[5] << 8));
  return true;
}

bool SensorHub::readIna226(float& busVolts, float& amps) {
  uint8_t b[2];
  if (!i2cRead(I2C_ADDR_INA226, 0x02, b, 2)) return false;      // BUS_VOLTAGE
  busVolts = (float)(uint16_t)((b[0] << 8) | b[1]) * 0.00125f;  // 1.25 mV/LSB
  if (!i2cRead(I2C_ADDR_INA226, 0x04, b, 2)) return false;      // CURRENT
  const float currentLsb = INA226_MAX_CURRENT_A / 32768.0f;
  amps = (float)(int16_t)((b[0] << 8) | b[1]) * currentLsb;
  return true;
}

bool SensorHub::readVl53l1x(uint16_t& mm) {
  // GPIO__TIO_HV_STATUS (0x0031): bit 0 clear means a measurement is ready.
  Wire.beginTransmission(I2C_ADDR_VL53L1X);
  Wire.write(0x00); Wire.write(0x31);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)I2C_ADDR_VL53L1X, 1) != 1) return false;
  if ((Wire.read() & 0x01) != 0) return false;

  // RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 (0x0096)
  Wire.beginTransmission(I2C_ADDR_VL53L1X);
  Wire.write(0x00); Wire.write(0x96);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)I2C_ADDR_VL53L1X, 2) != 2) return false;
  const uint8_t hi = Wire.read(), lo = Wire.read();
  mm = (uint16_t)((hi << 8) | lo);

  // Clear the interrupt so the next measurement can be flagged.
  Wire.beginTransmission(I2C_ADDR_VL53L1X);
  Wire.write(0x00); Wire.write(0x86); Wire.write(0x01);
  Wire.endTransmission();
  return true;
}

// -------------------------------------------------------------------------------------
//  GNSS
//
//  FIX FOR FINDING 16. Section 9 of the specification documented the BN-220 at
//  115200 baud; the firmware opened the port at 9600. A builder who followed the
//  documentation and reconfigured the module got no fix, no home lock, and therefore
//  no arm -- with nothing on the console to explain it.
//
//  The link is now negotiated: probe the documented rate first, fall back to the
//  factory rate, then push the module to 115200 / 10 Hz and confirm the change stuck.
// -------------------------------------------------------------------------------------
uint32_t SensorHub::configureGnssLink() {
  const uint32_t candidates[2] = { GNSS_TARGET_BAUD, GNSS_FALLBACK_BAUD };

  for (int attempt = 0; attempt < 2; ++attempt) {
    const uint32_t baud = candidates[attempt];
    GnssSerial.begin(baud, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    delay(120);
    while (GnssSerial.available()) GnssSerial.read();     // discard partial sentences

    const uint32_t deadline = millis() + GNSS_PROBE_TIMEOUT_MS;
    uint32_t goodSentences = 0;
    while (millis() < deadline && goodSentences < 3) {
      while (GnssSerial.available()) {
        if (gnssParser.encode(GnssSerial.read())) {
          if (gnssParser.sentencesWithFix() > 0 || gnssParser.charsProcessed() > 200) {
            ++goodSentences;
          }
        }
      }
      delay(5);
    }

    if (goodSentences >= 3) {
      Serial.printf("[GNSS] module answered at %lu baud\n", (unsigned long)baud);
      if (baud != GNSS_TARGET_BAUD) {
        Serial.printf("[GNSS] switching to %d baud\n", GNSS_TARGET_BAUD);
        sendUbxCfgPrt(GNSS_TARGET_BAUD);
        GnssSerial.flush();
        delay(150);
        GnssSerial.begin(GNSS_TARGET_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
        delay(150);
      }
      sendUbxCfgRate(1000 / GNSS_TARGET_RATE_HZ);
      return GNSS_TARGET_BAUD;
    }
    Serial.printf("[GNSS] no response at %lu baud\n", (unsigned long)baud);
  }

  // Leave the port open at the documented rate so a module that comes alive late is
  // still picked up. healthMask() will keep reporting GNSS as failed until it does.
  GnssSerial.begin(GNSS_TARGET_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  Serial.println("[GNSS] FAILED -- no NMEA at either rate; arming will be blocked");
  return 0;
}

static void ubxChecksum(uint8_t* msg, size_t len) {
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < len - 2; ++i) { a += msg[i]; b += a; }
  msg[len - 2] = a;
  msg[len - 1] = b;
}

void SensorHub::sendUbxCfgPrt(uint32_t baud) {
  uint8_t m[28] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00,              // portID 1 (UART1), reserved
    0xD0, 0x08, 0x00, 0x00,              // mode: 8N1
    0x00, 0x00, 0x00, 0x00,              // baudRate, filled below
    0x07, 0x00,                          // inProtoMask:  UBX + NMEA + RTCM
    0x03, 0x00,                          // outProtoMask: UBX + NMEA
    0x00, 0x00, 0x00, 0x00,              // flags, reserved
    0x00, 0x00                           // checksum
  };
  m[14] = (uint8_t)(baud      );
  m[15] = (uint8_t)(baud >>  8);
  m[16] = (uint8_t)(baud >> 16);
  m[17] = (uint8_t)(baud >> 24);
  ubxChecksum(m, sizeof(m));
  GnssSerial.write(m, sizeof(m));
}

void SensorHub::sendUbxCfgRate(uint16_t measRateMs) {
  uint8_t m[14] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0x00, 0x00,          // measRate, filled below
    0x01, 0x00,          // navRate: one solution per measurement
    0x01, 0x00,          // timeRef: GPS time
    0x00, 0x00
  };
  m[6] = (uint8_t)(measRateMs & 0xFF);
  m[7] = (uint8_t)(measRateMs >> 8);
  ubxChecksum(m, sizeof(m));
  GnssSerial.write(m, sizeof(m));
}

void SensorHub::serviceGnss(uint32_t nowMs) {
  while (GnssSerial.available()) gnssParser.encode(GnssSerial.read());

  if (gnssParser.location.isValid() && gnssParser.location.age() < GNSS_MAX_AGE_MS) {
    GnssFix f;
    f.latitude    = gnssParser.location.lat();
    f.longitude   = gnssParser.location.lng();
    f.altitudeMsl = gnssParser.altitude.isValid()
                      ? (float)gnssParser.altitude.meters() : 0.0f;
    f.groundSpeed = gnssParser.speed.isValid()
                      ? (float)gnssParser.speed.mps() : 0.0f;
    f.courseDeg   = gnssParser.course.isValid()
                      ? (float)gnssParser.course.deg() : 0.0f;
    f.satellites  = gnssParser.satellites.isValid()
                      ? (uint8_t)gnssParser.satellites.value() : 0;
    f.fixQuality  = f.satellites >= 6 ? 2 : (f.satellites >= 4 ? 1 : 0);
    f.valid       = true;
    gnss_.set(f, nowMs);
  }
}

// -------------------------------------------------------------------------------------
//  Forward LiDAR (TFmini-S)
//
//  FIX FOR FINDING 15, part one. The frame parser is now a proper byte-at-a-time state
//  machine with checksum validation, and every accepted reading is timestamped so
//  forwardObstacle() can refuse to hand back a stale distance.
// -------------------------------------------------------------------------------------
void SensorHub::serviceLidar(uint32_t nowMs) {
  while (LidarSerial.available()) {
    const uint8_t byteIn = (uint8_t)LidarSerial.read();

    // Resynchronise on the 0x59 0x59 header rather than blindly consuming nine bytes.
    if (lidarIdx_ == 0) {
      if (byteIn != 0x59) continue;
      lidarBuf_[lidarIdx_++] = byteIn;
      continue;
    }
    if (lidarIdx_ == 1 && byteIn != 0x59) {
      lidarIdx_ = (byteIn == 0x59) ? 1 : 0;
      continue;
    }

    lidarBuf_[lidarIdx_++] = byteIn;
    if (lidarIdx_ < 9) continue;
    lidarIdx_ = 0;

    uint8_t sum = 0;
    for (int i = 0; i < 8; ++i) sum += lidarBuf_[i];
    if (sum != lidarBuf_[8]) continue;                 // corrupt frame, drop it

    const uint16_t dist     = (uint16_t)(lidarBuf_[2] | (lidarBuf_[3] << 8));
    const uint16_t strength = (uint16_t)(lidarBuf_[4] | (lidarBuf_[5] << 8));

    // The TFmini-S signals a bad measurement with strength 0xFFFF or a very low
    // return. Both mean "no confident reading", which is NOT the same as "clear".
    if (strength < 100 || strength == 0xFFFF) continue;
    if (dist < 10 || dist > 1200) continue;            // outside the 0.1-12 m envelope

    lidarCm_.set(dist, nowMs);
  }
}

// -------------------------------------------------------------------------------------
//  Calibration
// -------------------------------------------------------------------------------------
CalibrationResult SensorHub::calibrate() {
  CalibrationResult r;

  if (!havePrimaryImu_) {
    strncpy(r.failure, "primary IMU not present", sizeof(r.failure) - 1);
    return r;
  }

  const int SAMPLES = 400;
  double sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
  int    taken = 0;

  for (int i = 0; i < SAMPLES; ++i) {
    Vec3 g, a;
    if (readPrimaryImu(g, a)) {
      sg[0] += g.x; sg[1] += g.y; sg[2] += g.z;
      sa[0] += a.x; sa[1] += a.y; sa[2] += a.z;
      ++taken;
    }
    delay(3);
  }

  if (taken < SAMPLES / 2) {
    snprintf(r.failure, sizeof(r.failure), "only %d/%d IMU samples read", taken, SAMPLES);
    return r;
  }

  for (int i = 0; i < 3; ++i) r.gyroBias[i] = (float)(sg[i] / taken);

  const float ax = (float)(sa[0] / taken);
  const float ay = (float)(sa[1] / taken);
  const float az = (float)(sa[2] / taken);
  r.gravityMagnitude = sqrtf(ax * ax + ay * ay + az * az);

  if (r.gravityMagnitude < 9.0f || r.gravityMagnitude > 10.6f) {
    snprintf(r.failure, sizeof(r.failure), "gravity %.2f m/s2 out of range",
             r.gravityMagnitude);
    return r;
  }

  // Gyro bias itself must be small. A large bias on a stationary airframe means the
  // aircraft was moved during calibration or the sensor is faulty.
  for (int i = 0; i < 3; ++i) {
    if (fabsf(r.gyroBias[i]) > 15.0f) {
      snprintf(r.failure, sizeof(r.failure), "gyro bias axis %d = %.1f dps", i,
               r.gyroBias[i]);
      return r;
    }
  }

  if (haveBaro_) {
    double sum = 0; int n = 0;
    for (int i = 0; i < 60; ++i) {
      I2cLock lk;
      if (lk.held) {
        const float alt = baro.readAltitude(1013.25f);
        if (isfinite(alt)) { sum += alt; ++n; }
      }
      delay(10);
    }
    if (n < 30) {
      strncpy(r.failure, "barometer did not respond", sizeof(r.failure) - 1);
      return r;
    }
    r.groundAltitudeM = (float)(sum / n);
    groundReferenceM_ = r.groundAltitudeM;
    groundRefLatched_ = true;
  } else {
    strncpy(r.failure, "barometer not present", sizeof(r.failure) - 1);
    return r;
  }

  if (!haveMag_) {
    strncpy(r.failure, "magnetometer not present (RTH needs heading)",
            sizeof(r.failure) - 1);
    return r;
  }

  r.passed = true;
  return r;
}

void SensorHub::latchGroundReference() {
  const uint32_t now = millis();
  if (baroAltMsl_.isFresh(now, 1000)) {
    groundReferenceM_ = baroAltMsl_.value;
    groundRefLatched_ = true;
  }
  resetCoulombCount();
  lastCoulombMs_ = now;
}

// -------------------------------------------------------------------------------------
//  Age-checked accessors
// -------------------------------------------------------------------------------------
bool SensorHub::baroAgl(uint32_t nowMs, float& outM) const {
  if (!groundRefLatched_) return false;
  if (!baroAltMsl_.isFresh(nowMs, 500)) return false;
  outM = baroAltMsl_.value - groundReferenceM_;
  return true;
}

bool SensorHub::tofAgl(uint32_t nowMs, float& outM) const {
  if (!tofRangeM_.isFresh(nowMs, TOF_MAX_AGE_MS)) return false;
  outM = tofRangeM_.value;
  return true;
}

bool SensorHub::forwardObstacle(uint32_t nowMs, uint16_t& outCm) const {
  if (!lidarCm_.isFresh(nowMs, LIDAR_MAX_AGE_MS)) return false;
  outCm = lidarCm_.value;
  return true;
}

bool SensorHub::gnssFix(uint32_t nowMs, GnssFix& out) const {
  if (!gnss_.isFresh(nowMs, GNSS_MAX_AGE_MS)) return false;
  out = gnss_.value;
  return true;
}

// Tilt-compensated heading. Without the roll/pitch compensation a 20-degree bank
// swings the reported heading by tens of degrees, which would make the RTH controller
// chase its own attitude.
bool SensorHub::magneticHeading(uint32_t nowMs, float rollDeg, float pitchDeg,
                                float& outDeg) const {
  if (!magRaw_.isFresh(nowMs, MAG_MAX_AGE_MS)) return false;

  const float r = rollDeg  * (float)M_PI / 180.0f;
  const float p = pitchDeg * (float)M_PI / 180.0f;
  const Vec3& m = magRaw_.value;

  const float xh = m.x * cosf(p) + m.z * sinf(p);
  const float yh = m.x * sinf(r) * sinf(p) + m.y * cosf(r) - m.z * sinf(r) * cosf(p);

  float heading = atan2f(-yh, xh) * 180.0f / (float)M_PI;
  if (heading < 0.0f)    heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  outDeg = heading;
  return true;
}

uint16_t SensorHub::healthMask(uint32_t nowMs) const {
  uint16_t m = 0;
  if (havePrimaryImu_ && primaryGyro_.isFresh(nowMs, IMU_MAX_AGE_MS) && !imuDisagree_)
    m |= ODY_SENS_IMU_PRIMARY;
  if (haveBackupImu_ && backupGyro_.isFresh(nowMs, 200))       m |= ODY_SENS_IMU_BACKUP;
  if (haveBaro_ && baroAltMsl_.isFresh(nowMs, 500))            m |= ODY_SENS_BARO;
  if (haveMag_ && magRaw_.isFresh(nowMs, MAG_MAX_AGE_MS))      m |= ODY_SENS_MAG;
  if (gnss_.isFresh(nowMs, GNSS_MAX_AGE_MS) &&
      gnss_.value.satellites >= ARM_MIN_SATELLITES)            m |= ODY_SENS_GNSS;
  if (lidarCm_.isFresh(nowMs, LIDAR_MAX_AGE_MS))               m |= ODY_SENS_LIDAR_FWD;
  if (tofRangeM_.isFresh(nowMs, TOF_MAX_AGE_MS))               m |= ODY_SENS_TOF_DOWN;
  if (haveCurrent_ && currentA_.isFresh(nowMs, CURRENT_MAX_AGE_MS))
    m |= ODY_SENS_CURRENT;
  return m;
}
