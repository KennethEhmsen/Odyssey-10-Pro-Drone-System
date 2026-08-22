# Vendored Arduino libraries

`platformio.ini` declares these as `lib_deps` and PlatformIO fetches them automatically.
ESP-IDF's component manager does not know about the Arduino library ecosystem, so each
one is vendored here with a generated `CMakeLists.txt` that presents it as a component.

| Component | Upstream | Pinned | `platformio.ini` constraint |
| --- | --- | --- | --- |
| `Adafruit_BusIO` | adafruit/Adafruit_BusIO | 1.17.4 | transitive dependency |
| `Adafruit_Unified_Sensor` | adafruit/Adafruit_Sensor | 1.1.15 | `^1.1.14` |
| `Adafruit_MPU6050` | adafruit/Adafruit_MPU6050 | 2.2.6 | `^2.2.6` |
| `Adafruit_BMP280` | adafruit/Adafruit_BMP280_Library | 2.6.8 | `^2.6.8` |
| `TinyGPSPlus` | mikalhart/TinyGPSPlus | v1.0.3a | `^1.0.3` |
| `LoRa` | sandeepmistry/arduino-LoRa | 0.8.0 | `^0.8.0` |

The versions are pinned to tags satisfying the same constraints as `platformio.ini`, so
the two build systems compile the same code rather than quietly diverging.

`.git` is removed from each so they are ordinary directories rather than something git
mistakes for a submodule.

## Why vendored rather than fetched

`sensors.cpp` reads the MPU-6050 through raw register access now (see §4.3.2 on the
IMU read split), but still uses the Adafruit driver for initialisation, and the BMP280
and GNSS drivers are used as-is. Replacing them with native IDF drivers is a larger
change with no bearing on what the bring-up is trying to verify.

## Updating one

Delete the directory, clone the new tag, remove `.git`, and restore its
`CMakeLists.txt`. Then update `platformio.ini` to match, or the two build systems stop
agreeing — which is the failure this table exists to prevent.
