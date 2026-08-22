# The sources stay where they are. This project predates the ESP-IDF build and its
# layout is documented in section 10.1; moving every file to satisfy a build system
# would churn the repository and every path in the specification with it.
idf_component_register(
    SRCS
        "../src/main.cpp"
        "../src/sensors.cpp"
        "../src/navigation.cpp"
        "../src/radio_link.cpp"
        "../src/blackbox.cpp"
        "../src/dshot_rmt.cpp"
    INCLUDE_DIRS
        "../include"
        "../../../shared"
    REQUIRES
        driver
        esp_timer
        esp_adc
        nvs_flash
        espressif__arduino-esp32
        # An IDF component's INCLUDE_DIRS reach only components that REQUIRE it. Leaving
        # these out does not warn -- the headers simply are not found, which reads like a
        # missing library rather than a missing declaration.
        Adafruit_BusIO
        Adafruit_Unified_Sensor
        Adafruit_MPU6050
        Adafruit_BMP280
        TinyGPSPlus
        LoRa
)
