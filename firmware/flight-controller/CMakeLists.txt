# =====================================================================================
#  Odyssey-10 Pro -- Flight Controller, ESP-IDF build
#  ------------------------------------------------------------------------------------
#  WHY THIS EXISTS ALONGSIDE platformio.ini
#
#  PlatformIO's stock espressif32 platform does not know the ESP32-P4: it lists 242
#  boards and esp32-p4-function-ev-board is not among them, so `pio run -e odyssey-fc`
#  fails resolving the board before it reaches a compiler. See section 3.2.
#
#  The alternatives were a community fork of the platform (pioarduino) or ESP-IDF, which
#  supports the P4 directly. ESP-IDF was chosen: official, and it is the toolchain
#  dshot_rmt.cpp was written against in the first place.
#
#  platformio.ini is kept rather than deleted. It documents the library versions and the
#  warning flags this project builds with, and it becomes usable again the moment the
#  stock platform gains P4 support.
# =====================================================================================

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#  ARDUINO'S Serial IS NOT THE IDF CONSOLE, AND DEFAULTS TO UART0.
#
#  HardwareSerial.h resolves `Serial` at compile time:
#
#      ARDUINO_USB_CDC_ON_BOOT=0   ->  #define Serial Serial0     (UART0 pins)
#      =1 with ARDUINO_USB_MODE=1  ->  #define Serial HWCDCSerial (USB Serial/JTAG)
#
#  Neither is set by default, so every Serial.printf in this firmware went to a physical
#  header with nothing connected. Routing the IDF console to USB does not help: ESP_LOG
#  and Arduino's Serial are different objects, and this firmware logs only through the
#  latter. The symptom is a port that opens, stays open and delivers zero bytes, which
#  is indistinguishable from a crashed board.
#
#  These must be set for the WHOLE BUILD, not just for main. HWCDCSerial is instantiated
#  inside the arduino component behind the same guard, so defining them locally makes our
#  code reference an object that was never compiled -- an undefined reference at link.
#  TEMPORARILY DISABLED FOR BRING-UP. With these set the application owns the USB
#  Serial/JTAG peripheral, so a fault in setup() takes USB down with it and the
#  panic handler has nowhere to print. Left off until the firmware is known to
#  boot; the IDF console stays on USB either way, so the bootloader log and any
#  backtrace still arrive.
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_CDC_ON_BOOT=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_MODE=1" APPEND)

#  BENCH PIN REMAP FOR THE ESP32-P4-FUNCTION-EV-BOARD.
#
#  This board carries an ESP32-C6 as its Wi-Fi co-processor, reached over SDIO on GPIO
#  14-19, and ESP-Hosted starts that driver before app_main:
#
#      I (132) H_SDIO_DRV: sdio_data_to_rx_buf_task started
#
#  Four of this project's pins sit on that bus. setup() calls ledcAttach on GPIO 15 and
#  opens UARTs on 17, 18 and 19, tearing the SDIO pins out from under a live driver. The
#  firmware dies before printing a character, and because it owns the USB console by
#  then, the port vanishes with it -- which looks exactly like a board that will not boot.
#
#  On the AIRCRAFT these pins are correct: its flight controller is a P4 module with no
#  C6 on SDIO. Section 9's pinout is not wrong, it is wrong FOR THIS BOARD. So the remap
#  lives here, in the bench build, and not in config.h.
idf_build_set_property(COMPILE_DEFINITIONS "MOTOR4_PIN=32" APPEND)    # was 15, SDIO D1
idf_build_set_property(COMPILE_DEFINITIONS "PIN_GNSS_RX=29" APPEND)   # was 17, SDIO D3
idf_build_set_property(COMPILE_DEFINITIONS "PIN_GNSS_TX=30" APPEND)   # was 18, SDIO CLK
idf_build_set_property(COMPILE_DEFINITIONS "PIN_VTX_TX=31" APPEND)    # was 19, SDIO CMD

#  STEP 0 OF SECTION 4.3.2: compile and run the DShot RMT driver for the first time.
#  dshot_rmt.cpp has never been through a compiler. Not a default -- the
#  unverified-defaults check keeps DSHOT_ENABLE at 0 in config.h.
idf_build_set_property(COMPILE_DEFINITIONS "DSHOT_ENABLE=1" APPEND)

#  BUILD SWITCHES FROM THE COMMAND LINE.
#
#  Section 3.2 documents the frame/motor/propeller matrix as compile-time switches
#  rather than sets of edits, and under PlatformIO that was `pio run -- -DFRAME_SIZE_IN=7`.
#  idf.py's -D sets CMake CACHE variables, not compile definitions, so it needs this
#  forwarding step to mean the same thing:
#
#      idf.py build -DFRAME_SIZE_IN=7
#      idf.py build -DFRAME_SIZE_IN=10 -DMOTOR_CLASS=MOTOR_3115 -DPROP_BLADES=3
#
#  ONE DIFFERENCE FROM pio, AND IT WILL CATCH SOMEBODY. CMake caches these, so a switch
#  passed once keeps applying to every later `idf.py build` until it is overridden or the
#  cache is cleared with `idf.py fullclean`. pio applied them per invocation. The build
#  prints what it applied for exactly this reason, and so does the firmware at boot.
#
#  Every name here is #ifndef-guarded in config.h; the build-switches check in
#  tools/check_consistency.py verifies that, so a switch cannot be documented here and
#  silently ignored by the header.
#  DSHOT_ENABLE is deliberately absent: it is force-set above for bring-up, and
#  appending a second definition would leave two -D flags disagreeing.
foreach(sw FRAME_SIZE_IN MOTOR_CLASS PROP_BLADES FLIGHT_LOOP_HZ
           REQUIRE_REMOTE_ID_TO_ARM ACCEPT_CONNECTOR_OVER_RATING)
  if(DEFINED ${sw})
    idf_build_set_property(COMPILE_DEFINITIONS "${sw}=${${sw}}" APPEND)
    message(STATUS "odyssey: build switch ${sw}=${${sw}}")
  endif()
endforeach()

project(odyssey-fc)






