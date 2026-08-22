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

#  Enable for bring-up: -DBRINGUP_TRACE=1 prints a flushed marker before each
#  call in setup(), so a fault that takes the console down names itself. This is
#  what found finding 41 in one run after a long detour of guessing.
# idf_build_set_property(COMPILE_DEFINITIONS "BRINGUP_TRACE=1" APPEND)

project(odyssey-fc)






