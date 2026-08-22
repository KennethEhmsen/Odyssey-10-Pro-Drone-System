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

idf_build_set_property(COMPILE_DEFINITIONS "BRINGUP_TRACE=1" APPEND)

#  THE ESP32-P4's USB SERIAL/JTAG LINES ARE GPIO 24/26 (D-) AND 25/27 (D+).
#
#  Section 9's pinout assigns all four. Touching any of them while the USB console is
#  live makes the device vanish from the host -- Espressif documents exactly this -- and
#  on this board the chip then resets, so it boot-loops forever:
#
#      [TRACE] before pinMode(REMOTEID_HEALTH gpio25)
#      rst:0x17 (CHIP_USB_UART_RESET),boot:0x20f
#
#  That single pinMode is the cause of every symptom during bring-up: the port that
#  disappeared, the captures that returned nothing, and the "crash" that had no
#  backtrace because the console died with it.
#
#  This is NOT only a development-board problem. It would equally prevent USB
#  programming and console on the finished aircraft, so section 9 needs correcting --
#  see finding 41. These overrides keep the bench moving in the meantime.
idf_build_set_property(COMPILE_DEFINITIONS "PIN_AUX_BUS_TX=39" APPEND)       # was 24, USB D-
idf_build_set_property(COMPILE_DEFINITIONS "PIN_REMOTEID_HEALTH=40" APPEND)  # was 25, USB D+
idf_build_set_property(COMPILE_DEFINITIONS "PIN_PARACHUTE_SRV=41" APPEND)    # was 26, USB D-
idf_build_set_property(COMPILE_DEFINITIONS "PIN_CRSF_RX=42" APPEND)          # was 27, USB D+

project(odyssey-fc)






