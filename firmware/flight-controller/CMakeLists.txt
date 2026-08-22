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
project(odyssey-fc)
