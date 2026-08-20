#!/bin/sh
# Build and run the host-side verification of the safety-critical algorithms.
# Requires only a C++17 compiler -- no ESP32 toolchain, no hardware.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra \
    -I . \
    -I ../../shared \
    -I ../../firmware/flight-controller/include \
    test_all.cpp -o test_all
./test_all
