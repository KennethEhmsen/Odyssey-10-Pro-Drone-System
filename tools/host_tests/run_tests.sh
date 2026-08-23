#!/bin/sh
# Build and run the host-side verification of the safety-critical algorithms.
# Requires only a C++17 compiler -- no ESP32 toolchain, no hardware.
#
# CI sets EXTRA_CXXFLAGS=-Werror. It is deliberately NOT the default here, so a new
# compiler release introducing a new diagnostic breaks CI rather than blocking local
# development.
set -e
cd "$(dirname "$0")"

g++ -std=c++17 -Wall -Wextra ${EXTRA_CXXFLAGS:-} \
    -I . \
    -I ../../shared \
    -I ../../firmware/flight-controller/include \
    -I ../../firmware/remote-id/src \
    test_all.cpp \
    ../../firmware/remote-id/src/identity.cpp \
    ../../firmware/flight-controller/src/blackbox.cpp \
    -o test_all

./test_all
