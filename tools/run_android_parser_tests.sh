#!/bin/sh
# Compiles and runs the Remote ID decoder tests with nothing but javac and java.
#
# No Gradle, no Android SDK, no network. That is the point: the decoder is
# dependency-free Java precisely so it can be verified in about a second on any machine
# with a JDK, which is what makes it viable in the pre-push hook.
#
# Building the actual APK needs Gradle and the Android SDK -- see android/README.md.
set -e
cd "$(dirname "$0")/.."

SRC="android/app/src/main/java/dk/odyssey/ridtest/odid"
TEST="android/app/src/test/java/dk/odyssey/ridtest/odid"
OUT="android/build/test-classes"

rm -rf "$OUT"
mkdir -p "$OUT"

javac -Xlint:-options -d "$OUT" "$SRC"/*.java "$TEST"/*.java
java -cp "$OUT" dk.odyssey.ridtest.odid.OdidParserTest
