"""
Vendors the Arduino libraries platformio.ini declares, as ESP-IDF components.

PlatformIO fetches lib_deps automatically. The IDF component manager does not know about
the Arduino library ecosystem, so each one needs a CMakeLists.txt to become a component.

Versions are pinned to tags matching the constraints in platformio.ini, so the two build
systems agree about what is being compiled rather than drifting apart.
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path("firmware/flight-controller/components")

#  name, repo, tag, source layout, extra REQUIRES
LIBS = [
    ("Adafruit_BusIO", "https://github.com/adafruit/Adafruit_BusIO.git",
     "1.17.4", ".", ["espressif__arduino-esp32"]),
    ("Adafruit_Unified_Sensor", "https://github.com/adafruit/Adafruit_Sensor.git",
     "1.1.15", ".", ["espressif__arduino-esp32"]),
    ("Adafruit_MPU6050", "https://github.com/adafruit/Adafruit_MPU6050.git",
     "2.2.6", ".", ["espressif__arduino-esp32", "Adafruit_BusIO", "Adafruit_Unified_Sensor"]),
    ("Adafruit_BMP280", "https://github.com/adafruit/Adafruit_BMP280_Library.git",
     "2.6.8", ".", ["espressif__arduino-esp32", "Adafruit_BusIO", "Adafruit_Unified_Sensor"]),
    ("TinyGPSPlus", "https://github.com/mikalhart/TinyGPSPlus.git",
     "v1.0.3a", "src", ["espressif__arduino-esp32"]),
    ("LoRa", "https://github.com/sandeepmistry/arduino-LoRa.git",
     "0.8.0", "src", ["espressif__arduino-esp32"]),
]

CMAKE = """# Generated wrapper: an Arduino library presented to ESP-IDF as a component.
# See firmware/flight-controller/components/README.md for why these are vendored.
file(GLOB_RECURSE SOURCES "{src}/*.cpp" "{src}/*.c")
idf_component_register(
    SRCS ${{SOURCES}}
    INCLUDE_DIRS "{src}"
    REQUIRES {reqs}
)
"""

ROOT.mkdir(parents=True, exist_ok=True)
failed = []

for name, repo, tag, src, reqs in LIBS:
    dest = ROOT / name
    if dest.exists():
        print(f"  {name:26} already present")
    else:
        r = subprocess.run(
            ["git", "clone", "--depth", "1", "--branch", tag, repo, str(dest)],
            capture_output=True, text=True)
        if r.returncode != 0:
            # Some of these tag with a leading v and some do not.
            alt = tag[1:] if tag.startswith("v") else "v" + tag
            r = subprocess.run(
                ["git", "clone", "--depth", "1", "--branch", alt, repo, str(dest)],
                capture_output=True, text=True)
        if r.returncode != 0:
            failed.append((name, r.stderr.strip().splitlines()[-1] if r.stderr else "?"))
            print(f"  {name:26} CLONE FAILED")
            continue
        print(f"  {name:26} cloned at {tag}")

    # The vendored .git directories would make these look like submodules.
    dotgit = dest / ".git"
    if dotgit.exists():
        subprocess.run(["rm", "-rf", str(dotgit)], capture_output=True)

    (dest / "CMakeLists.txt").write_text(
        CMAKE.format(src=src, reqs=" ".join(reqs)), encoding="utf-8")

if failed:
    print("\nFAILED:")
    for n, e in failed:
        print(f"  {n}: {e}")
    sys.exit(1)
print("\nall libraries vendored as components")
