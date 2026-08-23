// =====================================================================================
//  Host stand-in for the SD / SPI / FS surface that blackbox.cpp uses.
//
//  The point is to run the REAL BlackBox ring on a desktop -- blackbox.cpp is compiled
//  and linked into the test binary, not reimplemented here. Only the card underneath it
//  is fake: writes land in a std::vector so a test can assert on exactly what would have
//  reached the file.
//
//  WHAT THIS CAN AND CANNOT VERIFY
//
//  It verifies the drain CONTRACT: that every record pushed is either written or counted
//  as dropped, that records arrive in order across a wrap, that the gate closes before
//  the final drain so nothing is stranded, and that the sequencing guards hold.
//
//  It does NOT verify the memory ORDERING. x86 will not reorder the stores that an
//  ESP32-P4 can, so a single-threaded desktop run would pass on the volatile code that
//  commit a668baa replaced. The acquire/release pairing is argued at its call sites and
//  confirmed only by reasoning. Do not read a green suite here as evidence about it.
//
//  This is a TEST FIXTURE. Nothing in firmware/ includes it.
// =====================================================================================

#ifndef ODY_SD_SHIM_H
#define ODY_SD_SHIM_H

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace hostsd {

struct Blob {
  std::vector<uint8_t> bytes;
  bool                 closed = false;
};

inline std::map<std::string, Blob>& files() {
  static std::map<std::string, Blob> m;
  return m;
}
inline bool& cardPresent() { static bool p = true; return p; }
inline uint32_t& flushes() { static uint32_t n = 0; return n; }

//  Serial output is discarded by default so the suite stays readable. Flip this when a
//  test needs to see what the firmware reported.
inline bool& echoSerial() { static bool e = false; return e; }

inline void reset() {
  files().clear();
  cardPresent() = true;
  flushes()     = 0;
}

}  // namespace hostsd

// ---- SPI -----------------------------------------------------------------------------
#define FSPI 0
class SPIClass {
public:
  explicit SPIClass(int = FSPI) {}
  void begin(int, int, int, int) {}
};

// ---- FS / SD -------------------------------------------------------------------------
#define FILE_WRITE "w"

class File {
public:
  File() = default;
  explicit File(const std::string& path) : path_(path), valid_(true) {}

  explicit operator bool() const { return valid_; }

  size_t write(const uint8_t* data, size_t len) {
    if (!valid_) return 0;
    auto& b = hostsd::files()[path_].bytes;
    b.insert(b.end(), data, data + len);
    return len;
  }
  void flush() { ++hostsd::flushes(); }
  void close() {
    if (valid_) hostsd::files()[path_].closed = true;
    valid_ = false;
  }

private:
  std::string path_;
  bool        valid_ = false;
};

class SDClass {
public:
  bool begin(int, SPIClass&, uint32_t) { return hostsd::cardPresent(); }
  uint64_t cardSize() const { return 8ULL * 1024 * 1024 * 1024; }
  File open(const char* path, const char* /*mode*/) {
    if (!hostsd::cardPresent()) return File();
    hostsd::files()[path] = hostsd::Blob{};
    return File(path);
  }
};

inline SDClass SD;

// ---- Serial --------------------------------------------------------------------------
class HostSerial {
public:
  void printf(const char* fmt, ...) {
    if (!hostsd::echoSerial()) return;
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
  }
  void println(const char* s) {
    if (hostsd::echoSerial()) std::printf("%s\n", s);
  }
  void println() {
    if (hostsd::echoSerial()) std::printf("\n");
  }
};

inline HostSerial Serial;

#endif  // ODY_SD_SHIM_H
