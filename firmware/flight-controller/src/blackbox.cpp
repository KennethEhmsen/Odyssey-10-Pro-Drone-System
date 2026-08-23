// =====================================================================================
//  Odyssey-10 Pro -- BlackBox implementation
// =====================================================================================

#include "blackbox.h"
#include "config.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>

BlackBox blackbox;

static SPIClass sdSpi(FSPI);
static File     logFile;

bool BlackBox::begin() {
  sdSpi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // 25 MHz is optimistic for a hand-wired SPI adapter with long leads. Try it, then
  // fall back rather than reporting a dead card.
  if (!SD.begin(PIN_SD_CS, sdSpi, 25000000)) {
    if (!SD.begin(PIN_SD_CS, sdSpi, 8000000)) {
      Serial.println("[BLACKBOX] no MicroSD card");
      ready_ = false;
      return false;
    }
    Serial.println("[BLACKBOX] card mounted at 8 MHz (25 MHz failed)");
  }

  ready_ = true;
  Serial.printf("[BLACKBOX] card ready, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

bool BlackBox::startFlight(uint32_t flightNumber) {
  if (!ready_) return false;
  endFlight();

  snprintf(path_, sizeof(path_), "/flight_%04lu.ody", (unsigned long)flightNumber);
  logFile = SD.open(path_, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[BLACKBOX] cannot open %s\n", path_);
    return false;
  }

  BlackBoxHeader h{};
  h.magic       = BLACKBOX_MAGIC;
  h.version     = BLACKBOX_VERSION;
  h.recordBytes = (uint16_t)sizeof(BlackBoxRecord);
  h.logRateHz   = BLACKBOX_LOG_HZ;
  h.bootTimeMs  = millis();
  strncpy(h.airframe, AIRFRAME_NAME, sizeof(h.airframe) - 1);
  logFile.write((const uint8_t*)&h, sizeof(h));
  logFile.flush();

  //  Relaxed: no logging is in flight yet -- fileOpen_ is still false, so the
  //  producer cannot be running.
  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  dropped_.store(0, std::memory_order_relaxed);
  written_ = 0;
  fileOpen_ = true;
  Serial.printf("[BLACKBOX] logging to %s\n", path_);
  return true;
}

void BlackBox::endFlight() {
  if (!fileOpen_) return;
  service();                 // drain whatever is still buffered
  logFile.flush();
  logFile.close();
  fileOpen_ = false;
  Serial.printf("[BLACKBOX] closed %s -- %lu records, %lu dropped\n",
                path_, (unsigned long)written_,
                (unsigned long)dropped_.load(std::memory_order_relaxed));
}

void BlackBox::push(const BlackBoxRecord& rec) {
  if (!fileOpen_) return;

  //  Relaxed on our own index: this task is the only writer of head_.
  const uint32_t head = head_.load(std::memory_order_relaxed);
  const uint32_t next = (head + 1) % BLACKBOX_RING_RECORDS;

  //  Acquire on tail_: once we see the consumer's advance, the slots it freed really
  //  are finished with.
  if (next == tail_.load(std::memory_order_acquire)) {
    // Ring full: the card is stalling. Drop the sample and count it. Dropping is the
    // right call -- blocking here would miss a 500 Hz control deadline, and a gap in
    // the log is far cheaper than a gap in the control loop.
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ring_[head] = rec;

  //  RELEASE. This is the barrier that makes the record above visible to the
  //  storage task on the other core before the index that points at it.
  head_.store(next, std::memory_order_release);
}

void BlackBox::service() {
  if (!fileOpen_) return;

  // Copy out of the ring in blocks so the SD layer sees large sequential writes.
  static BlackBoxRecord batch[BLACKBOX_FLUSH_RECORDS];
  //  ACQUIRE, paired with the release store in push(): every record published
  //  before this head value is visible to us now.
  uint32_t head = head_.load(std::memory_order_acquire);
  uint32_t tail = tail_.load(std::memory_order_relaxed);   // we are its only writer

  while (tail != head) {
    uint32_t n = 0;
    while (tail != head && n < BLACKBOX_FLUSH_RECORDS) {
      batch[n++] = ring_[tail];
      tail = (tail + 1) % BLACKBOX_RING_RECORDS;
    }
    if (n == 0) break;

    //  RELEASE, and published BEFORE the write rather than after it. The producer may
    //  not reuse these slots until the copy above has finished, and an SD card can
    //  stall for 100 ms -- freeing the slots first is what keeps that stall from
    //  turning into dropped samples.
    tail_.store(tail, std::memory_order_release);

    logFile.write((const uint8_t*)batch, n * sizeof(BlackBoxRecord));
    written_ += n;

    head = head_.load(std::memory_order_acquire);
  }

  // Flush roughly once a second. Frequent flushes cost throughput; rare ones cost
  // data if the aircraft loses power hard.
  static uint32_t lastFlush = 0;
  const uint32_t now = millis();
  if (now - lastFlush >= 1000) {
    lastFlush = now;
    logFile.flush();
  }
}
