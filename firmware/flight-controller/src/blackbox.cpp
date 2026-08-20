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

  head_ = tail_ = 0;
  dropped_ = 0;
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
                path_, (unsigned long)written_, (unsigned long)dropped_);
}

void BlackBox::push(const BlackBoxRecord& rec) {
  if (!fileOpen_) return;

  const uint32_t next = (head_ + 1) % BLACKBOX_RING_RECORDS;
  if (next == tail_) {
    // Ring full: the card is stalling. Drop the sample and count it. Dropping is the
    // right call -- blocking here would miss a 500 Hz control deadline, and a gap in
    // the log is far cheaper than a gap in the control loop.
    ++dropped_;
    return;
  }
  ring_[head_] = rec;
  head_ = next;
}

void BlackBox::service() {
  if (!fileOpen_) return;

  // Copy out of the ring in blocks so the SD layer sees large sequential writes.
  static BlackBoxRecord batch[BLACKBOX_FLUSH_RECORDS];
  while (tail_ != head_) {
    uint32_t n = 0;
    while (tail_ != head_ && n < BLACKBOX_FLUSH_RECORDS) {
      batch[n++] = ring_[tail_];
      tail_ = (tail_ + 1) % BLACKBOX_RING_RECORDS;
    }
    if (n == 0) break;
    logFile.write((const uint8_t*)batch, n * sizeof(BlackBoxRecord));
    written_ += n;
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
