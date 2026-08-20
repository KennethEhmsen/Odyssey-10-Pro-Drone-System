// =====================================================================================
//  Odyssey-10 Pro -- BlackBox flight data recorder
//  ------------------------------------------------------------------------------------
//  100 Hz binary logging to MicroSD over a dedicated SPI bus.
//
//  Changes from the original:
//
//    * A file header identifies the format, so tools/blackbox_decode.py does not have
//      to guess the record layout. The original wrote bare structs with no versioning.
//    * Records are staged in a ring buffer and flushed in whole blocks from a
//      dedicated low-priority task. The original called blackboxFile.write() straight
//      from loop(), where an SD card's occasional 100 ms write stall would block LoRa
//      command reception on the same thread.
//    * Motor fields carry the post-clamp values actually written to the ESCs, so
//      mixer saturation is visible in the log.
//    * Each flight opens its own file. Appending every flight to one /blackbox.bin
//      made log extraction guesswork.
// =====================================================================================

#ifndef ODY_BLACKBOX_H
#define ODY_BLACKBOX_H

#include <Arduino.h>
#include "types.h"

#define BLACKBOX_RING_RECORDS   256    // ~2.5 s of buffer at 100 Hz
#define BLACKBOX_FLUSH_RECORDS  32     // write in blocks, not one record at a time

class BlackBox {
public:
  bool begin();

  // Called from the flight loop at BLACKBOX_LOG_HZ. Lock-free single-producer push;
  // it never blocks and never touches the filesystem.
  void push(const BlackBoxRecord& rec);

  // Called from the storage task. Drains the ring to the card.
  void service();

  // Closes the current file and opens a fresh one for the next flight.
  bool startFlight(uint32_t flightNumber);
  void endFlight();

  bool     ready()        const { return ready_; }
  uint32_t recordsWritten() const { return written_; }
  uint32_t recordsDropped() const { return dropped_; }
  const char* currentPath() const { return path_; }

private:
  BlackBoxRecord ring_[BLACKBOX_RING_RECORDS];
  volatile uint32_t head_ = 0;   // producer, flight loop
  volatile uint32_t tail_ = 0;   // consumer, storage task
  volatile uint32_t dropped_ = 0;
  uint32_t written_ = 0;
  bool     ready_   = false;
  bool     fileOpen_ = false;
  char     path_[32] = {0};
};

extern BlackBox blackbox;

#endif // ODY_BLACKBOX_H
