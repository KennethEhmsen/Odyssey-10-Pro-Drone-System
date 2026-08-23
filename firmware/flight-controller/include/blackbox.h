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
#include <atomic>
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
  uint32_t recordsDropped() const { return dropped_.load(std::memory_order_relaxed); }
  const char* currentPath() const { return path_; }

private:
  BlackBoxRecord ring_[BLACKBOX_RING_RECORDS];
  //  ATOMIC, NOT VOLATILE -- AND THE DIFFERENCE IS A CORRUPTED FLIGHT LOG.
  //
  //  push() runs in TaskFlightLoop pinned to core 1; service() runs in TaskStorage
  //  pinned to core 0. This is a real cross-core single-producer/single-consumer
  //  queue, not two threads on one core.
  //
  //  volatile keeps the compiler from caching these in registers, and that is ALL it
  //  does. It emits no memory barrier, and -- the part that bites -- it does not
  //  order the NON-volatile store of ring_[head_] against the volatile store to
  //  head_. Either the compiler or the store buffer may let the index land first, so
  //  the consumer can observe an advanced head_ while ring_[tail_] still holds the
  //  previous record, or half of the new one. BlackBoxRecord is many words wide, so
  //  the failure is a torn record, silently, in the one file that exists to explain
  //  a crash.
  //
  //  With acquire/release the release store to head_ publishes everything written to
  //  the slot before it, and the consumer's acquire load makes it visible. See
  //  push() and service() for where each ordering is paired.
  //
  //  C++20 deprecating volatile compound assignment is what surfaced this. The
  //  warning was pointing at a defect, not at a style preference.
  std::atomic<uint32_t> head_{0};      // written by the producer only
  std::atomic<uint32_t> tail_{0};      // written by the consumer only
  std::atomic<uint32_t> dropped_{0};

  //  A lock-based atomic here would take a mutex inside the 500 Hz flight loop --
  //  exactly the blocking this ring exists to avoid. Fail the build instead.
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "blackbox ring indices must be lock-free: push() runs in the flight "
                "loop and must never block");
  uint32_t written_ = 0;
  bool     ready_   = false;
  bool     fileOpen_ = false;
  char     path_[32] = {0};
};

extern BlackBox blackbox;

#endif // ODY_BLACKBOX_H
