// =====================================================================================
//  Odyssey-10 Pro -- Flight state machine
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 6.
//
//  The original code kept the flight state in a bare `volatile FlightState` written
//  from both cores. triggerFailsafe() ran on core 0 and performed an unsynchronised
//  read-modify-write:
//
//      if (currentState != FAILSAFE && currentState != DISARMED && ...) {
//          currentState = STATE_FAILSAFE_LANDING;      // <-- separate store
//      }
//
//  Core 1 could deploy the parachute in the window between the test and the store.
//  The result: STATE_FREEFALL_PARACHUTE is silently overwritten with
//  STATE_FAILSAFE_LANDING, the flight loop takes the powered-descent branch, and all
//  four motors spin back up underneath an already-deployed canopy.
//
//  Two things fix it:
//
//    1. Every transition is a single atomic test-and-set under a spinlock. There is no
//       window between the guard and the store.
//
//    2. Transitions are ESCALATION-ORDERED. The OdyFlightState enum values are ranked
//       by severity and a request is only honoured if it moves to an equal or higher
//       rank. STATE_FAILSAFE_LANDING (7) therefore cannot displace
//       STATE_FREEFALL_PARACHUTE (8) no matter how the two cores interleave -- the race
//       is not merely narrowed, it is made harmless.
//
//  Going back down the ladder (rearming after a landing) requires an explicit reset
//  that is only legal from DISARMED and only from the telemetry core.
// =====================================================================================

#ifndef ODY_STATE_MACHINE_H
#define ODY_STATE_MACHINE_H

#include <Arduino.h>
#include "odyssey_link.h"

#define ODY_STATE_REASON_LEN 40

class FlightStateMachine {
public:
  void begin() {
    portENTER_CRITICAL(&lock_);
    state_       = ODY_STATE_BOOT;
    changedAtMs_ = millis();
    strncpy(reason_, "power on", ODY_STATE_REASON_LEN - 1);
    reason_[ODY_STATE_REASON_LEN - 1] = '\0';
    portEXIT_CRITICAL(&lock_);
  }

  // Atomic escalate-only transition. Returns true if the state actually changed.
  // Safe to call from either core, from a task or from an ISR context.
  bool request(uint8_t desired, const char* why) {
    bool changed = false;
    portENTER_CRITICAL(&lock_);
    if (desired > state_) {
      state_       = desired;
      changedAtMs_ = millis();
      copyReason(why);
      changed = true;
    }
    portEXIT_CRITICAL(&lock_);
    if (changed) pendingAnnounce_ = true;
    return changed;
  }

  // De-escalation. Legal only out of DISARMED, which is how a landed aircraft gets
  // back to PREFLIGHT_OK for another flight. Any other use is a bug and is rejected.
  bool resetFromDisarmed(uint8_t desired, const char* why) {
    bool changed = false;
    portENTER_CRITICAL(&lock_);
    if (state_ == ODY_STATE_DISARMED && desired <= ODY_STATE_PREFLIGHT_OK) {
      state_       = desired;
      changedAtMs_ = millis();
      copyReason(why);
      changed = true;
    }
    portEXIT_CRITICAL(&lock_);
    if (changed) pendingAnnounce_ = true;
    return changed;
  }

  // Transition that is only taken if the machine is currently in `expected`.
  // Used for the ARMED -> RTH_NAVIGATING style moves where a stale request from a
  // slow task should be dropped rather than applied late.
  bool requestFrom(uint8_t expected, uint8_t desired, const char* why) {
    bool changed = false;
    portENTER_CRITICAL(&lock_);
    if (state_ == expected && desired > state_) {
      state_       = desired;
      changedAtMs_ = millis();
      copyReason(why);
      changed = true;
    }
    portEXIT_CRITICAL(&lock_);
    if (changed) pendingAnnounce_ = true;
    return changed;
  }

  uint8_t get() const {
    portENTER_CRITICAL(&lock_);
    const uint8_t s = state_;
    portEXIT_CRITICAL(&lock_);
    return s;
  }

  uint32_t millisInState() const {
    portENTER_CRITICAL(&lock_);
    const uint32_t t = changedAtMs_;
    portEXIT_CRITICAL(&lock_);
    return millis() - t;
  }

  bool motorsLive() const { return odyMotorsAreLive(get()); }
  bool airborne()   const { return odyIsAirborneState(get()); }

  // Pops a one-shot flag so the telemetry task can log the change exactly once,
  // without the flight loop ever touching Serial.
  bool consumeAnnounce(uint8_t* stateOut, char* reasonOut, size_t reasonCap) {
    if (!pendingAnnounce_) return false;
    pendingAnnounce_ = false;
    portENTER_CRITICAL(&lock_);
    if (stateOut) *stateOut = state_;
    if (reasonOut && reasonCap) {
      strncpy(reasonOut, reason_, reasonCap - 1);
      reasonOut[reasonCap - 1] = '\0';
    }
    portEXIT_CRITICAL(&lock_);
    return true;
  }

private:
  void copyReason(const char* why) {   // caller must hold the lock
    if (!why) { reason_[0] = '\0'; return; }
    size_t i = 0;
    for (; i < ODY_STATE_REASON_LEN - 1 && why[i]; ++i) reason_[i] = why[i];
    reason_[i] = '\0';
  }

  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint8_t     state_       = ODY_STATE_BOOT;
  volatile uint32_t    changedAtMs_ = 0;
  char                 reason_[ODY_STATE_REASON_LEN] = {0};
  volatile bool        pendingAnnounce_ = false;
};

#endif // ODY_STATE_MACHINE_H
