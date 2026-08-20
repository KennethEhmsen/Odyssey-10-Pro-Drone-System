// =====================================================================================
//  Odyssey-10 Pro -- OpenDroneID broadcast transport (BLE 5 Long Range + Wi-Fi beacon)
//  ------------------------------------------------------------------------------------
//  Thin wrapper over the ESP-IDF radio APIs. The MESSAGE ENCODING is done by
//  opendroneid-core-c; this file only wraps the already-encoded 25-byte messages in the
//  advertising containers that ASTM F3411 specifies:
//
//    Bluetooth:  AD type 0x16 (Service Data, 16-bit UUID), UUID 0xFFFA, then the
//                ASTM application code 0x0D, a message counter, and the message.
//                Broadcast on the Coded PHY (S=8) for long range, using extended
//                advertising -- legacy advertising cannot carry a 25-byte message
//                plus the header inside the 31-byte legacy PDU.
//
//    Wi-Fi:      vendor-specific information element in a beacon frame, OUI FA-0B-BC.
//
//  Both media are enabled because ground receivers vary: most phone apps read
//  Bluetooth, while some regulator-supplied receivers read Wi-Fi.
// =====================================================================================

#ifndef ODY_ODID_TRANSPORT_H
#define ODY_ODID_TRANSPORT_H

#include <stdint.h>

// Brings up the BLE extended advertiser and the Wi-Fi beacon interface.
void odidTransportBegin();

// Publishes one already-encoded OpenDroneID message on every enabled medium.
//   messageType : ODID_messagetype_t value, used for logging and the Wi-Fi IE
//   encoded     : pointer to the 25-byte encoded message from opendroneid-core-c
//   length      : always 25 for a single message
//   counter     : per-message-type sequence counter required by the standard
void odidTransportSend(uint8_t messageType, const uint8_t* encoded,
                       uint32_t length, uint8_t counter);

// True once both radios have accepted their first advertisement. The flight
// controller's arm check treats a false here as a blocking preflight failure.
bool odidTransportHealthy();

#endif // ODY_ODID_TRANSPORT_H
