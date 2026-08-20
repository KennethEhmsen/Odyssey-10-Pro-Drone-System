# Remote ID — regulatory reference notes

Working notes on Direct Remote ID for a **privately built** FPV aircraft flown in
**Denmark / EU**, supplied as project input on 20 August 2026. These informed the
revision 2.1 corrections to section 12 of the specification.

> **Status of this document.** These are reference notes, not legal advice, and the
> regulatory position moves. Where they disagree with the specification, the
> specification has been corrected to match them. Where either disagrees with your
> national authority, the authority wins.

---

## 1. Whether Remote ID is required at all

The requirement attaches to **class-marked** aircraft, not to weight alone.

| DIY aircraft | Open-category operation | Direct Remote ID |
| --- | --- | --- |
| < 250 g, < 19 m/s | A1 | Not required solely because it is DIY |
| < 25 kg | A3 | Not required solely because it is DIY |
| Any | **Specific category** | **Required** |

EASA explicitly allows privately built UAS in A1 under 250 g and in A3 under 25 kg. The
A3 requirement for a privately built aircraft does not itself say the aircraft must have
an active Direct Remote ID system — that requirement appears for **C2/C3-class**
aircraft. This is the important distinction from a commercially produced C1/C2/C3 drone,
and it is the one revision 2.0 of the specification got wrong.

**Denmark.** Trafikstyrelsen's guidance requires Remote ID for relevant C-marked drones,
and regardless of weight or class in the specific category. A national change is in
progress: proposed rules broadening electronic-visibility requirements, consultation
closed **21 August 2026**, proposed effect **1 January 2027**. Proposed, not adopted.

---

## 2. Architecture — Remote ID is not carried over ExpressLRS

ELRS is the command-and-control link. Remote ID is a separate local broadcast. Keeping
them separate is the whole point.

```
                    RADIO
                ┌─────────────┐
                │ EdgeTX      │
                │ ELRS 2.4GHz │
                └──────┬──────┘
                       │ ELRS RF link
              ┌────────▼────────┐
              │ ELRS Receiver   │
              └────────┬────────┘
                       │ CRSF
                 ┌─────▼──────┐
                 │   Flight   │
                 │ Controller │
                 └─────┬──────┘
                       │ GPS / UART / MAVLink
             ┌─────────▼─────────┐
             │ Remote-ID module  │
             │  MCU              │
             │  GNSS or FC feed  │
             │  CTA-2063-A SN    │
             │  Operator ID      │
             │  OpenDroneID      │
             └─────────┬─────────┘
                       │ Bluetooth / Wi-Fi
              Nearby smartphones
```

EU law requires Direct Remote ID to be a **local periodic broadcast using an open and
documented protocol that existing mobile devices can receive directly**. An ELRS
telemetry packet containing GPS coordinates is *not* Remote ID.

**Two integration designs:**

- **A — Remote ID has its own GNSS.** Independent, easier to certify as an add-on,
  does not depend on flight-controller internals. Costs another GNSS, weight, power and
  a slower initial fix.
- **B — Remote ID takes data from the flight controller** over MAVLink, MSP, DroneCAN or
  a custom UART. Lighter and faster to fix.

*This project uses design B*, via the AUX broadcast bus. See specification section 7.2.

---

## 3. The two identifiers

### CTA-2063-A — identifies the hardware

```
    MFR CODE | LENGTH CODE | MFR SERIAL
      8653   |      C      | 000000000001
```

Four-character ICAO manufacturer code, a length code, and a manufacturer serial of up to
15 characters. Permitted characters are 0–9 and A–Z **excluding I and O**.

A 15-character serial gives a fixed 20-character identifier:

```
    K7E3F000000000000001
    K7E3 = ICAO manufacturer code
    F    = 15-character serial follows
```

Advantages: fixed-length database field, maximum serial namespace, straightforward
validation, easy QR/barcode generation, and it matches OpenDroneID's 20-byte UAS ID
field exactly.

**Prefer non-sequential serials.** Sequential numbering exposes manufacturing volume.
Something like `7C2P4K8M1X6R3T9` does not, while remaining deterministic internally
against a record of `device_uuid`, `manufacturing_sequence`, `cta_serial`,
`hardware_revision`, `firmware_version`, `public_key`, `production_date`.

**Manufacturer code application:** `OPSInbox@icao.int`, providing company name,
manufacturer's common name, headquarters address, contact name, telephone, email and
website. ICAO assigns codes to aircraft and UAS manufacturers.
See <https://www.icao.int/operational-safety/doc-8643-aircraft-type-designators/manufacturers-codes>

### Operator registration number — identifies the person

Issued by Trafikstyrelsen in Denmark. EASA's recommended interoperable structure:

```
    CCC + 12 random characters + checksum      (16 public characters)
    -   + 3 secret characters
```

Example: `FIN87astrdge12k8-xyz`. Denmark uses the `DNK` prefix. Trafikstyrelsen calls the
additional Danish value the **three-digit security code**, needed when configuring
Remote ID.

> **Do not publish the three secret characters** — not on a sticker, not on a website,
> not in a repository. EASA is explicit that they should not be shared. The airframe is
> marked with the ordinary operator registration number; the full string is loaded into
> the identification system.

---

## 4. Information the broadcast must carry

**Identity**

```
UAS operator registration
CTA-2063-A Remote-ID serial number
```

**Aircraft state**

```
Timestamp, Latitude, Longitude, Height, Course, Ground speed
```

**Operator / take-off information**

```
Remote pilot geographical position, or for an add-on, the take-off position
```

EU Part 6 specifies these elements for a Direct Remote Identification add-on.

---

## 5. Broadcasting OpenDroneID is not the same as compliance

There is a large gap between *"my ESP32 broadcasts OpenDroneID packets"* and *"this is
an EU-compliant Direct Remote Identification add-on"*. The first is relatively easy. The
second is a regulated product-compliance problem.

EU Part 6 additionally requires operator-ID upload, validity and consistency checking, a
CTA-2063-A serial physically associated with the module, continuous periodic broadcast,
defined aircraft information, tamper resistance, and manufacturer instructions covering
installation and protocol. Placing the module on the EU market brings further
product-conformity obligations.

Using the OpenDroneID library does not by itself confer regulatory compliance.

---

## 6. 2.4 GHz coexistence

An aircraft may carry ELRS at 2.4 GHz, Remote ID Wi-Fi/BLE at 2.4 GHz, and video at
5.8 GHz. Antenna placement matters.

```
                FRONT
            GPS antenna
                │
         ┌──────────────┐
         │ Flight Ctrl  │
ELRS RX ─┤              ├─ Remote ID
         └──────────────┘
    │                         │
ELRS antenna              RID antenna
    │                         │
    ▼                         ▼
     physical separation where practical
                 REAR
```

Do not mount the Wi-Fi/BLE Remote ID antenna immediately against the ELRS receiver
antenna. **ELRS packet reception is mission-critical; Remote ID should not degrade the
RC receiver's RF environment.** See specification section 8.5 for how this project
handles it.

---

## 7. Suggested build order

1. Define the regulatory profile — Denmark/EU rather than FAA.
2. **Define the CTA-2063-A serial-number database and generator.**
3. Apply to ICAO for the 4-character manufacturer code.
4. Select MCU and GNSS architecture.
5. Integrate OpenDroneID.
6. Design the flight-controller → Remote ID telemetry path.
7. **Implement Danish/EASA operator-ID validation including Luhn mod-36.**
8. Implement BLE/Wi-Fi broadcasting.
9. Build an Android test receiver/scanner.
10. Create a compliance/test matrix against EU 2019/945 Part 6.
11. Then design the PCB.

Steps 2 and 7 come first because they can be fully specified before buying hardware,
and they give a solid identity/provisioning layer to build on.

> **Status in this project:** steps 2 and 7 are implemented in
> `firmware/remote-id/src/identity.{h,cpp}` with 34 host assertions in
> `tools/host_tests/test_all.cpp`. Step 5 is implemented against the reference encoder,
> step 6 uses the AUX broadcast bus. Steps 3, 9, 10 and 11 are outstanding.
>
> The Luhn mod-36 implementation reproduces EASA's published example but has been
> validated against **that one vector only** — see the confidence warning in
> `identity.h`. It is advisory in the firmware for that reason.
