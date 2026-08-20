// =====================================================================================
//  Odyssey-10 Pro -- Remote ID identity handling
//  ------------------------------------------------------------------------------------
//  Two completely different identifiers are involved in Direct Remote ID, and conflating
//  them is a common and consequential mistake:
//
//    CTA-2063-A SERIAL NUMBER  identifies the HARDWARE.
//        Issued by the manufacturer under an ICAO-assigned 4-character code.
//        Structure:  [MFR code: 4][length code: 1][manufacturer serial: 1..15]
//        Length code: '1'..'9' = 1..9 characters, 'A'..'F' = 10..15 characters.
//        Character set excludes I and O to avoid confusion with 1 and 0.
//        Example: K7E3F000000000000001  (K7E3, F = 15 chars, then 15 characters)
//        OpenDroneID stores the UAS ID in a 20-byte field, so a 4+1+15 = 20 character
//        serial uses it exactly.
//
//    EU OPERATOR REGISTRATION NUMBER  identifies the PERSON.
//        Issued to the operator by their national authority (Trafikstyrelsen in
//        Denmark, prefix DNK).
//        Structure:  [ICAO country: 3][random: 12][checksum: 1] then '-' then [secret: 3]
//        Example: FIN87astrdge12k8-xyz
//        The 16 characters before the hyphen are PUBLIC and go on the airframe label.
//        The 3 characters after it are SECRET.
//
//  ------------------------------------------------------------------------------------
//  THE SECRET MUST NOT BE COMMITTED TO THIS REPOSITORY.
//
//  EASA is explicit that the three trailing characters are not to be shared, and
//  Trafikstyrelsen calls the Danish equivalent a "security code". They are provisioned
//  at runtime into NVS, never compiled in. See identity.cpp and docs section 12.3.
//
//  ------------------------------------------------------------------------------------
//  A NOTE ON WHAT THIS DOES AND DOES NOT ACHIEVE
//
//  Validating these strings does not make the module a compliant Direct Remote
//  Identification add-on. EU 2019/945 Part 6 additionally requires tamper resistance,
//  a serial physically associated with the module, operator-ID upload with validity
//  checking, continuous periodic broadcast, and manufacturer instructions -- plus
//  product-conformity obligations if the module is placed on the EU market. Linking
//  opendroneid-core-c gets the wire format right; it does not confer compliance.
// =====================================================================================

#ifndef ODY_IDENTITY_H
#define ODY_IDENTITY_H

#include <stdint.h>
#include <stddef.h>

// Longest valid CTA-2063-A serial: 4 (MFR) + 1 (length code) + 15 (serial).
#define ODY_CTA_SERIAL_MAX      20
#define ODY_CTA_MFR_CODE_LEN    4
#define ODY_CTA_SERIAL_MIN      6      // 4 + 1 + at least 1

// EU operator ID: 3 country + 12 random + 1 checksum, then '-' and 3 secret.
#define ODY_OPERATOR_PUBLIC_LEN 16
#define ODY_OPERATOR_SECRET_LEN 3
#define ODY_OPERATOR_FULL_LEN   (ODY_OPERATOR_PUBLIC_LEN + 1 + ODY_OPERATOR_SECRET_LEN)

enum OdyIdResult : uint8_t {
  ODY_ID_OK = 0,
  ODY_ID_ERR_NULL,
  ODY_ID_ERR_LENGTH,
  ODY_ID_ERR_CHARSET,          // a character outside the permitted alphabet
  ODY_ID_ERR_MFR_CODE,         // ICAO manufacturer code malformed
  ODY_ID_ERR_LENGTH_CODE,      // length code absent or not 1-9 / A-F
  ODY_ID_ERR_LENGTH_MISMATCH,  // declared length does not match the actual serial
  ODY_ID_ERR_COUNTRY,          // operator ID country prefix malformed
  ODY_ID_ERR_SEPARATOR,        // missing '-' before the secret
  ODY_ID_ERR_CHECKSUM,         // Luhn mod 36 check character does not match
  ODY_ID_ERR_PLACEHOLDER,      // still the shipped placeholder value
};

const char* odyIdResultText(uint8_t r);

// -------------------------------------------------------------------------------------
//  CTA-2063-A
// -------------------------------------------------------------------------------------

// Validates structure, alphabet, and that the declared length code matches the actual
// manufacturer serial length.
uint8_t odyValidateCtaSerial(const char* serial);

// Builds a serial from a 4-character ICAO manufacturer code and a manufacturer serial
// of 1..15 permitted characters. Writes at most ODY_CTA_SERIAL_MAX + 1 bytes.
uint8_t odyMakeCtaSerial(const char* mfrCode, const char* mfrSerial,
                         char* out, size_t outCap);

// True if c is permitted in a CTA-2063-A serial: 0-9 and A-Z excluding I and O.
bool odyIsCtaChar(char c);

// -------------------------------------------------------------------------------------
//  EU / Danish operator registration number
// -------------------------------------------------------------------------------------

// Validates the 16-character PUBLIC portion. `strict` additionally requires the Luhn
// mod 36 check character to match.
//
// IMPORTANT: pass strict = false in any path that can block flight. See the warning on
// odyOperatorChecksum() below -- a false rejection would ground a legitimate operator.
uint8_t odyValidateOperatorIdPublic(const char* publicPart, bool strict);

// Validates a full "PUBLIC-SECRET" string. Provided for the provisioning interface;
// nothing in the broadcast path should ever hold the secret.
uint8_t odyValidateOperatorIdFull(const char* full, bool strict);

// Computes the Luhn mod 36 check character over the 12-character random payload.
//
//  *** CONFIDENCE WARNING ***
//  This implementation reproduces EASA's single published example
//  (FIN87astrdge12k8) using the a-z0-9 code-point ordering over the payload alone.
//  That is ONE test vector. A brute-force over 16 plausible variants found two that
//  reproduce it, so a single example cannot distinguish between them, and the odds of
//  a spurious match across that many variants are not negligible.
//
//  Treat the result as ADVISORY until it has been checked against ASD-STAN
//  prEN 4709-002 or your national authority's specification with more vectors. This
//  is why strict mode is opt-in and why the arm path does not use it.
char odyOperatorChecksum(const char* twelveCharPayload);

// Splits a full operator ID. `secretOut` may be null if the caller does not want it.
uint8_t odySplitOperatorId(const char* full, char* publicOut, size_t publicCap,
                           char* secretOut, size_t secretCap);

#endif // ODY_IDENTITY_H
