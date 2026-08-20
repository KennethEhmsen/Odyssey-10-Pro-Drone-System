// =====================================================================================
//  Odyssey-10 Pro -- Remote ID identity handling implementation
// =====================================================================================

#include "identity.h"
#include <string.h>
#include <ctype.h>

// Luhn mod 36 code-point ordering. See the confidence warning in identity.h: the a-z0-9
// ordering (not 0-9a-z) is what reproduces EASA's published example.
static const char kLuhnCharset[] = "abcdefghijklmnopqrstuvwxyz0123456789";
static const int  kLuhnN = 36;

const char* odyIdResultText(uint8_t r) {
  switch (r) {
    case ODY_ID_OK:                    return "ok";
    case ODY_ID_ERR_NULL:              return "null string";
    case ODY_ID_ERR_LENGTH:            return "wrong length";
    case ODY_ID_ERR_CHARSET:           return "character outside the permitted alphabet";
    case ODY_ID_ERR_MFR_CODE:          return "ICAO manufacturer code malformed";
    case ODY_ID_ERR_LENGTH_CODE:       return "length code is not 1-9 or A-F";
    case ODY_ID_ERR_LENGTH_MISMATCH:   return "declared length does not match the serial";
    case ODY_ID_ERR_COUNTRY:           return "country prefix is not three letters";
    case ODY_ID_ERR_SEPARATOR:         return "missing '-' before the secret";
    case ODY_ID_ERR_CHECKSUM:          return "Luhn mod 36 check character mismatch";
    case ODY_ID_ERR_PLACEHOLDER:       return "still the shipped placeholder value";
    default:                           return "unknown";
  }
}

// -------------------------------------------------------------------------------------
//  CTA-2063-A
// -------------------------------------------------------------------------------------

// I and O are excluded so a serial cannot be misread as containing 1 or 0.
bool odyIsCtaChar(char c) {
  if (c >= '0' && c <= '9') return true;
  if (c >= 'A' && c <= 'Z') return (c != 'I' && c != 'O');
  return false;
}

// Length code: '1'..'9' means 1..9 characters, 'A'..'F' means 10..15.
static int ctaLengthFromCode(char code) {
  if (code >= '1' && code <= '9') return code - '0';
  if (code >= 'A' && code <= 'F') return 10 + (code - 'A');
  return -1;
}

static char ctaCodeFromLength(int len) {
  if (len >= 1 && len <= 9)  return (char)('0' + len);
  if (len >= 10 && len <= 15) return (char)('A' + (len - 10));
  return '\0';
}

uint8_t odyValidateCtaSerial(const char* serial) {
  if (!serial) return ODY_ID_ERR_NULL;

  const size_t n = strlen(serial);
  if (n < ODY_CTA_SERIAL_MIN || n > ODY_CTA_SERIAL_MAX) return ODY_ID_ERR_LENGTH;

  // ICAO manufacturer code: 4 characters from the permitted alphabet.
  for (int i = 0; i < ODY_CTA_MFR_CODE_LEN; ++i) {
    if (!odyIsCtaChar(serial[i])) return ODY_ID_ERR_MFR_CODE;
  }

  const int declared = ctaLengthFromCode(serial[ODY_CTA_MFR_CODE_LEN]);
  if (declared < 0) return ODY_ID_ERR_LENGTH_CODE;

  const size_t actual = n - ODY_CTA_MFR_CODE_LEN - 1;
  if ((size_t)declared != actual) return ODY_ID_ERR_LENGTH_MISMATCH;

  for (size_t i = ODY_CTA_MFR_CODE_LEN + 1; i < n; ++i) {
    if (!odyIsCtaChar(serial[i])) return ODY_ID_ERR_CHARSET;
  }
  return ODY_ID_OK;
}

uint8_t odyMakeCtaSerial(const char* mfrCode, const char* mfrSerial,
                         char* out, size_t outCap) {
  if (!mfrCode || !mfrSerial || !out) return ODY_ID_ERR_NULL;

  if (strlen(mfrCode) != ODY_CTA_MFR_CODE_LEN) return ODY_ID_ERR_MFR_CODE;
  for (int i = 0; i < ODY_CTA_MFR_CODE_LEN; ++i) {
    if (!odyIsCtaChar(mfrCode[i])) return ODY_ID_ERR_MFR_CODE;
  }

  const size_t sn = strlen(mfrSerial);
  if (sn < 1 || sn > 15) return ODY_ID_ERR_LENGTH;
  for (size_t i = 0; i < sn; ++i) {
    if (!odyIsCtaChar(mfrSerial[i])) return ODY_ID_ERR_CHARSET;
  }

  const char code = ctaCodeFromLength((int)sn);
  if (code == '\0') return ODY_ID_ERR_LENGTH_CODE;

  const size_t total = ODY_CTA_MFR_CODE_LEN + 1 + sn;
  if (outCap < total + 1) return ODY_ID_ERR_LENGTH;

  memcpy(out, mfrCode, ODY_CTA_MFR_CODE_LEN);
  out[ODY_CTA_MFR_CODE_LEN] = code;
  memcpy(out + ODY_CTA_MFR_CODE_LEN + 1, mfrSerial, sn);
  out[total] = '\0';
  return ODY_ID_OK;
}

// -------------------------------------------------------------------------------------
//  CAA registration identifier
//
//  The route for a privately built aircraft: no ICAO manufacturer code, no CTA serial.
//  Formats are national, so this validates structure only and does not pretend to know
//  every authority's scheme.
// -------------------------------------------------------------------------------------
uint8_t odyValidateCaaRegistration(const char* reg) {
  if (!reg) return ODY_ID_ERR_NULL;

  // The placeholder is checked FIRST. It is longer than the field, so a length check
  // would otherwise report "wrong length" -- true, but far less useful than telling the
  // operator they simply have not set it yet.
  if (strncmp(reg, "SET-YOUR", 8) == 0) return ODY_ID_ERR_PLACEHOLDER;

  const size_t n = strlen(reg);
  if (n == 0 || n > ODY_CAA_REG_MAX) return ODY_ID_ERR_LENGTH;

  // Alphanumerics plus the separators registrations commonly use. Anything else is
  // either a typo or would not survive the fixed-width ODID field cleanly.
  for (size_t i = 0; i < n; ++i) {
    const char c = reg[i];
    const bool ok = isalnum((unsigned char)c) || c == '-' || c == '.' || c == '_';
    if (!ok) return ODY_ID_ERR_CHARSET;
  }
  return ODY_ID_OK;
}

// -------------------------------------------------------------------------------------
//  EU operator registration number
// -------------------------------------------------------------------------------------

static int luhnCodePoint(char c) {
  const char lower = (char)tolower((unsigned char)c);
  for (int i = 0; i < kLuhnN; ++i) {
    if (kLuhnCharset[i] == lower) return i;
  }
  return -1;
}

char odyOperatorChecksum(const char* payload) {
  if (!payload) return '\0';

  // Standard Luhn mod N, right to left, factor alternating from 1.
  int factor = 1;
  int sum = 0;
  const size_t n = strlen(payload);

  for (size_t i = n; i > 0; --i) {
    const int cp = luhnCodePoint(payload[i - 1]);
    if (cp < 0) return '\0';
    int addend = factor * cp;
    factor = (factor == 2) ? 1 : 2;
    addend = (addend / kLuhnN) + (addend % kLuhnN);
    sum += addend;
  }
  return kLuhnCharset[(kLuhnN - (sum % kLuhnN)) % kLuhnN];
}

uint8_t odyValidateOperatorIdPublic(const char* publicPart, bool strict) {
  if (!publicPart) return ODY_ID_ERR_NULL;
  if (strlen(publicPart) != ODY_OPERATOR_PUBLIC_LEN) return ODY_ID_ERR_LENGTH;

  // Country prefix: three letters, conventionally the ICAO/ISO-3166 alpha-3 code
  // (DNK for Denmark).
  for (int i = 0; i < 3; ++i) {
    if (!isalpha((unsigned char)publicPart[i])) return ODY_ID_ERR_COUNTRY;
  }

  // Remaining 13 characters must be alphanumeric.
  for (int i = 3; i < ODY_OPERATOR_PUBLIC_LEN; ++i) {
    if (!isalnum((unsigned char)publicPart[i])) return ODY_ID_ERR_CHARSET;
  }

  if (strict) {
    char payload[13];
    memcpy(payload, publicPart + 3, 12);
    payload[12] = '\0';
    const char want = odyOperatorChecksum(payload);
    if (want == '\0') return ODY_ID_ERR_CHARSET;
    if (tolower((unsigned char)publicPart[15]) != (unsigned char)want) {
      return ODY_ID_ERR_CHECKSUM;
    }
  }
  return ODY_ID_OK;
}

uint8_t odySplitOperatorId(const char* full, char* publicOut, size_t publicCap,
                           char* secretOut, size_t secretCap) {
  if (!full) return ODY_ID_ERR_NULL;

  const char* dash = strchr(full, '-');
  if (!dash) return ODY_ID_ERR_SEPARATOR;

  const size_t publicLen = (size_t)(dash - full);
  if (publicLen != ODY_OPERATOR_PUBLIC_LEN) return ODY_ID_ERR_LENGTH;
  if (strlen(dash + 1) != ODY_OPERATOR_SECRET_LEN) return ODY_ID_ERR_LENGTH;

  if (publicOut) {
    if (publicCap < ODY_OPERATOR_PUBLIC_LEN + 1) return ODY_ID_ERR_LENGTH;
    memcpy(publicOut, full, ODY_OPERATOR_PUBLIC_LEN);
    publicOut[ODY_OPERATOR_PUBLIC_LEN] = '\0';
  }
  if (secretOut) {
    if (secretCap < ODY_OPERATOR_SECRET_LEN + 1) return ODY_ID_ERR_LENGTH;
    memcpy(secretOut, dash + 1, ODY_OPERATOR_SECRET_LEN);
    secretOut[ODY_OPERATOR_SECRET_LEN] = '\0';
  }
  return ODY_ID_OK;
}

uint8_t odyValidateOperatorIdFull(const char* full, bool strict) {
  char pub[ODY_OPERATOR_PUBLIC_LEN + 1];
  char sec[ODY_OPERATOR_SECRET_LEN + 1];

  const uint8_t split = odySplitOperatorId(full, pub, sizeof(pub), sec, sizeof(sec));
  if (split != ODY_ID_OK) return split;

  for (int i = 0; i < ODY_OPERATOR_SECRET_LEN; ++i) {
    if (!isalnum((unsigned char)sec[i])) return ODY_ID_ERR_CHARSET;
  }
  return odyValidateOperatorIdPublic(pub, strict);
}
