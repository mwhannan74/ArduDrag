#pragma once
#include <Arduino.h>

// Helper functions to parse PMT ACK message to check if we set correct modes
namespace GpsHelper {

// If true, prints \r and \n as tokens and prints hex for non-printables.
// If false, prints raw bytes (Serial.write).
static const bool kReadableEchoDefault = true;

// Print one char in a readable way (optional)
inline void printCharReadable(usb_serial_class& Serial, char c)
{
  if (c == '\r') { Serial.print("\\r"); return; }
  if (c == '\n') { Serial.print("\\n\n"); return; }
  if (c >= 32 && c <= 126) { Serial.write((uint8_t)c); return; }

  Serial.print("<0x");
  uint8_t u = (uint8_t)c;
  if (u < 16) Serial.print("0");
  Serial.print(u, HEX);
  Serial.print(">");
}

// Drain any pending GPS input (so we don't match an old ACK)
inline void flushGPS(HardwareSerial& SerialGPS)
{
  while (SerialGPS.available() > 0) (void)SerialGPS.read();
}

// Echo bytes from GPS for a short time (useful for debugging)
inline void echoGPS(usb_serial_class& Serial,
                    HardwareSerial& SerialGPS,
                    uint32_t duration_ms,
                    bool readable = kReadableEchoDefault)
{
  uint32_t start = millis();
  while (millis() - start < duration_ms)
  {
    while (SerialGPS.available() > 0)
    {
      char c = (char)SerialGPS.read();
      if (readable) printCharReadable(Serial, c);
      else Serial.write((uint8_t)c);
    }
  }
}

// Read one line (up to '\n') from GPS into buf (strips '\r' and '\n').
// Optionally echoes all received characters to Serial.
inline bool readLine(usb_serial_class& Serial,
                     HardwareSerial& SerialGPS,
                     char* buf,
                     size_t bufLen,
                     uint32_t timeout_ms,
                     bool echoChars = true,
                     bool readableEcho = kReadableEchoDefault)
{
  if (!buf || bufLen < 2) return false;

  uint32_t start = millis();
  size_t idx = 0;

  while (millis() - start < timeout_ms)
  {
    while (SerialGPS.available() > 0)
    {
      char c = (char)SerialGPS.read();

      if (echoChars)
      {
        if (readableEcho) printCharReadable(Serial, c);
        else Serial.write((uint8_t)c);
      }

      if (c == '\r') continue;

      if (c == '\n')
      {
        buf[idx] = '\0';
        return (idx > 0);
      }

      if (idx < bufLen - 1) buf[idx++] = c;
      else
      {
        // line too long; terminate and return what we have
        buf[bufLen - 1] = '\0';
        return true;
      }
    }
  }

  return false; // timeout
}

// Wait for a PMTK001 ACK for a specific command ID (e.g., 886).
// Prints the ACK line when received (since readLine echoes chars).
// Returns true if ACK flag == 3 (success). Writes flag into outFlag if non-null.
inline bool waitForPMTKAck(usb_serial_class& Serial,
                           HardwareSerial& SerialGPS,
                           uint16_t cmdId,
                           uint32_t timeout_ms,
                           int* outFlag = nullptr,
                           bool echoChars = true,
                           bool readableEcho = kReadableEchoDefault)
{
  char line[128];
  uint32_t start = millis();

  // Expected prefix: $PMTK001,<cmdId>,
  char prefix[24];
  snprintf(prefix, sizeof(prefix), "$PMTK001,%u,", cmdId);
  size_t prefixLen = strlen(prefix);

  while (millis() - start < timeout_ms)
  {
    uint32_t remaining = timeout_ms - (millis() - start);
    if (!readLine(Serial, SerialGPS, line, sizeof(line), remaining, echoChars, readableEcho))
      continue;

    // At this point, 'line' holds one full line without \r\n.
    if (strncmp(line, prefix, prefixLen) == 0)
    {
      int flag = atoi(line + prefixLen);
      if (outFlag) *outFlag = flag;

      // // Print a clean summary line too (in addition to raw echo)
      // Serial.print("ACK for PMTK");
      // Serial.print(cmdId);
      // Serial.print(": ");
      // Serial.print(line);
      // Serial.print("  (flag=");
      // Serial.print(flag);
      // Serial.println(")");

      return (flag == 3);
    }
    // else: it's some other line; keep looping
  }

  if (outFlag) *outFlag = -1;
  //Serial.print("ACK for PMTK");
  //Serial.print(cmdId);
  //Serial.println(": TIMEOUT");
  return false;
}

} // namespace GpsHelper

