#include "smbus.h"

#include <Arduino.h>

Smbus::Smbus(uint8_t address) : Smbus(address, Wire) {}

Smbus::Smbus(uint8_t address, TwoWire& wire) : address_(address), wire_(wire) {}

bool Smbus::begin() { 
  return wire_.begin();
}

bool Smbus::sendByte(uint8_t command) {
#ifdef SMBUS_DEBUG
  Serial.print(F("SiC45x [I]: sendByte; I2C addr: "));
  Serial.print(address_, HEX);
  Serial.print(F(", command: "));
  Serial.println(command, HEX);
#endif

  wire_.beginTransmission(address_);
  size_t writeRes = wire_.write(command);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write command"));
#endif
    wire_.endTransmission();
    return false;
  }
  uint8_t endTransmissionRes = wire_.endTransmission();
  if (endTransmissionRes != 0) {
    // from
    // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/utility/twi.c#L246-L251
    // Output   0 .. success
    //          1 .. length to long for buffer
    //          2 .. address send, NACK received
    //          3 .. data send, NACK received
    //          4 .. other twi error (lost bus arbitration, bus error, ..)
    //          5 .. timeout
#ifdef SMBUS_DEBUG
    Serial.print(F("SiC45x [E]: failed to write command; result: "));
    Serial.println(endTransmissionRes);
#endif
    return false;
  }
  return true;
}

bool Smbus::writeByte(uint8_t command, uint8_t value) {
#ifdef SMBUS_DEBUG
  Serial.print(F("SiC45x [I]: writeByte; I2C addr: "));
  Serial.print(address_, HEX);
  Serial.print(F(", command: "));
  Serial.println(command, HEX);
#endif

  wire_.beginTransmission(address_);
  size_t writeRes = wire_.write(command);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write command"));
#endif
    wire_.endTransmission();
    return false;
  }
  writeRes = wire_.write(value);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write data"));
#endif
    wire_.endTransmission();
    return false;
  }
  uint8_t endTransmissionRes = wire_.endTransmission();
  if (endTransmissionRes != 0) {
    // from
    // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/utility/twi.c#L246-L251
    // Output   0 .. success
    //          1 .. length to long for buffer
    //          2 .. address send, NACK received
    //          3 .. data send, NACK received
    //          4 .. other twi error (lost bus arbitration, bus error, ..)
    //          5 .. timeout
#ifdef SMBUS_DEBUG
    Serial.print(F("SiC45x [E]: failed to write byte; result: "));
    Serial.println(endTransmissionRes);
#endif
    return false;
  }
  return true;
}

uint8_t Smbus::readByte(uint8_t command) {
#ifdef SMBUS_DEBUG
  Serial.print(F("SiC45x [I]: readByte; I2C addr: "));
  Serial.print(address_, HEX);
  Serial.print(F(", command: "));
  Serial.println(command, HEX);
#endif

  wire_.beginTransmission(address_);
  size_t writeRes = wire_.write(command);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write command"));
#endif
    wire_.endTransmission();
    return 0;
  }
  // SMBus 3.2 under 6.5.5 specifies no stop condition after transmitting
  // command
  uint8_t writeCmdRes = wire_.endTransmission(false);
  if (writeCmdRes != 0) {
    // from
    // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/utility/twi.c#L246-L251
    // Output   0 .. success
    //          1 .. length to long for buffer
    //          2 .. address send, NACK received
    //          3 .. data send, NACK received
    //          4 .. other twi error (lost bus arbitration, bus error, ..)
    //          5 .. timeout
#ifdef SMBUS_DEBUG
    Serial.print(F("SiC45x [E]: failed to request data; result: "));
    Serial.println(writeCmdRes);
#endif
    return 0;
  }

  // Repeated start following the skipped stop condition. Will send a stop
  // condition.
  size_t requestRes = wire_.requestFrom(address_, (uint8_t)1);
  if (requestRes != 1) {
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to listen for data"));
#endif
    return 0;
  }
  int8_t data = wire_.read();
  if (data == -1) {
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to read data"));
#endif
    return 0;
  }

  return data;
}

bool Smbus::writeWord(uint8_t command, uint16_t value) {
#ifdef SMBUS_DEBUG
  Serial.print(F("SiC45x [I]: writeWord; I2C addr: "));
  Serial.print(address_, HEX);
  Serial.print(F(", command: "));
  Serial.println(command, HEX);
#endif

  wire_.beginTransmission(address_);
  size_t writeRes = wire_.write(command);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write command"));
#endif
    wire_.endTransmission();
    return false;
  }
  writeRes = wire_.write(value & 0xFF); // data byte low
  writeRes += wire_.write(value >> 8); // data byte high
  if (writeRes != 2) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write data"));
#endif
    wire_.endTransmission();
    return false;
  }
  uint8_t endTransmissionRes = wire_.endTransmission();
  if (endTransmissionRes != 0) {
    // from
    // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/utility/twi.c#L246-L251
    // Output   0 .. success
    //          1 .. length to long for buffer
    //          2 .. address send, NACK received
    //          3 .. data send, NACK received
    //          4 .. other twi error (lost bus arbitration, bus error, ..)
    //          5 .. timeout
#ifdef SMBUS_DEBUG
    Serial.print(F("SiC45x [E]: failed to write word; result: "));
    Serial.println(endTransmissionRes);
#endif
    return false;
  }
  return true;
}

uint16_t Smbus::readWord(uint8_t command) {
#ifdef SMBUS_DEBUG
  Serial.print(F("SiC45x [I]: readWord; I2C addr: "));
  Serial.print(address_, HEX);
  Serial.print(F(", command: "));
  Serial.println(command, HEX);
#endif

  wire_.beginTransmission(address_);
  size_t writeRes = wire_.write(command);
  if (writeRes != 1) {
    // depending on implementation, this can be caused by issues writing into
    // the message buffer or by issues actually transmitting the data
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to write command"));
#endif
    wire_.endTransmission();
    return 0;
  }
  // SMBus 3.2 under 6.5.5 specifies no stop condition after transmitting
  // command
  uint8_t writeCmdRes = wire_.endTransmission(false);
  if (writeCmdRes != 0) {
    // from
    // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/utility/twi.c#L246-L251
    // Output   0 .. success
    //          1 .. length to long for buffer
    //          2 .. address send, NACK received
    //          3 .. data send, NACK received
    //          4 .. other twi error (lost bus arbitration, bus error, ..)
    //          5 .. timeout
#ifdef SMBUS_DEBUG
    Serial.print(F("SiC45x [E]: failed to request data; result: "));
    Serial.println(writeCmdRes);
#endif
    return 0;
  }

  // Repeated start following the skipped stop condition. Will send a stop
  // condition.
  size_t requestRes = wire_.requestFrom(address_, (uint8_t)2);
  if (requestRes != 2) {
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to listen for data"));
#endif
    return 0;
  }
  int16_t data = wire_.read();
  data |= wire_.read() << 8;
  if (data == -1) {
#ifdef SMBUS_DEBUG
    Serial.println(F("SiC45x [E]: failed to read data"));
#endif
    return 0;
  }

  return data;
}

float Smbus::l16ToFloat(uint8_t voutMode, uint16_t inputVal) {
  // Assume Linear 16, pull out 5 bits of exponent, and use signed value.
  int8_t exponent = (int8_t) voutMode & 0x1F;

  // Sign extend exponent from 5 to 8 bits
  if (exponent > 0x0F) exponent |= 0xE0;

  // Convert mantissa to a float so we can do math.
  float mantissa = (float)inputVal;

  float value = mantissa * pow(2.0, exponent);

  return  value;
}

uint16_t Smbus::floatToL16(uint8_t voutMode, float inputVal)
{
  // Assume Linear 16, pull out 5 bits of exponent, and use signed value.
  int8_t exponent = voutMode & 0x1F;

  // Sign extend exponent from 5 to 8 bits
  if (exponent > 0x0F) exponent |= 0xE0;

  // Scale the value to a mantissa based on the exponent
  uint16_t mantissa = (uint16_t)(inputVal / pow(2.0, exponent));

  return mantissa;
}

float Smbus::l11ToFloat(uint16_t l11) {
  // Extract exponent as MS 5 bits
  int8_t exponent = (int8_t) (l11 >> 11);

  // Extract mantissa as LS 11 bits
  int16_t mantissa = l11 & 0x7ff;

  // Sign extend exponent from 5 to 8 bits
  if (exponent > 0x0F) exponent |= 0xE0;

  // Sign extend mantissa from 11 to 16 bits
  if (mantissa > 0x03FF) mantissa |= 0xF800;

  // Compute value as mantissa * 2^(exponent)
  return  mantissa * pow(2.0,(float)exponent);
}

uint16_t Smbus::floatToL11(float rawValue, int8_t exponent) {
    uint16_t mantissa = rawValue * pow(2, -exponent);
    uint16_t l11 = ((exponent & 0b00011111) << 11) | mantissa;
    return l11;
}

// x = y*2^n
//   x -> raw value
//   y -> mantissa
//   n -> exponent
// l11 = ((n & 0b00011111) << 11) | y
uint16_t Smbus::floatToL11(float rawValue) {
    // largest integer that can be represented with 11 bit two's compliment
    // equivalent to 2^10-1
    uint16_t mantissaMax = 1023;
    int8_t exponent = log2(rawValue/mantissaMax);
    return floatToL11(rawValue, exponent);
}