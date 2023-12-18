#ifndef SMBUS_H_
#define SMBUS_H_

#include <Wire.h>

class Smbus {
 public:
  Smbus(uint8_t address);
  Smbus(uint8_t address, TwoWire& wire);

  bool begin();

  bool sendByte(uint8_t command);
  bool writeByte(uint8_t command, uint8_t value);
  uint8_t readByte(uint8_t command);
  bool writeWord(uint8_t command, uint16_t value);
  uint16_t readWord(uint8_t command);

 	float l16ToFloat(uint8_t voutMode, uint16_t inputVal);
	uint16_t floatToL16(uint8_t voutMode, float inputVal);

	float l11ToFloat(uint16_t l11);
	uint16_t floatToL11(float rawValue, int8_t exponent);
	uint16_t floatToL11(float rawValue);
 private:
  TwoWire& wire_;
  uint8_t address_;
};

#endif