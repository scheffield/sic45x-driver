// This file was generated by the generateApi.js, do not edit it manually.
// Change the pmbusConfig in generateApi.js and call node generateApi.js to
// regenerate this file.

#include <Arduino.h>
  
#include "SiC45x.h"

SiC45x::SiC45x(uint8_t i2cAddress) : SiC45x(i2cAddress, Wire) {}

SiC45x::SiC45x(uint8_t i2cAddress, TwoWire& wire) : smbus_(i2cAddress, wire) {}

bool SiC45x::begin() { return smbus_.begin(); }

uint8_t SiC45x::getOperation() {
  return smbus_.readByte(SIC45X_OPERATION_CMD);
}

void SiC45x::printOperation() {
  uint8_t value = getOperation();
  Serial.println(F("Operation: "));
  Serial.print(F("  OnOff: "));
  printIfEq(value, 0b1 << 7, SIC45X_OPERATION_ON_OFF_DISABLED, F("DISABLED"));
  printIfEq(value, 0b1 << 7, SIC45X_OPERATION_ON_OFF_ENABLED, F("ENABLED"));
  Serial.println();
  Serial.print(F("  OffB: "));
  printIfEq(value, 0b1 << 6, SIC45X_OPERATION_OFF_B_IMMEDIATE, F("IMMEDIATE"));
  printIfEq(value, 0b1 << 6, SIC45X_OPERATION_OFF_B_DELAYED, F("DELAYED"));
  Serial.println();
  Serial.print(F("  Margin: "));
  printIfEq(value, 0b11 << 4, SIC45X_OPERATION_MARGIN_COMMAND, F("COMMAND"));
  printIfEq(value, 0b11 << 4, SIC45X_OPERATION_MARGIN_LOW, F("LOW"));
  printIfEq(value, 0b11 << 4, SIC45X_OPERATION_MARGIN_HIGH, F("HIGH"));
  printIfEq(value, 0b11 << 4, SIC45X_OPERATION_MARGIN_NOT_SUPPORTED, F("NOT_SUPPORTED"));
  Serial.println();
  Serial.print(F("  Mrgnflt: "));
  printIfEq(value, 0b11 << 2, SIC45X_OPERATION_MRGNFLT_NOT_SUPPORTED_1, F("NOT_SUPPORTED_1"));
  printIfEq(value, 0b11 << 2, SIC45X_OPERATION_MRGNFLT_IGNORE, F("IGNORE"));
  printIfEq(value, 0b11 << 2, SIC45X_OPERATION_MRGNFLT_FOLLOW, F("FOLLOW"));
  printIfEq(value, 0b11 << 2, SIC45X_OPERATION_MRGNFLT_NOT_SUPPORTED_2, F("NOT_SUPPORTED_2"));
  Serial.println();
}

void SiC45x::setOperation(uint8_t value) {
  smbus_.writeByte(SIC45X_OPERATION_CMD, value);
}

uint8_t SiC45x::getOnOffConfiguration() {
  return smbus_.readByte(SIC45X_ON_OFF_CONFIGURATION_CMD);
}

void SiC45x::printOnOffConfiguration() {
  uint8_t value = getOnOffConfiguration();
  Serial.println(F("OnOffConfiguration: "));
  Serial.print(F("  Pu: "));
  printIfEq(value, 0b1 << 4, SIC45X_ON_OFF_CONFIGURATION_PU_POWER, F("POWER"));
  printIfEq(value, 0b1 << 4, SIC45X_ON_OFF_CONFIGURATION_PU_COMMAND, F("COMMAND"));
  Serial.println();
  Serial.print(F("  Cmd: "));
  printIfEq(value, 0b1 << 3, SIC45X_ON_OFF_CONFIGURATION_CMD_IGNORE, F("IGNORE"));
  printIfEq(value, 0b1 << 3, SIC45X_ON_OFF_CONFIGURATION_CMD_RESPOND, F("RESPOND"));
  Serial.println();
  Serial.print(F("  En: "));
  printIfEq(value, 0b1 << 2, SIC45X_ON_OFF_CONFIGURATION_EN_IGNORE, F("IGNORE"));
  printIfEq(value, 0b1 << 2, SIC45X_ON_OFF_CONFIGURATION_EN_REQUIRE, F("REQUIRE"));
  Serial.println();
  Serial.print(F("  Enpol: "));
  printIfEq(value, 0b1 << 1, SIC45X_ON_OFF_CONFIGURATION_ENPOL_LOW, F("LOW"));
  printIfEq(value, 0b1 << 1, SIC45X_ON_OFF_CONFIGURATION_ENPOL_HIGH, F("HIGH"));
  Serial.println();
  Serial.print(F("  Offb1: "));
  printIfEq(value, 0b1 << 0, SIC45X_ON_OFF_CONFIGURATION_OFFB1_DELAYED, F("DELAYED"));
  printIfEq(value, 0b1 << 0, SIC45X_ON_OFF_CONFIGURATION_OFFB1_IMMEDIATE, F("IMMEDIATE"));
  Serial.println();
}

void SiC45x::setOnOffConfiguration(uint8_t value) {
  smbus_.writeByte(SIC45X_ON_OFF_CONFIGURATION_CMD, value);
}

void SiC45x::sendClearFaults() { smbus_.sendByte(SIC45X_CLEAR_FAULTS_CMD); }

void SiC45x::setWriteProtect(uint8_t value) {
  smbus_.writeByte(SIC45X_WRITE_PROTECT_CMD, value);
}

void SiC45x::sendStoreUserAll() { smbus_.sendByte(SIC45X_STORE_USER_ALL_CMD); }

void SiC45x::sendRestoreUserAll() { smbus_.sendByte(SIC45X_RESTORE_USER_ALL_CMD); }

uint8_t SiC45x::getCapability() {
  return smbus_.readByte(SIC45X_CAPABILITY_CMD);
}

void SiC45x::printCapability() {
  uint8_t value = getCapability();
  Serial.println(F("Capability: "));
  Serial.print(F("  Pec: "));
  printIfEq(value, 0b1 << 7, SIC45X_CAPABILITY_PEC_SUPPORTED, F("SUPPORTED"));
  Serial.println();
  Serial.print(F("  Spd: "));
  printIfEq(value, 0b11 << 5, SIC45X_CAPABILITY_SPD_M1, F("M1"));
  Serial.println();
  Serial.print(F("  Alrt: "));
  printIfEq(value, 0b1 << 4, SIC45X_CAPABILITY_ALRT_SUPPORTED, F("SUPPORTED"));
  Serial.println();
  Serial.print(F("  Nfmt: "));
  printIfEq(value, 0b1 << 3, SIC45X_CAPABILITY_NFMT_L11_L16_DIRECT, F("L11_L16_DIRECT"));
  Serial.println();
  Serial.print(F("  Avs: "));
  printIfEq(value, 0b1 << 3, SIC45X_CAPABILITY_AVS_NOT_SUPPORTED, F("NOT_SUPPORTED"));
  Serial.println();
}

float SiC45x::getVoutMode() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_MODE_CMD));
}

float SiC45x::getVoutCommand() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_COMMAND_CMD));
}

void SiC45x::setVoutCommand(float value) {
  smbus_.writeWord(SIC45X_VOUT_COMMAND_CMD, floatToL16(value));
}

float SiC45x::getVoutMax() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_MAX_CMD));
}

void SiC45x::setVoutMax(float value) {
  smbus_.writeWord(SIC45X_VOUT_MAX_CMD, floatToL16(value));
}

float SiC45x::getVoutMarginHigh() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_MARGIN_HIGH_CMD));
}

void SiC45x::setVoutMarginHigh(float value) {
  smbus_.writeWord(SIC45X_VOUT_MARGIN_HIGH_CMD, floatToL16(value));
}

float SiC45x::getVoutMarginLow() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_MARGIN_LOW_CMD));
}

void SiC45x::setVoutMarginLow(float value) {
  smbus_.writeWord(SIC45X_VOUT_MARGIN_LOW_CMD, floatToL16(value));
}

float SiC45x::getVoutTransitionRate() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VOUT_TRANSITION_RATE_CMD));
}

void SiC45x::setVoutTransitionRate(float value) {
  smbus_.writeWord(SIC45X_VOUT_TRANSITION_RATE_CMD, smbus_.floatToL11(value, SIC45X_VOUT_TRANSITION_RATE_EXP));
}

float SiC45x::getVoutScaleLoop() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VOUT_SCALE_LOOP_CMD));
}

void SiC45x::setVoutScaleLoop(float value) {
  smbus_.writeWord(SIC45X_VOUT_SCALE_LOOP_CMD, smbus_.floatToL11(value, SIC45X_VOUT_SCALE_LOOP_EXP));
}

float SiC45x::getFrequencySwitch() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_FREQUENCY_SWITCH_CMD));
}

void SiC45x::setFrequencySwitch(float value) {
  smbus_.writeWord(SIC45X_FREQUENCY_SWITCH_CMD, smbus_.floatToL11(value, SIC45X_FREQUENCY_SWITCH_EXP));
}

float SiC45x::getVinOn() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VIN_ON_CMD));
}

void SiC45x::setVinOn(float value) {
  smbus_.writeWord(SIC45X_VIN_ON_CMD, smbus_.floatToL11(value, SIC45X_VIN_ON_EXP));
}

float SiC45x::getVinOff() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VIN_OFF_CMD));
}

void SiC45x::setVinOff(float value) {
  smbus_.writeWord(SIC45X_VIN_OFF_CMD, smbus_.floatToL11(value, SIC45X_VIN_OFF_EXP));
}

uint16_t SiC45x::getInterleave() {
  return smbus_.readWord(SIC45X_INTERLEAVE_CMD);
}

void SiC45x::printInterleave() {
  uint16_t value = getInterleave();
  Serial.println(F("Interleave: "));
  Serial.print(F("  Mode: "));
  printIfEq(value, 0b1111111111111111 << 0, SIC45X_INTERLEAVE_MODE_STANDALONE, F("STANDALONE"));
  printIfEq(value, 0b1111111111111111 << 0, SIC45X_INTERLEAVE_MODE_MASTER, F("MASTER"));
  printIfEq(value, 0b1111111111111111 << 0, SIC45X_INTERLEAVE_MODE_SLAVE_IN_PHASE, F("SLAVE_IN_PHASE"));
  printIfEq(value, 0b1111111111111111 << 0, SIC45X_INTERLEAVE_MODE_SLAVE_OUT_OF_PHASE, F("SLAVE_OUT_OF_PHASE"));
  Serial.println();
}

void SiC45x::setInterleave(uint16_t value) {
  smbus_.writeWord(SIC45X_INTERLEAVE_CMD, value);
}

float SiC45x::getVoutOvFaultLimit() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_OV_FAULT_LIMIT_CMD));
}

void SiC45x::setVoutOvFaultLimit(float value) {
  smbus_.writeWord(SIC45X_VOUT_OV_FAULT_LIMIT_CMD, floatToL16(value));
}

uint8_t SiC45x::getVoutOvFaultResponse() {
  return smbus_.readByte(SIC45X_VOUT_OV_FAULT_RESPONSE_CMD);
}

void SiC45x::printVoutOvFaultResponse() {
  uint8_t value = getVoutOvFaultResponse();
  Serial.println(F("VoutOvFaultResponse: "));
  Serial.print(F("  Ovrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRSP_CONTINUE, F("CONTINUE"));
  printIfEq(value, 0b11 << 6, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY, F("DISABLED_WHILE_FAULTY"));
  Serial.println();
  Serial.print(F("  Ovrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_RETRIES_2, F("RETRIES_2"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_RETRIES_4, F("RETRIES_4"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_RETRIES_6, F("RETRIES_6"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_OV_FAULT_RESPONSE_OVRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Ovdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_OV_FAULT_RESPONSE_OVDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_OV_FAULT_RESPONSE_OVDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_OV_FAULT_RESPONSE_OVDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setVoutOvFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_VOUT_OV_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getVoutOvWarnLimit() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_OV_WARN_LIMIT_CMD));
}

void SiC45x::setVoutOvWarnLimit(float value) {
  smbus_.writeWord(SIC45X_VOUT_OV_WARN_LIMIT_CMD, floatToL16(value));
}

float SiC45x::getVoutUvWarnLimit() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_UV_WARN_LIMIT_CMD));
}

void SiC45x::setVoutUvWarnLimit(float value) {
  smbus_.writeWord(SIC45X_VOUT_UV_WARN_LIMIT_CMD, floatToL16(value));
}

float SiC45x::getVoutUvFaultLimit() {
  return l16ToFloat(smbus_.readWord(SIC45X_VOUT_UV_FAULT_LIMIT_CMD));
}

void SiC45x::setVoutUvFaultLimit(float value) {
  smbus_.writeWord(SIC45X_VOUT_UV_FAULT_LIMIT_CMD, floatToL16(value));
}

uint8_t SiC45x::getVoutUvFaultResponse() {
  return smbus_.readByte(SIC45X_VOUT_UV_FAULT_RESPONSE_CMD);
}

void SiC45x::printVoutUvFaultResponse() {
  uint8_t value = getVoutUvFaultResponse();
  Serial.println(F("VoutUvFaultResponse: "));
  Serial.print(F("  Uvrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_CONTINUE, F("CONTINUE"));
  printIfEq(value, 0b11 << 6, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_RETRY, F("RETRY"));
  printIfEq(value, 0b11 << 6, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_DISABLED_WHILE_FAULTY, F("DISABLED_WHILE_FAULTY"));
  Serial.println();
  Serial.print(F("  Uvrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_RETRIES_1, F("RETRIES_1"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_RETRIES_2, F("RETRIES_2"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_RETRIES_6, F("RETRIES_6"));
  printIfEq(value, 0b111 << 3, SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Uvdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setVoutUvFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_VOUT_UV_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getIoutOcFaultLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_IOUT_OC_FAULT_LIMIT_CMD));
}

void SiC45x::setIoutOcFaultLimit(float value) {
  smbus_.writeWord(SIC45X_IOUT_OC_FAULT_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_IOUT_OC_FAULT_LIMIT_EXP));
}

uint8_t SiC45x::getIoutOcFaultResponse() {
  return smbus_.readByte(SIC45X_IOUT_OC_FAULT_RESPONSE_CMD);
}

void SiC45x::printIoutOcFaultResponse() {
  uint8_t value = getIoutOcFaultResponse();
  Serial.println(F("IoutOcFaultResponse: "));
  Serial.print(F("  Ocrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_CONTINUE, F("CONTINUE"));
  printIfEq(value, 0b11 << 6, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_RETRY, F("RETRY"));
  printIfEq(value, 0b11 << 6, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_DISABLED_WHILE_FAULTY, F("DISABLED_WHILE_FAULTY"));
  Serial.println();
  Serial.print(F("  Ocrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_RETRIES_2, F("RETRIES_2"));
  printIfEq(value, 0b111 << 3, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_RETRIES_4, F("RETRIES_4"));
  printIfEq(value, 0b111 << 3, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_RETRIES_6, F("RETRIES_6"));
  printIfEq(value, 0b111 << 3, SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Ocdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_IOUT_OC_FAULT_RESPONSE_OCDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_IOUT_OC_FAULT_RESPONSE_OCDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_IOUT_OC_FAULT_RESPONSE_OCDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setIoutOcFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_IOUT_OC_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getIoutOcWarnLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_IOUT_OC_WARN_LIMIT_CMD));
}

void SiC45x::setIoutOcWarnLimit(float value) {
  smbus_.writeWord(SIC45X_IOUT_OC_WARN_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_IOUT_OC_WARN_LIMIT_EXP));
}

float SiC45x::getOtFaultLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_OT_FAULT_LIMIT_CMD));
}

void SiC45x::setOtFaultLimit(float value) {
  smbus_.writeWord(SIC45X_OT_FAULT_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_OT_FAULT_LIMIT_EXP));
}

uint8_t SiC45x::getOtFaultResponse() {
  return smbus_.readByte(SIC45X_OT_FAULT_RESPONSE_CMD);
}

void SiC45x::printOtFaultResponse() {
  uint8_t value = getOtFaultResponse();
  Serial.println(F("OtFaultResponse: "));
  Serial.print(F("  Otrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_OT_FAULT_RESPONSE_OTRSP_CONTINUE, F("CONTINUE"));
  printIfEq(value, 0b11 << 6, SIC45X_OT_FAULT_RESPONSE_OTRSP_RETRY, F("RETRY"));
  printIfEq(value, 0b11 << 6, SIC45X_OT_FAULT_RESPONSE_OTRSP_DISABLED_WHILE_FAULTY, F("DISABLED_WHILE_FAULTY"));
  Serial.println();
  Serial.print(F("  Otrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_OT_FAULT_RESPONSE_OTRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_OT_FAULT_RESPONSE_OTRTY_RETRIES_2, F("RETRIES_2"));
  printIfEq(value, 0b111 << 3, SIC45X_OT_FAULT_RESPONSE_OTRTY_RETRIES_4, F("RETRIES_4"));
  printIfEq(value, 0b111 << 3, SIC45X_OT_FAULT_RESPONSE_OTRTY_RETRIES_6, F("RETRIES_6"));
  printIfEq(value, 0b111 << 3, SIC45X_OT_FAULT_RESPONSE_OTRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Otdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_OT_FAULT_RESPONSE_OTDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_OT_FAULT_RESPONSE_OTDLY_DELAY_1, F("DELAY_1"));
  printIfEq(value, 0b111 << 0, SIC45X_OT_FAULT_RESPONSE_OTDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_OT_FAULT_RESPONSE_OTDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setOtFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_OT_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getOtWarnLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_OT_WARN_LIMIT_CMD));
}

void SiC45x::setOtWarnLimit(float value) {
  smbus_.writeWord(SIC45X_OT_WARN_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_OT_WARN_LIMIT_EXP));
}

float SiC45x::getVinOvFaultLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VIN_OV_FAULT_LIMIT_CMD));
}

void SiC45x::setVinOvFaultLimit(float value) {
  smbus_.writeWord(SIC45X_VIN_OV_FAULT_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_VIN_OV_FAULT_LIMIT_EXP));
}

uint8_t SiC45x::getVinOvFaultResponse() {
  return smbus_.readByte(SIC45X_VIN_OV_FAULT_RESPONSE_CMD);
}

void SiC45x::printVinOvFaultResponse() {
  uint8_t value = getVinOvFaultResponse();
  Serial.println(F("VinOvFaultResponse: "));
  Serial.print(F("  Ovrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_VIN_OV_FAULT_RESPONSE_OVRSP_CONTINUE, F("CONTINUE"));
  printIfEq(value, 0b11 << 6, SIC45X_VIN_OV_FAULT_RESPONSE_OVRSP_DISABLED_WHILE_FAULTY, F("DISABLED_WHILE_FAULTY"));
  Serial.println();
  Serial.print(F("  Ovrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_RETRIES_2, F("RETRIES_2"));
  printIfEq(value, 0b111 << 3, SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_RETRIES_4, F("RETRIES_4"));
  printIfEq(value, 0b111 << 3, SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_RETRIES_6, F("RETRIES_6"));
  printIfEq(value, 0b111 << 3, SIC45X_VIN_OV_FAULT_RESPONSE_OVRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Ovdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_VIN_OV_FAULT_RESPONSE_OVDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_VIN_OV_FAULT_RESPONSE_OVDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_VIN_OV_FAULT_RESPONSE_OVDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setVinOvFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_VIN_OV_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getVinUvWarnLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_VIN_UV_WARN_LIMIT_CMD));
}

void SiC45x::setVinUvWarnLimit(float value) {
  smbus_.writeWord(SIC45X_VIN_UV_WARN_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_VIN_UV_WARN_LIMIT_EXP));
}

float SiC45x::getIinOcWarnLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_IIN_OC_WARN_LIMIT_CMD));
}

void SiC45x::setIinOcWarnLimit(float value) {
  smbus_.writeWord(SIC45X_IIN_OC_WARN_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_IIN_OC_WARN_LIMIT_EXP));
}

float SiC45x::getPowerGoodOn() {
  return l16ToFloat(smbus_.readWord(SIC45X_POWER_GOOD_ON_CMD));
}

void SiC45x::setPowerGoodOn(float value) {
  smbus_.writeWord(SIC45X_POWER_GOOD_ON_CMD, floatToL16(value));
}

float SiC45x::getPowerGoodOff() {
  return l16ToFloat(smbus_.readWord(SIC45X_POWER_GOOD_OFF_CMD));
}

void SiC45x::setPowerGoodOff(float value) {
  smbus_.writeWord(SIC45X_POWER_GOOD_OFF_CMD, floatToL16(value));
}

float SiC45x::getTonDelay() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TON_DELAY_CMD));
}

void SiC45x::setTonDelay(float value) {
  smbus_.writeWord(SIC45X_TON_DELAY_CMD, smbus_.floatToL11(value, SIC45X_TON_DELAY_EXP));
}

float SiC45x::getTonRise() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TON_RISE_CMD));
}

void SiC45x::setTonRise(float value) {
  smbus_.writeWord(SIC45X_TON_RISE_CMD, smbus_.floatToL11(value, SIC45X_TON_RISE_EXP));
}

float SiC45x::getTonMaxFaultLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TON_MAX_FAULT_LIMIT_CMD));
}

void SiC45x::setTonMaxFaultLimit(float value) {
  smbus_.writeWord(SIC45X_TON_MAX_FAULT_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_TON_MAX_FAULT_LIMIT_EXP));
}

uint8_t SiC45x::getTonMaxFaultResponse() {
  return smbus_.readByte(SIC45X_TON_MAX_FAULT_RESPONSE_CMD);
}

void SiC45x::printTonMaxFaultResponse() {
  uint8_t value = getTonMaxFaultResponse();
  Serial.println(F("TonMaxFaultResponse: "));
  Serial.print(F("  Onmxrsp: "));
  printIfEq(value, 0b11 << 6, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXRSP_RETRY, F("RETRY"));
  Serial.println();
  Serial.print(F("  Onmxrty: "));
  printIfEq(value, 0b111 << 3, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXRTY_NO_RESTART, F("NO_RESTART"));
  printIfEq(value, 0b111 << 3, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXRTY_RETRIES_1, F("RETRIES_1"));
  printIfEq(value, 0b111 << 3, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXRTY_RETRIES, F("RETRIES"));
  Serial.println();
  Serial.print(F("  Onmxdly: "));
  printIfEq(value, 0b111 << 0, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXDLY_NO_DELAY, F("NO_DELAY"));
  printIfEq(value, 0b111 << 0, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXDLY_DELAY_1, F("DELAY_1"));
  printIfEq(value, 0b111 << 0, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXDLY_DELAY_2, F("DELAY_2"));
  printIfEq(value, 0b111 << 0, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXDLY_DELAY_3, F("DELAY_3"));
  printIfEq(value, 0b111 << 0, SIC45X_TON_MAX_FAULT_RESPONSE_ONMXDLY_DELAY_6, F("DELAY_6"));
  Serial.println();
}

void SiC45x::setTonMaxFaultResponse(uint8_t value) {
  smbus_.writeByte(SIC45X_TON_MAX_FAULT_RESPONSE_CMD, value);
}

float SiC45x::getToffDelay() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TOFF_DELAY_CMD));
}

void SiC45x::setToffDelay(float value) {
  smbus_.writeWord(SIC45X_TOFF_DELAY_CMD, smbus_.floatToL11(value, SIC45X_TOFF_DELAY_EXP));
}

float SiC45x::getToffFall() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TOFF_FALL_CMD));
}

void SiC45x::setToffFall(float value) {
  smbus_.writeWord(SIC45X_TOFF_FALL_CMD, smbus_.floatToL11(value, SIC45X_TOFF_FALL_EXP));
}

float SiC45x::getToffMaxWarnLimit() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_TOFF_MAX_WARN_LIMIT_CMD));
}

void SiC45x::setToffMaxWarnLimit(float value) {
  smbus_.writeWord(SIC45X_TOFF_MAX_WARN_LIMIT_CMD, smbus_.floatToL11(value, SIC45X_TOFF_MAX_WARN_LIMIT_EXP));
}

uint8_t SiC45x::getStatusByte() {
  return smbus_.readByte(SIC45X_STATUS_BYTE_CMD);
}

void SiC45x::printStatusByte() {
  uint8_t value = getStatusByte();
  Serial.println(F("StatusByte: "));
  Serial.print(F("  Busy: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_BYTE_BUSY_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_BYTE_BUSY_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Off: "));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_BYTE_OFF_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_BYTE_OFF_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VoutOvFault: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_BYTE_VOUT_OV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_BYTE_VOUT_OV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  IoutUvFault: "));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_BYTE_IOUT_UV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_BYTE_IOUT_UV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VinUvFault: "));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_BYTE_VIN_UV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_BYTE_VIN_UV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Temperature: "));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_BYTE_TEMPERATURE_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_BYTE_TEMPERATURE_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Cml: "));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_BYTE_CML_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_BYTE_CML_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Other: "));
  printIfEq(value, 0b1 << 0, SIC45X_STATUS_BYTE_OTHER_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 0, SIC45X_STATUS_BYTE_OTHER_FAULT, F("FAULT"));
  Serial.println();
}

uint16_t SiC45x::getStatusWord() {
  return smbus_.readWord(SIC45X_STATUS_WORD_CMD);
}

void SiC45x::printStatusWord() {
  uint16_t value = getStatusWord();
  Serial.println(F("StatusWord: "));
  Serial.print(F("  Busy: "));
  printIfEq(value, 0b1 << 15, SIC45X_STATUS_WORD_BUSY_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 15, SIC45X_STATUS_WORD_BUSY_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Off: "));
  printIfEq(value, 0b1 << 14, SIC45X_STATUS_WORD_OFF_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 14, SIC45X_STATUS_WORD_OFF_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VoutOvFault: "));
  printIfEq(value, 0b1 << 13, SIC45X_STATUS_WORD_VOUT_OV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 13, SIC45X_STATUS_WORD_VOUT_OV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  IoutUvFault: "));
  printIfEq(value, 0b1 << 12, SIC45X_STATUS_WORD_IOUT_UV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 12, SIC45X_STATUS_WORD_IOUT_UV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VinUvFault: "));
  printIfEq(value, 0b1 << 11, SIC45X_STATUS_WORD_VIN_UV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 11, SIC45X_STATUS_WORD_VIN_UV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Temperature: "));
  printIfEq(value, 0b1 << 10, SIC45X_STATUS_WORD_TEMPERATURE_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 10, SIC45X_STATUS_WORD_TEMPERATURE_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Cml: "));
  printIfEq(value, 0b1 << 9, SIC45X_STATUS_WORD_CML_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 9, SIC45X_STATUS_WORD_CML_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Other: "));
  printIfEq(value, 0b1 << 8, SIC45X_STATUS_WORD_OTHER_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 8, SIC45X_STATUS_WORD_OTHER_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Vout: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_WORD_VOUT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_WORD_VOUT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  IoutPout: "));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_WORD_IOUT_POUT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_WORD_IOUT_POUT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Input: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_WORD_INPUT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_WORD_INPUT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Mfr: "));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_WORD_MFR_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_WORD_MFR_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  PowerGood: "));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_WORD_POWER_GOOD_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_WORD_POWER_GOOD_FAULT, F("FAULT"));
  Serial.println();
}

uint8_t SiC45x::getStatusVout() {
  return smbus_.readByte(SIC45X_STATUS_VOUT_CMD);
}

void SiC45x::printStatusVout() {
  uint8_t value = getStatusVout();
  Serial.println(F("StatusVout: "));
  Serial.print(F("  VoutOvFault: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_VOUT_VOUT_OV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_VOUT_VOUT_OV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VoutOvWarning: "));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_VOUT_VOUT_OV_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_VOUT_VOUT_OV_WARNING_WARNING, F("WARNING"));
  Serial.println();
  Serial.print(F("  VoutUvWarning: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_VOUT_VOUT_UV_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_VOUT_VOUT_UV_WARNING_WARNING, F("WARNING"));
  Serial.println();
  Serial.print(F("  VoutUvFault: "));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_VOUT_VOUT_UV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_VOUT_VOUT_UV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VoutMaxMin: "));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_VOUT_VOUT_MAX_MIN_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_VOUT_VOUT_MAX_MIN_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  TonMaxFault: "));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_VOUT_TON_MAX_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_VOUT_TON_MAX_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  ToffMaxWarning: "));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_VOUT_TOFF_MAX_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_VOUT_TOFF_MAX_WARNING_WARNING, F("WARNING"));
  Serial.println();
}

uint8_t SiC45x::getStatusIout() {
  return smbus_.readByte(SIC45X_STATUS_IOUT_CMD);
}

void SiC45x::printStatusIout() {
  uint8_t value = getStatusIout();
  Serial.println(F("StatusIout: "));
  Serial.print(F("  IoutOcFault: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_IOUT_IOUT_OC_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_IOUT_IOUT_OC_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  IoutOcWarning: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_IOUT_IOUT_OC_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_IOUT_IOUT_OC_WARNING_WARNING, F("WARNING"));
  Serial.println();
}

uint8_t SiC45x::getStatusInput() {
  return smbus_.readByte(SIC45X_STATUS_INPUT_CMD);
}

void SiC45x::printStatusInput() {
  uint8_t value = getStatusInput();
  Serial.println(F("StatusInput: "));
  Serial.print(F("  VinOvFault: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_INPUT_VIN_OV_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_INPUT_VIN_OV_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  VinUvWarning: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_INPUT_VIN_UV_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_INPUT_VIN_UV_WARNING_WARNING, F("WARNING"));
  Serial.println();
  Serial.print(F("  VinOff: "));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_INPUT_VIN_OFF_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_INPUT_VIN_OFF_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  IinOcWarning: "));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_INPUT_IIN_OC_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_INPUT_IIN_OC_WARNING_WARNING, F("WARNING"));
  Serial.println();
}

uint8_t SiC45x::getStatusTemperature() {
  return smbus_.readByte(SIC45X_STATUS_TEMPERATURE_CMD);
}

void SiC45x::printStatusTemperature() {
  uint8_t value = getStatusTemperature();
  Serial.println(F("StatusTemperature: "));
  Serial.print(F("  OtFault: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_TEMPERATURE_OT_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_TEMPERATURE_OT_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  OtWarning: "));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_TEMPERATURE_OT_WARNING_NO_WARNING, F("NO_WARNING"));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_TEMPERATURE_OT_WARNING_WARNING, F("WARNING"));
  Serial.println();
}

uint8_t SiC45x::getStatusCml() {
  return smbus_.readByte(SIC45X_STATUS_CML_CMD);
}

void SiC45x::printStatusCml() {
  uint8_t value = getStatusCml();
  Serial.println(F("StatusCml: "));
  Serial.print(F("  InvalidCommand: "));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_CML_INVALID_COMMAND_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 7, SIC45X_STATUS_CML_INVALID_COMMAND_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  InvalidData: "));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_CML_INVALID_DATA_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 6, SIC45X_STATUS_CML_INVALID_DATA_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  PecFailed: "));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_CML_PEC_FAILED_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 5, SIC45X_STATUS_CML_PEC_FAILED_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  MemoryFault: "));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_CML_MEMORY_FAULT_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 4, SIC45X_STATUS_CML_MEMORY_FAULT_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  Other: "));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_CML_OTHER_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_CML_OTHER_FAULT, F("FAULT"));
  Serial.println();
}

uint8_t SiC45x::getStatusMfrSpecific() {
  return smbus_.readByte(SIC45X_STATUS_MFR_SPECIFIC_CMD);
}

void SiC45x::printStatusMfrSpecific() {
  uint8_t value = getStatusMfrSpecific();
  Serial.println(F("StatusMfrSpecific: "));
  Serial.print(F("  IlMaster: "));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_MFR_SPECIFIC_IL_MASTER_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 3, SIC45X_STATUS_MFR_SPECIFIC_IL_MASTER_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  YfVerify: "));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_MFR_SPECIFIC_YF_VERIFY_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 2, SIC45X_STATUS_MFR_SPECIFIC_YF_VERIFY_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  YfErase: "));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_MFR_SPECIFIC_YF_ERASE_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 1, SIC45X_STATUS_MFR_SPECIFIC_YF_ERASE_FAULT, F("FAULT"));
  Serial.println();
  Serial.print(F("  YfPgm: "));
  printIfEq(value, 0b1 << 0, SIC45X_STATUS_MFR_SPECIFIC_YF_PGM_NO_FAULT, F("NO_FAULT"));
  printIfEq(value, 0b1 << 0, SIC45X_STATUS_MFR_SPECIFIC_YF_PGM_FAULT, F("FAULT"));
  Serial.println();
}

float SiC45x::getReadVin() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_VIN_CMD));
}

float SiC45x::getReadIin() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_IIN_CMD));
}

float SiC45x::getReadVout() {
  return l16ToFloat(smbus_.readWord(SIC45X_READ_VOUT_CMD));
}

float SiC45x::getReadIout() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_IOUT_CMD));
}

float SiC45x::getReadTemperature() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_TEMPERATURE_CMD));
}

float SiC45x::getReadDutyCycle() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_DUTY_CYCLE_CMD));
}

float SiC45x::getReadPout() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_POUT_CMD));
}

float SiC45x::getReadPin() {
  return smbus_.l11ToFloat(smbus_.readWord(SIC45X_READ_PIN_CMD));
}

uint8_t SiC45x::getPmbusRevision() {
  return smbus_.readByte(SIC45X_PMBUS_REVISION_CMD);
}

void SiC45x::printPmbusRevision() {
  uint8_t value = getPmbusRevision();
  Serial.println(F("PmbusRevision: "));
  Serial.print(F("  Part1: "));
  printIfEq(value, 0b1111 << 4, SIC45X_PMBUS_REVISION_PART_1_REVISION_1_0, F("REVISION_1_0"));
  printIfEq(value, 0b1111 << 4, SIC45X_PMBUS_REVISION_PART_1_REVISION_1_1, F("REVISION_1_1"));
  printIfEq(value, 0b1111 << 4, SIC45X_PMBUS_REVISION_PART_1_REVISION_1_2, F("REVISION_1_2"));
  printIfEq(value, 0b1111 << 4, SIC45X_PMBUS_REVISION_PART_1_REVISION_1_3, F("REVISION_1_3"));
  Serial.println();
  Serial.print(F("  Part2: "));
  printIfEq(value, 0b1111 << 0, SIC45X_PMBUS_REVISION_PART_2_REVISION_1_0, F("REVISION_1_0"));
  printIfEq(value, 0b1111 << 0, SIC45X_PMBUS_REVISION_PART_2_REVISION_1_1, F("REVISION_1_1"));
  printIfEq(value, 0b1111 << 0, SIC45X_PMBUS_REVISION_PART_2_REVISION_1_2, F("REVISION_1_2"));
  printIfEq(value, 0b1111 << 0, SIC45X_PMBUS_REVISION_PART_2_REVISION_1_3, F("REVISION_1_3"));
  Serial.println();
}

uint16_t SiC45x::getMfrSerial() {
  return smbus_.readWord(SIC45X_MFR_SERIAL_CMD);
}

void SiC45x::setMfrSerial(uint16_t value) {
  smbus_.writeWord(SIC45X_MFR_SERIAL_CMD, value);
}

uint16_t SiC45x::getIcDeviceId() {
  return smbus_.readWord(SIC45X_IC_DEVICE_ID_CMD);
}

void SiC45x::setIcDeviceId(uint16_t value) {
  smbus_.writeWord(SIC45X_IC_DEVICE_ID_CMD, value);
}

float SiC45x::l16ToFloat(uint16_t inputVal) {
  return smbus_.l16ToFloat(0x17, inputVal);
}

uint16_t SiC45x::floatToL16(float inputVal) {
  return smbus_.floatToL16(0x17, inputVal);
}