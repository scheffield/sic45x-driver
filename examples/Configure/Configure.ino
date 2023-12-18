#include <Arduino.h>
#include "SiC45x.h"

#define OK 33
#define BACK 47

SiC45x sic45x(0x0F);

void setup() {
  Serial.begin(115200);

  while(!Serial) {
    delay(10);
  }

  delay(3000);

  sic45x.begin();

  sic45x.sendClearFaults();
  sic45x.printStatusWord();

  sic45x.setOperation(
    SIC45X_OPERATION_ON_OFF_DISABLED
    | SIC45X_OPERATION_OFF_B_IMMEDIATE
    | SIC45X_OPERATION_MARGIN_COMMAND
    | SIC45X_OPERATION_MRGNFLT_FOLLOW
  );

  sic45x.setFrequencySwitch(1000);
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);

  float vout = 8;
  sic45x.setVoutOvFaultLimit(vout + 1.2);
  sic45x.setVoutOvWarnLimit(vout + 1.1);
  sic45x.setVoutUvWarnLimit(vout - 1.1);
  sic45x.setVoutUvFaultLimit(vout - 1.2);
  sic45x.setVoutUvFaultResponse(
    SIC45X_VOUT_UV_FAULT_RESPONSE_UVRSP_CONTINUE // this basically ignores faults
    | SIC45X_VOUT_UV_FAULT_RESPONSE_UVRTY_NO_RESTART
    | SIC45X_VOUT_UV_FAULT_RESPONSE_UVDLY_NO_DELAY
  );
  sic45x.setVoutScaleLoop(SIC45X_VOUT_SCALE_LOOP_5V0_12V0);
  sic45x.setVoutCommand(vout);
  sic45x.setPowerGoodOn(vout - 0.3);
  sic45x.setPowerGoodOff(vout - 0.6);

  sic45x.setVinOn(10); // 3S LiPo should not go much under 11V as per https://blog.ampow.com/lipo-voltage-chart/
  sic45x.setVinOff(9.5); // especially, let's not turn on when powered by USB
  sic45x.setVinOvFaultLimit(20);
  sic45x.setVinUvWarnLimit(10.8); // we're not gonna listen to warns, but it's a reasonable value
  sic45x.setIinOcWarnLimit(13); // taken from pcbs/buck_values.py

  sic45x.setIoutOcFaultLimit(20);
  sic45x.setIoutOcWarnLimit(17);
  sic45x.setIoutOcFaultResponse(
    SIC45X_IOUT_OC_FAULT_RESPONSE_OCRSP_CONTINUE // this basically ignores faults
    | SIC45X_IOUT_OC_FAULT_RESPONSE_OCRTY_NO_RESTART
    | SIC45X_IOUT_OC_FAULT_RESPONSE_OCDLY_NO_DELAY
  );

  sic45x.setOnOffConfiguration(
    SIC45X_ON_OFF_CONFIGURATION_PU_COMMAND
    | SIC45X_ON_OFF_CONFIGURATION_CMD_RESPOND
    | SIC45X_ON_OFF_CONFIGURATION_EN_IGNORE
    | SIC45X_ON_OFF_CONFIGURATION_ENPOL_HIGH
    | SIC45X_ON_OFF_CONFIGURATION_OFFB1_IMMEDIATE
  );

  sic45x.printStatusWord();

  // use two buttons to turn the buck on or off
  pinMode(OK, INPUT);
  pinMode(BACK, INPUT);
}

unsigned long _lastFastIntervalMs = 0;
unsigned long _lastSlowIntervalMs = 0;
bool buckEnabled = false;

void loop() {
  unsigned long current = millis();
  if (current - _lastFastIntervalMs >= 100) {
    _lastFastIntervalMs = current;

    // Check button state every 100ms
    if (digitalRead(OK) == LOW) {
      sic45x.setOperation(
        SIC45X_OPERATION_ON_OFF_ENABLED
        | SIC45X_OPERATION_OFF_B_IMMEDIATE
        | SIC45X_OPERATION_MARGIN_COMMAND
        | SIC45X_OPERATION_MRGNFLT_FOLLOW
      );
    }
    if (digitalRead(BACK) == LOW) {
      sic45x.setOperation(
        SIC45X_OPERATION_ON_OFF_DISABLED
        | SIC45X_OPERATION_OFF_B_IMMEDIATE
        | SIC45X_OPERATION_MARGIN_COMMAND
        | SIC45X_OPERATION_MRGNFLT_FOLLOW
      );
    }
  }
  if (current - _lastSlowIntervalMs >= 2000) {
    _lastSlowIntervalMs = current;

    // Report operational parameters every 2s
    Serial.print("IN: V: ");
    Serial.print(sic45x.getReadVin());
    Serial.print("V, I: ");
    Serial.print(sic45x.getReadIin());
    Serial.print("A, P: ");
    Serial.print(sic45x.getReadPin());
    Serial.print("W | OUT: V: ");
    Serial.print(sic45x.getReadVout());
    Serial.print("V, I: ");
    Serial.print(sic45x.getReadIout());
    Serial.print("A, P: ");
    Serial.print(sic45x.getReadPout());
    Serial.print("W | temp: ");
    Serial.print(sic45x.getReadTemperature());
    Serial.print("°C, duty cycle: ");
    Serial.println(sic45x.getReadDutyCycle());
  }
}
