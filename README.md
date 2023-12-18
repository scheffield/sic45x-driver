# SiC45x

A library to talk to the SiC45x series buck converters using I2C (Wire). It provides implementations for almost all features of the [SiC45x](https://www.vishay.com/en/product/77863/) series buck converters as described in the [datasheet](https://www.vishay.com/docs/77863/sic450_sic451_sic453.pdf). It is designed to be used with the Arduino Wire library.

## Usage
Usag is straigt forward. A simple example looks as follows

```cpp
#include "SiC45x.h"

SiC45x sic45x(0x0F);

void setup() {
  Serial.begin(115200);

  sic45x.begin();

  sic45x.setFrequencySwitch(1000);
  sic45x.setInterleave(SIC45X_INTERLEAVE_MODE_MASTER);
  sic45x.setVoutCommand(.5);

  sic45x.printStatusWord();

  sic45x.setOperation(
    SIC45X_OPERATION_ON_OFF_ENABLED
    | SIC45X_OPERATION_OFF_B_IMMEDIATE
    | SIC45X_OPERATION_MARGIN_COMMAND
    | SIC45X_OPERATION_MRGNFLT_FOLLOW
  );
}

void loop() {
  Serial.print(F("I_OUT: "))
  Serial.print(sic45x.getReadIout());
  Serial.println(F("A"))

  delay(1000);
}
```

A more complete setup can be found in the example sketch.

## Regenerate API
`SiC45x.h` and `SiC45x.cpp` are generated from `/extras/generateApi.js`. The structure of the library can be updated by making changes to that file and rerunning the generation:

```bash
node generateApi.js
```