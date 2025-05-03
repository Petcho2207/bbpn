#include <Arduino.h>
#include "BQ79600.h"

BQ79600 bq(Serial2);

void setup() {
    Serial.begin(115200);
    bq.begin();
    bq.wakeUp();
}

void loop() {
    // test loop
}
