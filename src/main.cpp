#include "BQ79600.h"

HardwareSerial mySerial(1);  // ใช้ UART1 ของ ESP32

void setup() {
    mySerial.begin(1000000, SERIAL_8N1, 16, 17);  // Rx = 16, Tx = 17
    BQ79600 bms(mySerial, 1000000, 17);           // tx_pin = 17

    bms.Initialize();
}