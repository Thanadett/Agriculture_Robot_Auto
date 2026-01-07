#include <Arduino.h>
const int PIN_STEP = 32, PIN_DIR = 25, PIN_EN = 26;

void setup()
{
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_EN, OUTPUT);

    // ENA ของ TB6600 ส่วนใหญ่ LOW = Enable
    digitalWrite(PIN_EN, LOW); // เปิดไดรเวอร์
    delay(20);

    digitalWrite(PIN_DIR, HIGH); // ล็อกทิศเดียว
}

void loop()
{
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(800);
}
