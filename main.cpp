#include <Arduino.h>

const int ledPin = 13;

extern "C" int main(void)
{
    pinMode(ledPin, OUTPUT);
    while (1) {
        digitalWriteFast(ledPin, HIGH);
        delay(500);
        digitalWriteFast(ledPin, LOW);
        delay(500);
    }
}

