#include <Arduino.h>

const int ledPin = 13;

int main(void)
{
    pinMode(ledPin, OUTPUT);
    while (1) {
        digitalWriteFast(ledPin, HIGH);
        delay(100);
        digitalWriteFast(ledPin, LOW);
        delay(100);
    }
}


volatile uint32_t systick_millis_count;
void systick_isr(void)
{
	systick_millis_count++;
}