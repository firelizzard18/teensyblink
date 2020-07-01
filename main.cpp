#include <Arduino.h>

const int ledPin = 13;

extern "C" volatile uint32_t systick_millis_count;
void systick_isr(void)
{
	  systick_millis_count++;
}

int main() {
    volatile uint32_t *config = portConfigRegister(ledPin);

    *portModeRegister(ledPin) = 1;
    *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *config &= ~PORT_PCR_ODE;

    while (1) {
        Serial.print(".");

        CORE_PIN13_PORTSET = CORE_PIN13_BITMASK;
        delay(500);
        CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK;
        delay(500);
    }
}