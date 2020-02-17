// #include "core_pins.h"
// #include "pins_arduino.h"
#include "kinetis.h"

const int ledPin = 13;

int main(void)
{
    GPIOC_PDDR |= 1<<5;
    PORTC_PCR5 |= PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR5 &= ~PORT_PCR_ODE;

    while (1) {
        GPIOC_PCOR = 1<<5;
        for (int i = 0; i < 1<<24; i++) __asm__ volatile ("nop");
        GPIOC_PSOR = 1<<5;
        for (int i = 0; i < 1<<24; i++) __asm__ volatile ("nop");
    }
}

void __doNotStripVectors()
{
	_VectorsRam[16] = NULL;
}