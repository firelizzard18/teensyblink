/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "core_pins.h"
#include "pins_arduino.h"

#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))
//#define GPIO_SET_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 1)
//#define GPIO_CLR_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 0)
const struct digital_pin_bitband_and_config_table_struct digital_pin_to_info_PGM[] = {
	{GPIO_BITBAND_PTR(CORE_PIN0_PORTREG, CORE_PIN0_BIT), &CORE_PIN0_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN1_PORTREG, CORE_PIN1_BIT), &CORE_PIN1_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN2_PORTREG, CORE_PIN2_BIT), &CORE_PIN2_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN3_PORTREG, CORE_PIN3_BIT), &CORE_PIN3_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN4_PORTREG, CORE_PIN4_BIT), &CORE_PIN4_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN5_PORTREG, CORE_PIN5_BIT), &CORE_PIN5_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN6_PORTREG, CORE_PIN6_BIT), &CORE_PIN6_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN7_PORTREG, CORE_PIN7_BIT), &CORE_PIN7_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN8_PORTREG, CORE_PIN8_BIT), &CORE_PIN8_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN9_PORTREG, CORE_PIN9_BIT), &CORE_PIN9_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN10_PORTREG, CORE_PIN10_BIT), &CORE_PIN10_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN11_PORTREG, CORE_PIN11_BIT), &CORE_PIN11_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN12_PORTREG, CORE_PIN12_BIT), &CORE_PIN12_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN13_PORTREG, CORE_PIN13_BIT), &CORE_PIN13_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN14_PORTREG, CORE_PIN14_BIT), &CORE_PIN14_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN15_PORTREG, CORE_PIN15_BIT), &CORE_PIN15_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN16_PORTREG, CORE_PIN16_BIT), &CORE_PIN16_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN17_PORTREG, CORE_PIN17_BIT), &CORE_PIN17_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN18_PORTREG, CORE_PIN18_BIT), &CORE_PIN18_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN19_PORTREG, CORE_PIN19_BIT), &CORE_PIN19_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN20_PORTREG, CORE_PIN20_BIT), &CORE_PIN20_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN21_PORTREG, CORE_PIN21_BIT), &CORE_PIN21_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN22_PORTREG, CORE_PIN22_BIT), &CORE_PIN22_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN23_PORTREG, CORE_PIN23_BIT), &CORE_PIN23_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN24_PORTREG, CORE_PIN24_BIT), &CORE_PIN24_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN25_PORTREG, CORE_PIN25_BIT), &CORE_PIN25_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN26_PORTREG, CORE_PIN26_BIT), &CORE_PIN26_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN27_PORTREG, CORE_PIN27_BIT), &CORE_PIN27_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN28_PORTREG, CORE_PIN28_BIT), &CORE_PIN28_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN29_PORTREG, CORE_PIN29_BIT), &CORE_PIN29_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN30_PORTREG, CORE_PIN30_BIT), &CORE_PIN30_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN31_PORTREG, CORE_PIN31_BIT), &CORE_PIN31_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN32_PORTREG, CORE_PIN32_BIT), &CORE_PIN32_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN33_PORTREG, CORE_PIN33_BIT), &CORE_PIN33_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN34_PORTREG, CORE_PIN34_BIT), &CORE_PIN34_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN35_PORTREG, CORE_PIN35_BIT), &CORE_PIN35_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN36_PORTREG, CORE_PIN36_BIT), &CORE_PIN36_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN37_PORTREG, CORE_PIN37_BIT), &CORE_PIN37_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN38_PORTREG, CORE_PIN38_BIT), &CORE_PIN38_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN39_PORTREG, CORE_PIN39_BIT), &CORE_PIN39_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN40_PORTREG, CORE_PIN40_BIT), &CORE_PIN40_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN41_PORTREG, CORE_PIN41_BIT), &CORE_PIN41_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN42_PORTREG, CORE_PIN42_BIT), &CORE_PIN42_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN43_PORTREG, CORE_PIN43_BIT), &CORE_PIN43_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN44_PORTREG, CORE_PIN44_BIT), &CORE_PIN44_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN45_PORTREG, CORE_PIN45_BIT), &CORE_PIN45_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN46_PORTREG, CORE_PIN46_BIT), &CORE_PIN46_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN47_PORTREG, CORE_PIN47_BIT), &CORE_PIN47_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN48_PORTREG, CORE_PIN48_BIT), &CORE_PIN48_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN49_PORTREG, CORE_PIN49_BIT), &CORE_PIN49_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN50_PORTREG, CORE_PIN50_BIT), &CORE_PIN50_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN51_PORTREG, CORE_PIN51_BIT), &CORE_PIN51_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN52_PORTREG, CORE_PIN52_BIT), &CORE_PIN52_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN53_PORTREG, CORE_PIN53_BIT), &CORE_PIN53_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN54_PORTREG, CORE_PIN54_BIT), &CORE_PIN54_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN55_PORTREG, CORE_PIN55_BIT), &CORE_PIN55_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN56_PORTREG, CORE_PIN56_BIT), &CORE_PIN56_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN57_PORTREG, CORE_PIN57_BIT), &CORE_PIN57_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN58_PORTREG, CORE_PIN58_BIT), &CORE_PIN58_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN59_PORTREG, CORE_PIN59_BIT), &CORE_PIN59_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN60_PORTREG, CORE_PIN60_BIT), &CORE_PIN60_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN61_PORTREG, CORE_PIN61_BIT), &CORE_PIN61_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN62_PORTREG, CORE_PIN62_BIT), &CORE_PIN62_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN63_PORTREG, CORE_PIN63_BIT), &CORE_PIN63_CONFIG},
};

static void dummy_isr() {};

typedef void (*voidFuncPtr)(void);
static void port_A_isr(void) __attribute__ ((section(".fastrun"), noinline, noclone ));
static void port_B_isr(void) __attribute__ ((section(".fastrun"), noinline, noclone ));
static void port_C_isr(void) __attribute__ ((section(".fastrun"), noinline, noclone ));
static void port_D_isr(void) __attribute__ ((section(".fastrun"), noinline, noclone ));
static void port_E_isr(void) __attribute__ ((section(".fastrun"), noinline, noclone ));

voidFuncPtr isr_table_portA[CORE_MAX_PIN_PORTA+1] = { [0 ... CORE_MAX_PIN_PORTA] = dummy_isr };
voidFuncPtr isr_table_portB[CORE_MAX_PIN_PORTB+1] = { [0 ... CORE_MAX_PIN_PORTB] = dummy_isr };
voidFuncPtr isr_table_portC[CORE_MAX_PIN_PORTC+1] = { [0 ... CORE_MAX_PIN_PORTC] = dummy_isr };
voidFuncPtr isr_table_portD[CORE_MAX_PIN_PORTD+1] = { [0 ... CORE_MAX_PIN_PORTD] = dummy_isr };
voidFuncPtr isr_table_portE[CORE_MAX_PIN_PORTE+1] = { [0 ... CORE_MAX_PIN_PORTE] = dummy_isr };

// The Pin Config Register is used to look up the correct interrupt table
// for the corresponding port.
static inline voidFuncPtr* getIsrTable(volatile uint32_t *config) {
	voidFuncPtr* isr_table = NULL;
	if(&PORTA_PCR0 <= config && config <= &PORTA_PCR31) isr_table = isr_table_portA;
	else if(&PORTB_PCR0 <= config && config <= &PORTB_PCR31) isr_table = isr_table_portB;
	else if(&PORTC_PCR0 <= config && config <= &PORTC_PCR31) isr_table = isr_table_portC;
	else if(&PORTD_PCR0 <= config && config <= &PORTD_PCR31) isr_table = isr_table_portD;
	else if(&PORTE_PCR0 <= config && config <= &PORTE_PCR31) isr_table = isr_table_portE;
	return isr_table;
}

inline uint32_t getPinIndex(volatile uint32_t *config) {
	uintptr_t v = (uintptr_t) config;
	// There are 32 pin config registers for each port, each port starting at a round address.
	// They are spaced 4 bytes apart.
	return (v % 128) / 4;
}

void attachInterruptVector(enum IRQ_NUMBER_t irq, void (*function)(void))
{
	_VectorsRam[irq + 16] = function;
}

void attachInterrupt(uint8_t pin, void (*function)(void), int mode)
{
	volatile uint32_t *config;
	uint32_t cfg, mask;

	if (pin >= CORE_NUM_DIGITAL) return;
	switch (mode) {
	  case CHANGE:	mask = 0x0B; break;
	  case RISING:	mask = 0x09; break;
	  case FALLING:	mask = 0x0A; break;
	  case LOW:	mask = 0x08; break;
	  case HIGH:	mask = 0x0C; break;
	  default: return;
	}
	mask = (mask << 16) | 0x01000000;
	config = portConfigRegister(pin);
	if ((*config & 0x00000700) == 0) {
		// for compatibility with programs which depend
		// on AVR hardware default to input mode.
		pinMode(pin, INPUT);
	}
	attachInterruptVector(IRQ_PORTA, port_A_isr);
	attachInterruptVector(IRQ_PORTB, port_B_isr);
	attachInterruptVector(IRQ_PORTC, port_C_isr);
	attachInterruptVector(IRQ_PORTD, port_D_isr);
	attachInterruptVector(IRQ_PORTE, port_E_isr);
	voidFuncPtr* isr_table = getIsrTable(config);
	if(!isr_table) return;
	uint32_t pin_index = getPinIndex(config);
	__disable_irq();
	cfg = *config;
	cfg &= ~0x000F0000;		// disable any previous interrupt
	*config = cfg;
	isr_table[pin_index] = function;	// set the function pointer
	cfg |= mask;
	*config = cfg;			// enable the new interrupt
	__enable_irq();
}

void detachInterrupt(uint8_t pin)
{
	volatile uint32_t *config;

	config = portConfigRegister(pin);
	voidFuncPtr* isr_table = getIsrTable(config);
	if(!isr_table) return;
	uint32_t pin_index = getPinIndex(config);
	__disable_irq();
	*config = ((*config & ~0x000F0000) | 0x01000000);
	isr_table[pin_index] = dummy_isr;
	__enable_irq();
}


typedef void (*voidFuncPtr)(void);

// Using CTZ instead of CLZ is faster, since it allows more efficient bit
// clearing and fast indexing into the pin ISR table.
#define PORT_ISR_FUNCTION_CLZ(port_name) \
	static void port_ ## port_name ## _isr(void) {            \
		uint32_t isfr = PORT ## port_name ##_ISFR;            \
		PORT ## port_name ##_ISFR = isfr;                     \
		voidFuncPtr* isr_table = isr_table_port ## port_name; \
		uint32_t bit_nr;                                      \
		while(isfr) {                                         \
			bit_nr = __builtin_ctz(isfr);                     \
			isr_table[bit_nr]();                              \
			isfr = isfr & (isfr-1);                           \
			if(!isfr) return;                                 \
		}                                                     \
	}
// END PORT_ISR_FUNCTION_CLZ

PORT_ISR_FUNCTION_CLZ(A)
PORT_ISR_FUNCTION_CLZ(B)
PORT_ISR_FUNCTION_CLZ(C)
PORT_ISR_FUNCTION_CLZ(D)
PORT_ISR_FUNCTION_CLZ(E)

unsigned long rtc_get(void)
{
	return RTC_TSR;
}

void rtc_set(unsigned long t)
{
	RTC_SR = 0;
	RTC_TPR = 0;
	RTC_TSR = t;
	RTC_SR = RTC_SR_TCE;
}

// adjust is the amount of crystal error to compensate, 1 = 0.1192 ppm
// For example, adjust = -100 is slows the clock by 11.92 ppm
//
void rtc_compensate(int adjust)
{
	uint32_t comp, interval, tcr;

	// This simple approach tries to maximize the interval.
	// Perhaps minimizing TCR would be better, so the
	// compensation is distributed more evenly across
	// many seconds, rather than saving it all up and then
	// altering one second up to +/- 0.38%
	if (adjust >= 0) {
		comp = adjust;
		interval = 256;
		while (1) {
			tcr = comp * interval;
			if (tcr < 128*256) break;
			if (--interval == 1) break;
		}
		tcr = tcr >> 8;
	} else {
		comp = -adjust;
		interval = 256;
		while (1) {
			tcr = comp * interval;
			if (tcr < 129*256) break;
			if (--interval == 1) break;
		}
		tcr = tcr >> 8;
		tcr = 256 - tcr;
	}
	RTC_TCR = ((interval - 1) << 8) | tcr;
}

extern void usb_init(void);


// create a default PWM at the same 488.28 Hz as Arduino Uno

#define F_TIMER F_BUS

#if F_TIMER == 128000000
#define DEFAULT_FTM_MOD (65536 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 120000000
#define DEFAULT_FTM_MOD (61440 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 108000000
#define DEFAULT_FTM_MOD (55296 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 96000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 90000000
#define DEFAULT_FTM_MOD (46080 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 80000000
#define DEFAULT_FTM_MOD (40960 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 72000000
#define DEFAULT_FTM_MOD (36864 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 64000000
#define DEFAULT_FTM_MOD (65536 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 60000000
#define DEFAULT_FTM_MOD (61440 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 56000000
#define DEFAULT_FTM_MOD (57344 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 54000000
#define DEFAULT_FTM_MOD (55296 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 48000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 40000000
#define DEFAULT_FTM_MOD (40960 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 36000000
#define DEFAULT_FTM_MOD (36864 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 24000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 16000000
#define DEFAULT_FTM_MOD (32768 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 8000000
#define DEFAULT_FTM_MOD (16384 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 4000000
#define DEFAULT_FTM_MOD (8192 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 2000000
#define DEFAULT_FTM_MOD (4096 - 1)
#define DEFAULT_FTM_PRESCALE 0
#endif

//void init_pins(void)
__attribute__((noinline))
void _init_Teensyduino_internal_(void)
{
	NVIC_ENABLE_IRQ(IRQ_PORTA);
	NVIC_ENABLE_IRQ(IRQ_PORTB);
	NVIC_ENABLE_IRQ(IRQ_PORTC);
	NVIC_ENABLE_IRQ(IRQ_PORTD);
	NVIC_ENABLE_IRQ(IRQ_PORTE);
	//SIM_SCGC6 |= SIM_SCGC6_FTM0;	// TODO: use bitband for atomic read-mod-write
	//SIM_SCGC6 |= SIM_SCGC6_FTM1;
	FTM0_CNT = 0;
	FTM0_MOD = DEFAULT_FTM_MOD;
	FTM0_C0SC = 0x28; // MSnB:MSnA = 10, ELSnB:ELSnA = 10
	FTM0_C1SC = 0x28;
	FTM0_C2SC = 0x28;
	FTM0_C3SC = 0x28;
	FTM0_C4SC = 0x28;
	FTM0_C5SC = 0x28;
	FTM0_C6SC = 0x28;
	FTM0_C7SC = 0x28;
	FTM3_C0SC = 0x28;
	FTM3_C1SC = 0x28;
	FTM3_C2SC = 0x28;
	FTM3_C3SC = 0x28;
	FTM3_C4SC = 0x28;
	FTM3_C5SC = 0x28;
	FTM3_C6SC = 0x28;
	FTM3_C7SC = 0x28;
	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
	FTM1_CNT = 0;
	FTM1_MOD = DEFAULT_FTM_MOD;
	FTM1_C0SC = 0x28;
	FTM1_C1SC = 0x28;
	FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
	FTM2_CNT = 0;
	FTM2_MOD = DEFAULT_FTM_MOD;
	FTM2_C0SC = 0x28;
	FTM2_C1SC = 0x28;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
	FTM3_CNT = 0;
	FTM3_MOD = DEFAULT_FTM_MOD;
	FTM3_C0SC = 0x28;
	FTM3_C1SC = 0x28;
	FTM3_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
	SIM_SCGC2 |= SIM_SCGC2_TPM1;
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(2);
	TPM1_CNT = 0;
	TPM1_MOD = 32767;
	TPM1_C0SC = 0x28;
	TPM1_C1SC = 0x28;
	TPM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	// analog_init();
	// for background about this startup delay, please see these conversations
	// https://forum.pjrc.com/threads/36606-startup-time-(400ms)?p=113980&viewfull=1#post113980
	// https://forum.pjrc.com/threads/31290-Teensey-3-2-Teensey-Loader-1-24-Issues?p=87273&viewfull=1#post87273
#if TEENSYDUINO >= 142
	delay(25);
	usb_init();
	delay(275);
#else
	delay(50);
	usb_init();
	delay(350);
#endif
}

// SOPT4 is SIM select clocks?
// FTM is clocked by the bus clock, either 24 or 48 MHz
// input capture can be FTM1_CH0, CMP0 or CMP1 or USB start of frame
// 24 MHz with reload 49152 to match Arduino's speed = 488.28125 Hz

// the systick interrupt is supposed to increment this at 1 kHz rate
volatile uint32_t systick_millis_count = 0;

//uint32_t systick_current, systick_count, systick_istatus;  // testing only

uint32_t micros(void)
{
	uint32_t count, current, istatus;

	__disable_irq();
	current = SYST_CVR;
	count = systick_millis_count;
	istatus = SCB_ICSR;	// bit 26 indicates if systick exception pending
	__enable_irq();
	 //systick_current = current;
	 //systick_count = count;
	 //systick_istatus = istatus & SCB_ICSR_PENDSTSET ? 1 : 0;
	if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
	current = ((F_CPU / 1000) - 1) - current;
	return count * 1000 + current / (F_CPU / 1000000);
}

void delay(uint32_t ms)
{
	uint32_t start = micros();

	if (ms > 0) {
		while (1) {
			while ((micros() - start) >= 1000) {
				ms--;
				if (ms == 0) return;
				start += 1000;
			}
		}
	}
}
