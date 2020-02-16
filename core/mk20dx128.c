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

#include "kinetis.h"
#include <errno.h>


// Flash Security Setting. On Teensy 3.2, you can lock the MK20 chip to prevent
// anyone from reading your code.  You CAN still reprogram your Teensy while
// security is set, but the bootloader will be unable to respond to auto-reboot
// requests from Arduino. Pressing the program button will cause a full chip
// erase to gain access, because the bootloader chip is locked out.  Normally,
// erase occurs when uploading begins, so if you press the Program button
// accidentally, simply power cycling will run your program again.  When
// security is locked, any Program button press causes immediate full erase.
// Special care must be used with the Program button, because it must be made
// accessible to initiate reprogramming, but it must not be accidentally
// pressed when Teensy Loader is not being used to reprogram.  To set lock the
// security change this to 0xDC.  Teensy 3.0 and 3.1 do not support security lock.
#define FSEC 0xDE

// Flash Options
#define FOPT 0xF9


extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _estack;
//extern void __init_array_start(void);
//extern void __init_array_end(void);



extern int main (void);
void ResetHandler(void);
void _init_Teensyduino_internal_(void) __attribute__((noinline));
void __libc_init_array(void);


void fault_isr(void)
{
	while (1) {
		// keep polling some communication while in fault
		// mode, so we don't completely die.
		if (SIM_SCGC4 & SIM_SCGC4_USBOTG) usb_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART0) uart0_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART1) uart1_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART2) uart2_status_isr();
	}
}

void unused_isr(void)
{
	fault_isr();
}

void nmi_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void hard_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void memmanage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void bus_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void usage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void svcall_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void debugmonitor_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pendablesrvreq_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void systick_isr(void);

void dma_ch0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch4_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch5_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch6_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch7_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch8_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch9_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch10_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch11_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch12_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch13_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch14_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch15_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void mcm_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void randnum_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void flash_cmd_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void flash_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void low_voltage_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void wakeup_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void watchdog_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void sdhc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_timer_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void enet_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void i2s0_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void uart0_lon_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void lpuart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void adc0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void adc1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmt_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void rtc_alarm_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void rtc_seconds_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pit_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pdb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_charge_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void usbhs_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usbhs_phy_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void dac0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dac1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tsi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void mcg_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void lptmr_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porta_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porte_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portcd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void software_isr(void)		__attribute__ ((weak, alias("unused_isr")));

__attribute__ ((section(".dmabuffers"), used, aligned(512)))
void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);

__attribute__ ((section(".vectors"), used))
void (* const _VectorsFlash[NVIC_NUM_INTERRUPTS+16])(void) =
{
	(void (*)(void))((unsigned long)&_estack),	//  0 ARM: Initial Stack Pointer
	ResetHandler,					//  1 ARM: Initial Program Counter
	nmi_isr,					//  2 ARM: Non-maskable Interrupt (NMI)
	hard_fault_isr,					//  3 ARM: Hard Fault
	memmanage_fault_isr,				//  4 ARM: MemManage Fault
	bus_fault_isr,					//  5 ARM: Bus Fault
	usage_fault_isr,				//  6 ARM: Usage Fault
	fault_isr,					//  7 --
	fault_isr,					//  8 --
	fault_isr,					//  9 --
	fault_isr,					// 10 --
	svcall_isr,					// 11 ARM: Supervisor call (SVCall)
	debugmonitor_isr,				// 12 ARM: Debug Monitor
	fault_isr,					// 13 --
	pendablesrvreq_isr,				// 14 ARM: Pendable req serv(PendableSrvReq)
	systick_isr,					// 15 ARM: System tick timer (SysTick)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_ch4_isr,					// 20 DMA channel 4 transfer complete
	dma_ch5_isr,					// 21 DMA channel 5 transfer complete
	dma_ch6_isr,					// 22 DMA channel 6 transfer complete
	dma_ch7_isr,					// 23 DMA channel 7 transfer complete
	dma_ch8_isr,					// 24 DMA channel 8 transfer complete
	dma_ch9_isr,					// 25 DMA channel 9 transfer complete
	dma_ch10_isr,					// 26 DMA channel 10 transfer complete
	dma_ch11_isr,					// 27 DMA channel 11 transfer complete
	dma_ch12_isr,					// 28 DMA channel 12 transfer complete
	dma_ch13_isr,					// 29 DMA channel 13 transfer complete
	dma_ch14_isr,					// 30 DMA channel 14 transfer complete
	dma_ch15_isr,					// 31 DMA channel 15 transfer complete
	dma_error_isr,					// 32 DMA error interrupt channel
	mcm_isr,					// 33 MCM
	flash_cmd_isr,					// 34 Flash Memory Command complete
	flash_error_isr,				// 35 Flash Read collision
	low_voltage_isr,				// 36 Low-voltage detect/warning
	wakeup_isr,					// 37 Low Leakage Wakeup
	watchdog_isr,					// 38 Both EWM and WDOG interrupt
	randnum_isr,					// 39 Random Number Generator
	i2c0_isr,					// 40 I2C0
	i2c1_isr,					// 41 I2C1
	spi0_isr,					// 42 SPI0
	spi1_isr,					// 43 SPI1
	i2s0_tx_isr,					// 44 I2S0 Transmit
	i2s0_rx_isr,					// 45 I2S0 Receive
	unused_isr,					// 46 --
	uart0_status_isr,				// 47 UART0 status
	uart0_error_isr,				// 48 UART0 error
	uart1_status_isr,				// 49 UART1 status
	uart1_error_isr,				// 50 UART1 error
	uart2_status_isr,				// 51 UART2 status
	uart2_error_isr,				// 52 UART2 error
	uart3_status_isr,				// 53 UART3 status
	uart3_error_isr,				// 54 UART3 error
	adc0_isr,					// 55 ADC0
	cmp0_isr,					// 56 CMP0
	cmp1_isr,					// 57 CMP1
	ftm0_isr,					// 58 FTM0
	ftm1_isr,					// 59 FTM1
	ftm2_isr,					// 60 FTM2
	cmt_isr,					// 61 CMT
	rtc_alarm_isr,					// 62 RTC Alarm interrupt
	rtc_seconds_isr,				// 63 RTC Seconds interrupt
	pit0_isr,					// 64 PIT Channel 0
	pit1_isr,					// 65 PIT Channel 1
	pit2_isr,					// 66 PIT Channel 2
	pit3_isr,					// 67 PIT Channel 3
	pdb_isr,					// 68 PDB Programmable Delay Block
	usb_isr,					// 69 USB OTG
	usb_charge_isr,					// 70 USB Charger Detect
	unused_isr,					// 71 --
	dac0_isr,					// 72 DAC0
	mcg_isr,					// 73 MCG
	lptmr_isr,					// 74 Low Power Timer
	porta_isr,					// 75 Pin detect (Port A)
	portb_isr,					// 76 Pin detect (Port B)
	portc_isr,					// 77 Pin detect (Port C)
	portd_isr,					// 78 Pin detect (Port D)
	porte_isr,					// 79 Pin detect (Port E)
	software_isr,					// 80 Software interrupt
	spi2_isr,					// 81 SPI2
	uart4_status_isr,				// 82 UART4 status
	uart4_error_isr,				// 83 UART4 error
	unused_isr,					// 84 --
	unused_isr,					// 85 --
	cmp2_isr,					// 86 CMP2
	ftm3_isr,					// 87 FTM3
	dac1_isr,					// 88 DAC1
	adc1_isr,					// 89 ADC1
	i2c2_isr,					// 90 I2C2
	can0_message_isr,				// 91 CAN OR'ed Message buffer (0-15)
	can0_bus_off_isr,				// 92 CAN Bus Off
	can0_error_isr,					// 93 CAN Error
	can0_tx_warn_isr,				// 94 CAN Transmit Warning
	can0_rx_warn_isr,				// 95 CAN Receive Warning
	can0_wakeup_isr,				// 96 CAN Wake Up
	sdhc_isr,					// 97 SDHC
	enet_timer_isr,					// 98 Ethernet IEEE1588 Timers
	enet_tx_isr,					// 99 Ethernet Transmit
	enet_rx_isr,					// 100 Ethernet Receive
	enet_error_isr,					// 101 Ethernet Error
	lpuart0_status_isr,				// 102 LPUART
	tsi0_isr,					// 103 TSI0
	tpm1_isr,					// 104 FTM1
	tpm2_isr,					// 105 FTM2
	usbhs_phy_isr,					// 106 USB-HS Phy
	i2c3_isr,					// 107 I2C3
	cmp3_isr,					// 108 CMP3
	usbhs_isr,					// 109 USB-HS
	can1_message_isr,				// 110 CAN OR'ed Message buffer (0-15)
	can1_bus_off_isr,				// 111 CAN Bus Off
	can1_error_isr,					// 112 CAN Error
	can1_tx_warn_isr,				// 113 CAN Transmit Warning
	can1_rx_warn_isr,				// 114 CAN Receive Warning
	can1_wakeup_isr,				// 115 CAN Wake Up
};


__attribute__ ((section(".flashconfig"), used))
const uint8_t flashconfigbytes[16] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, FSEC, FOPT, 0xFF, 0xFF
};

static inline void startup_preinit();

static void startup_default_early_hook(void) {
	WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
}
static void startup_default_late_hook(void) {}
void startup_early_hook(void)		__attribute__ ((weak, alias("startup_default_early_hook")));
void startup_late_hook(void)		__attribute__ ((weak, alias("startup_default_late_hook")));


#if defined(__PURE_CODE__) || !defined(__OPTIMIZE__) || defined(__clang__)
// cases known to compile too large for 0-0x400 memory region
__attribute__ ((optimize("-Os")))
#else
// hopefully all others fit into startup section (below 0x400)
__attribute__ ((section(".startup"),optimize("-Os")))
#endif
void ResetHandler(void)
{
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	// programs using the watchdog timer or needing to initialize hardware as
	// early as possible can implement startup_early_hook()
	startup_early_hook();

	// enable clocks to always-used peripherals
	SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
	SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;
	SCB_CPACR = 0x00F00000;
	LMEM_PCCCR = 0x85000003;

	// release I/O pins hold, if we woke up from VLLS mode
	if (PMC_REGSC & PMC_REGSC_ACKISO) PMC_REGSC |= PMC_REGSC_ACKISO;

    // since this is a write once register, make it visible to all F_CPU's
    // so we can into other sleep modes in the future at any speed
	SMC_PMPROT = SMC_PMPROT_AHSRUN | SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;

	// startup_preinit();

	// hardware always starts in FEI mode
	//  C1[CLKS] bits are written to 00
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] bit is written to 0
	// MCG_SC[FCDIV] defaults to divide by two for internal ref clock
	// I tried changing MSG_SC to divide by 1, it didn't work for me
    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
	// enable osc, 8-32 MHz range, low power mode
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
	// switch to crystal as clock source, FLL input = 16 MHz / 512
	MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
	// wait for crystal oscillator to begin
	while ((MCG_S & MCG_S_OSCINIT0) == 0) ;
	// wait for FLL to use oscillator
	while ((MCG_S & MCG_S_IREFST) != 0) ;
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) ;

	main();

	while (1) ;
}

static inline void startup_preinit() {
	uint32_t *src = &_etext;
	uint32_t *dest = &_sdata;

	// TODO: do this while the PLL is waiting to lock....
	while (dest < &_edata) *dest++ = *src++;
	dest = &_sbss;
	while (dest < &_ebss) *dest++ = 0;
}

char *__brkval = (char *)&_ebss;

#ifndef STACK_MARGIN
#define STACK_MARGIN  8192
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void * _sbrk(int incr)
{
	char *prev, *stack;

	prev = __brkval;
	if (incr != 0) {
		__asm__ volatile("mov %0, sp" : "=r" (stack) ::);
		if (prev + incr >= stack - STACK_MARGIN) {
			errno = ENOMEM;
			return (void *)-1;
		}
		__brkval = prev + incr;
	}
	return prev;
}

#include <sys/stat.h>

__attribute__((weak))
void abort(void)
{
	while (1) ;
}

#pragma GCC diagnostic pop

int nvic_execution_priority(void)
{
	uint32_t priority=256;
	uint32_t primask, faultmask, basepri, ipsr;

	// full algorithm in ARM DDI0403D, page B1-639
	// this isn't quite complete, but hopefully good enough
	__asm__ volatile("mrs %0, faultmask\n" : "=r" (faultmask)::);
	if (faultmask) return -1;
	__asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
	if (primask) return 0;
	__asm__ volatile("mrs %0, ipsr\n" : "=r" (ipsr)::);
	if (ipsr) {
		if (ipsr < 16) priority = 0; // could be non-zero
		else priority = NVIC_GET_PRIORITY(ipsr - 16);
	}
	__asm__ volatile("mrs %0, basepri\n" : "=r" (basepri)::);
	if (basepri > 0 && basepri < priority) priority = basepri;
	return priority;
}
