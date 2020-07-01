#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "core_pins.h"
#include "pins_arduino.h"

#include "usb_serial.h"
#include "usb_undef.h" // do not allow usb_desc.h stuff to leak to user programs

const int ledPin = 13;

extern "C" volatile uint32_t systick_millis_count;
void systick_isr(void)
{
	systick_millis_count++;
}

// #define SERIAL1_TX_BUFFER_SIZE     64 // number of outgoing bytes to buffer
// #define SERIAL1_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
// #define RTS_HIGH_WATERMARK (SERIAL1_RX_BUFFER_SIZE-24) // RTS requests sender to pause
// #define RTS_LOW_WATERMARK  (SERIAL1_RX_BUFFER_SIZE-38) // RTS allows sender to resume
// #define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

// static volatile uint8_t tx_buffer[SERIAL1_TX_BUFFER_SIZE];
// static volatile uint8_t rx_buffer[SERIAL1_RX_BUFFER_SIZE];
// static volatile uint8_t transmitting = 0;
// static volatile uint8_t tx_buffer_head = 0;
// static volatile uint8_t tx_buffer_tail = 0;
// static volatile uint8_t rx_buffer_head = 0;
// static volatile uint8_t rx_buffer_tail = 0;

// // UART0 and UART1 are clocked by F_CPU, UART2 is clocked by F_BUS
// // UART0 has 8 byte fifo, UART1 and UART2 have 1 byte buffer

// #define C2_ENABLE		UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
// #define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
// #define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
// #define C2_TX_INACTIVE		C2_ENABLE
// #define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))

// void serial_putchar(uint32_t c)
// {
// 	uint32_t head, n;

// 	if (!(SIM_SCGC4 & SIM_SCGC4_UART0)) return;
// 	head = tx_buffer_head;
// 	if (++head >= SERIAL1_TX_BUFFER_SIZE) head = 0;
// 	while (tx_buffer_tail == head) {
// 		int priority = nvic_execution_priority();
// 		if (priority <= IRQ_PRIORITY) {
// 			if ((UART0_S1 & UART_S1_TDRE)) {
// 				uint32_t tail = tx_buffer_tail;
// 				if (++tail >= SERIAL1_TX_BUFFER_SIZE) tail = 0;
// 				n = tx_buffer[tail];
// 				UART0_D = n;
// 				tx_buffer_tail = tail;
// 			}
// 		} else if (priority >= 256) {
// 			// yield();
// 		}
// 	}
// 	tx_buffer[head] = c;
// 	transmitting = 1;
// 	tx_buffer_head = head;
// 	UART0_C2 = C2_TX_ACTIVE;
// }

// void serial_print(const char *p)
// {
// 	while (*p) {
// 		char c = *p++;
// 		if (c == '\n') serial_putchar('\r');
// 		serial_putchar(c);
// 	}
// }

// void serial_write(const char *p, int n)
// {
//     while (n > 0) {
//         char c = *p++;
// 		if (c == '\n') serial_putchar('\r');
// 		serial_putchar(c);
//         n--;
//     }
// }

// void serial_begin(uint32_t divisor)
// {
// 	SIM_SCGC4 |= SIM_SCGC4_UART0;	// turn on clock, TODO: use bitband
// 	rx_buffer_head = 0;
// 	rx_buffer_tail = 0;
// 	tx_buffer_head = 0;
// 	tx_buffer_tail = 0;
// 	transmitting = 0;
//     CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
//     CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
// 	if (divisor < 32) divisor = 32;
// 	UART0_BDH = (divisor >> 13) & 0x1F;
// 	UART0_BDL = (divisor >> 5) & 0xFF;
// 	UART0_C4 = divisor & 0x1F;
// 	UART0_C1 = UART_C1_ILT;
// 	UART0_TWFIFO = 2; // tx watermark, causes S1_TDRE to set
// 	UART0_RWFIFO = 4; // rx watermark, causes S1_RDRF to set
// 	UART0_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
// 	UART0_C2 = C2_TX_INACTIVE;
// 	NVIC_SET_PRIORITY(IRQ_UART0_STATUS, IRQ_PRIORITY);
// 	NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
// }

// void uart0_status_isr(void)
// {
// 	uint32_t head, tail, n;
// 	uint8_t c;
// 	uint32_t newhead;
// 	uint8_t avail;

// 	if (UART0_S1 & (UART_S1_RDRF | UART_S1_IDLE)) {
// 		__disable_irq();
// 		avail = UART0_RCFIFO;
// 		if (avail == 0) {
// 			// The only way to clear the IDLE interrupt flag is
// 			// to read the data register.  But reading with no
// 			// data causes a FIFO underrun, which causes the
// 			// FIFO to return corrupted data.  If anyone from
// 			// Freescale reads this, what a poor design!  There
// 			// write should be a write-1-to-clear for IDLE.
// 			c = UART0_D;
// 			// flushing the fifo recovers from the underrun,
// 			// but there's a possible race condition where a
// 			// new character could be received between reading
// 			// RCFIFO == 0 and flushing the FIFO.  To minimize
// 			// the chance, interrupts are disabled so a higher
// 			// priority interrupt (hopefully) doesn't delay.
// 			// TODO: change this to disabling the IDLE interrupt
// 			// which won't be simple, since we already manage
// 			// which transmit interrupts are enabled.
// 			UART0_CFIFO = UART_CFIFO_RXFLUSH;
// 			__enable_irq();
// 		} else {
// 			__enable_irq();
// 			head = rx_buffer_head;
// 			tail = rx_buffer_tail;
// 			do {
// 				n = UART0_D;
// 				newhead = head + 1;
// 				if (newhead >= SERIAL1_RX_BUFFER_SIZE) newhead = 0;
// 				if (newhead != tail) {
// 					head = newhead;
// 					rx_buffer[head] = n;
// 				}
// 			} while (--avail > 0);
// 			rx_buffer_head = head;
// 		}
// 	}
// 	c = UART0_C2;
// 	if ((c & UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE)) {
// 		head = tx_buffer_head;
// 		tail = tx_buffer_tail;
// 		do {
// 			if (tail == head) break;
// 			if (++tail >= SERIAL1_TX_BUFFER_SIZE) tail = 0;
// 			avail = UART0_S1;
// 			n = tx_buffer[tail];
// 			UART0_D = n;
// 		} while (UART0_TCFIFO < 8);
// 		tx_buffer_tail = tail;
// 		if (UART0_S1 & UART_S1_TDRE) UART0_C2 = C2_TX_COMPLETING;
// 	}

// 	if ((c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {
// 		transmitting = 0;
// 		UART0_C2 = C2_TX_INACTIVE;
// 	}
// }

// char messages_buf1[1024] = {0}, messages_buf2[1024] = {0};
// char * messages, * messages_alt;
// volatile int messages_pos = 0, messages_alt_pos = 0;

int main() {
    volatile uint32_t *config = portConfigRegister(ledPin);

    *portModeRegister(ledPin) = 1;
    *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *config &= ~PORT_PCR_ODE;

    // CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK;
    CORE_PIN13_PORTSET = CORE_PIN13_BITMASK;

    // messages = &messages_buf1[0];
    // messages_alt = &messages_buf2[0];

    // serial_begin(BAUD2DIV(115200));

    while (1) {
        // serial_print(".");
        // char * tmp1 = messages;
        // int tmp2 = messages_pos;
        // messages = messages_alt;
        // messages_pos = messages_alt_pos;
        // messages_alt = tmp1;
        // messages_alt_pos = tmp2;

        // serial_write(messages_alt, messages_alt_pos);

        usb_serial_write(".", 1);

        CORE_PIN13_PORTSET = CORE_PIN13_BITMASK;
        delay(500);
        CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK;
        delay(500);
    }
}