/*
 * usart.c
 *
 * Created: 05.12.2014 11:14:32
 *  Author: alex
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

#include "sysclock.h"
#include "usart.h"

ISR(USARTD0_RXC_vect)
{
	USARTD0_DATA = USARTD0_DATA;
}

ISR(USARTD0_TXC_vect)
{
}

void usart_init(void)
{
	/* 0. Enable clock source for USERTD0 */
	PR.PRPD &= ~PR_USART0_bm;
	/* 1. Set the TxD pin value high, and optionally set the XCK pin low. */
	PORTD.OUTSET = PIN3_bm;
	/* 2. Set the TxD and optionally the XCK pin as output. */
	PORTD.DIRSET = PIN3_bm;
	/* 3. Set the baud rate and frame format. */
	/* SysClk = 24MHz, => BSEL = 832, BSCALE=-6 => 115246,1 (0,04% error) */
	USARTD0.BAUDCTRLA = 0b01000000;	USARTD0.BAUDCTRLB = 0b10010011;
	/* 4. Set the mode of operation (enables XCK pin output in synchronous mode). */
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	/* 5. Enable the transmitter or the receiver, depending on the usage. */
	USARTD0.CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;
}
