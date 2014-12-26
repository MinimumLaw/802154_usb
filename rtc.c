/*
 * rtc.c
 *
 * Created: 08.12.2014 9:43:27
 *  Author: alex
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

ISR(RTC_COMP_vect)
{
	/* ToDo: place once per second code here */
	;;
	//USARTD0_DATA = '$';
}
ISR_ALIAS(RTC_OVF_vect, RTC_COMP_vect);

/************************************************
* REAL TIME CLOCK AND SYSTEM TIMER CLOCK SOURCE *
************************************************/
void rtc_init(void)
{
	PR.PRGEN &= ~ PR_RTC_bm; /* enable RTC clock */
	RTC.PER  = 65535; /* PER and COMP routine is a same */
	RTC.COMP = 32767; /* 32768 COMP and 65535 OVF caused */
	RTC.CNT = 0;      /* ISR once per second no CNT modify */
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC.INTCTRL = RTC_COMPINTLVL_MED_gc | RTC_OVFINTLVL_MED_gc;
	/* Then start int.32KHz RC osc. as RTC clock source */
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC32_gc | CLK_RTCEN_bm;
}
