/*
 * sysclock.c
 *
 * Created: 01.12.2014 13:29:44
 *  Author: alex
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include "sysclock.h"

void disable_all_peripherial (void)
{
	/* disable all peripheral clocks */
	PR.PRGEN = 0xFF;;
	PR.PRPA = 0xFF;
	PR.PRPB = 0xFF;
	PR.PRPC = 0xFF;
	PR.PRPD = 0xFF;
	PR.PRPE = 0xFF;
	PR.PRPF = 0xFF;	
}

uint8_t read_calibration_byte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_word(index);
	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}

void sysclock_init(void)
{
	/* disable IOREG protection */
	CCP = CCP_IOREG_gc;
	/* System clock dividers A-PER(1), B-PER2(1), C-PER4(1) - all 32MHz (PLL) */
	CLK.PSCTRL = CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;

	/* Enable int. 32KHz RC osc. and wait them for ready */
	OSC.CTRL |= OSC_RC32KEN_bm;
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));

	/* Enable int. 2MHz RC osc. and wait them for ready */
	OSC.CTRL |= OSC_RC2MEN_bm;
	while(!(OSC.STATUS & OSC_RC2MRDY_bm));
	/* when enable DFLL for int. 2Mhz RC osc. */
	DFLLRC2M.CTRL = DFLL_ENABLE_bm;

	/* PLL multiple int. 2MHz RC osc. by 16 - output is 32MHz (maximum) */
	OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | 16;
	OSC.CTRL = OSC_PLLEN_bm;
	while(!(OSC.STATUS & OSC_PLLRDY_bm));

	OSC.CTRL &= ~OSC_RC32MEN_bm;
	/* calibrate int 32MHz RC osc to 48Mhz clock output */
	DFLLRC32M.CALA = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSCA));
	DFLLRC32M.CALB = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));

	/* enable int 32Mhz osc. and wait them for ready */
	OSC.CTRL |= OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));
	
	/* use DFLL USB SOF for 48MHz clock stability */
	DFLLRC32M.COMP1 = 0x80;
	DFLLRC32M.COMP2 = 0xBB;
	OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc;

	/* DFLL enable */
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

	/* disable IOREG protection */
	CCP = CCP_IOREG_gc;
	/* then switch clock to PLL 32MHz source and lock system clock settings */
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}
