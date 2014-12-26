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
	/* System clock dividers A-PER(2), B-PER2(1), C-PER4(1) - all 24MHz */
	CLK.PSCTRL = CLK_PSADIV_2_gc | CLK_PSBCDIV_1_1_gc;

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

	/* use DFLL RC32KHz clock source 
	DFLLRC32M.COMP1 = 0x1B;
	DFLLRC32M.COMP2 = 0xB7;	
	OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc; */

	/* DFLL enable */
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

	/* disable IOREG protection */
	CCP = CCP_IOREG_gc;
	/* then switch clock to INT RC 32MHz source and lock system clock settings */
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	/* disable unneeded 2MHz int. RC osc */
	OSC.CTRL &= ~OSC_RC2MEN_bm;
}
