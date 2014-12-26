#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

#include "sysclock.h"
#include "rtc.h"
#include "usbdev.h"
#include "usart.h"
#include "at86rf212.h"

rf_dev *rf;

void hw_init(void)
{
	cli();
	disable_all_peripherial();
	rtc_init();
	SLEEP_CTRL = SLEEP_SMODE_IDLE_gc;
	sysclock_init();
	/* enable all interrupt level */
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	// spi_init();
	usart_init();
	usbdev_init();
	rf = at86rf212_init();
	sei();
	rf->fops->reset(rf);
}

int main(void)
{
	hw_init();
	
    while(1){
        rf->fops->update_status(rf);
    }
}
