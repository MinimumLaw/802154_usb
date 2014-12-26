/*
 * usbdev.c
 *
 * Created: 01.12.2014 14:35:03
 *  Author: alex
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h> /* bzero */

#include "usbdev.h"

uint8_t		outControlTransfer[1024];
uint16_t	outControlTransferSize;

uint8_t control_transfer_out_buff[64];
uint8_t control_transfer_in_buff[64];
usb_sram_layout usb_sram;

ISR(USB_TRNCOMPL_vect)
{
	USARTD0_DATA = 'T';
	if(USB_INTFLAGSBSET & USB_TRNIF_bm) { /* translation packet */
		USB_INTFLAGSBCLR = USB_TRNIF_bm; /* ACK */
	};
	if(USB_INTFLAGSBSET & USB_SETUPIF_bm) { 	/* setup packet */
		USB_INTFLAGSBCLR = USB_SETUPIF_bm; /* ACK */
	};
}

ISR(USB_BUSEVENT_vect)
{
	if(USB_INTFLAGSASET & USB_SOFIF_bm) { /* start of frame */
//		USARTD0_DATA = 'F';
		USB_INTFLAGSACLR = USB_SOFIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_SUSPENDIF_bm) { /* suspend */
		USARTD0_DATA = '-';
		USB_INTFLAGSACLR = USB_SUSPENDIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_RESUMEIF_bm) { /* resume */
		USARTD0_DATA = '+';
		USB_INTFLAGSACLR = USB_RESUMEIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_RSTIF_bm) { /* reset */
		USARTD0_DATA = 'R';
		/* init USB sram layout */
		memset(&usb_sram, 0, sizeof(usb_sram_layout));
		USB.EPPTR = (uint16_t)&(usb_sram.ep);
		/* ep0 now enabled */
		usb_sram.ep[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_BUFSIZE_64_gc;
		usb_sram.ep[0].out.STATUS = USB_EP_BUSNACK0_bm;
		usb_sram.ep[0].out.DATAPTR = (uint16_t)&control_transfer_out_buff;
		usb_sram.ep[0].out.AUXDATA = 64; // sizeof(control_transfer_out_buff);
		usb_sram.ep[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_BUFSIZE_64_gc;
		usb_sram.ep[0].in.DATAPTR = (uint16_t)&control_transfer_in_buff;
		usb_sram.ep[0].in.CNT = 64;
		usb_sram.ep[0].in.STATUS = USB_EP_BUSNACK0_bm;
		/* reset device address */
		USB.ADDR = 0;
		USB_INTFLAGSACLR = USB_RSTIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_CRCIF_bm) { /* CRC error */
//		USARTD0_DATA = 'X';
		USB_INTFLAGSACLR = USB_CRCIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_UNFIF_bm) { /* underflow */
//		USARTD0_DATA = 'U';
		USB_INTFLAGSACLR = USB_UNFIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_OVFIF_bm) { /* overflow */
//		USARTD0_DATA = 'O';
		USB_INTFLAGSACLR = USB_OVFIF_bm; /* ACK */
	};
	if(USB_INTFLAGSASET & USB_STALLIF_bm) {	/* stall */
//		USARTD0_DATA = '=';
		USB_INTFLAGSACLR = USB_STALLIF_bm; /* ACK */
	};
}

void usbdev_init(void)
{
	/* USB pins factory calibration apply */
	USB.CAL0 = PRODSIGNATURES_USBCAL0;
	USB.CAL1 = PRODSIGNATURES_USBCAL1;
	
	/* disable IOREG protection */
	CCP = CCP_IOREG_gc;
	/* USB clock use int RC 48MHz output */
	CLK.USBCTRL = CLK_USBPSDIV_1_gc | CLK_USBSRC_RC32M_gc | CLK_USBSEN_bm;	/* USB clocks enable */
	/* enable USB and DMA module clock */
	PR.PRGEN &= ~PR_USB_bm;
	
	/* USB Endpoint operations depend on DMA controller, enable them */
	PR.PRGEN &= ~PR_DMA_bm;
	DMA.CTRL |= DMA_RESET_bm;
	DMA.CTRL |= DMA_ENABLE_bm;

	USB_CTRLA = USB_SPEED_bm | (USB_DEVICE_MAX_EP);
	USB_CTRLB = USB_URESUME_bm | USB_ATTACH_bm;
	
	/* enable ALL interrupts (medium level) */
	USB_INTCTRLA = USB_SOFIE_bm | USB_BUSEVIE_bm | USB_BUSERRIE_bm | USB_STALLIE_bm | USB_INTLVL_MED_gc;
	USB_INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;

	USB_CTRLA |= USB_ENABLE_bm;
}
