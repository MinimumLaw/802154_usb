/*
 * usbdev.h
 *
 * Created: 01.12.2014 14:35:55
 *  Author: alex
 */ 


#ifndef _INCLUDE_USBDEV_H_
#define _INCLUDE_USBDEV_H_

#define USB_DEVICE_MAX_EP	2

typedef struct {
	uint32_t fifo[USB_DEVICE_MAX_EP + 1];
	struct {
		USB_EP_t out;
		USB_EP_t in;
	} ep[(USB_DEVICE_MAX_EP + 1)];
	uint16_t frame_number;
} usb_sram_layout __attribute__((aligned(16)));

extern void usbdev_init(void);

#endif /* _INCLUDE_USBDEV_H_ */