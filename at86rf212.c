/*
 * at86rf212.c
 *
 * Created: 24.12.2014 10:13:40
 *  Author: alex
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stddef.h>
#include "at86rf212.h"

rf_dev	at86rf212;

inline void activate_cs(void) { RF_SPI_CS_PORT.OUTCLR = RF_SPI_CS_PIN; }
inline void deactivate_cs(void) { RF_SPI_CS_PORT.OUTSET = RF_SPI_CS_PIN; }

inline uint8_t spi_xchng_byte(rf_dev *dev, uint8_t out)
{
	dev->spi_dev->DATA = out;
	while(!(dev->spi_dev->STATUS & 0x80));	
	return dev->spi_dev->DATA;
}

uint8_t rf_reg_read(rf_dev *dev, uint8_t reg)
{
	uint8_t tmp = reg | 0x80;
	
	activate_cs();
	dev->state = spi_xchng_byte(dev, tmp);// & 0x1F;
	tmp = spi_xchng_byte(dev, 0x00); /* dummy write */
	deactivate_cs();
	
	return tmp;
}

void rf_reg_write(rf_dev *dev, uint8_t reg, uint8_t val)
{
	uint8_t tmp = reg | 0xC0;
	
	activate_cs();
	dev->state = spi_xchng_byte(dev, tmp);// & 0x1F;
	tmp = spi_xchng_byte(dev, val); /* dummy read */
	deactivate_cs();
}

void rf_frame_write(rf_dev* dev, uint8_t* frame, uint8_t len)
{
	uint8_t tmp = 0x60;
	
	activate_cs();
	dev->state = spi_xchng_byte(dev, tmp);// & 0x1F;
	tmp = spi_xchng_byte(dev, len); /* dummy read */
	while(len--)
		tmp = spi_xchng_byte(dev, *frame++); /* dummy read */
	deactivate_cs();
}

void rf_frame_read(rf_dev* dev, uint8_t* buff)
{
	uint8_t tmp = 0x20;
	
	activate_cs();
	dev->state = spi_xchng_byte(dev, tmp);// & 0x1F;
	tmp = spi_xchng_byte(dev, 0x00); /* PHR (PSDU LEN) in tmp */
	*buff++ = tmp; /* put PHR into buffer */
	while(tmp--)
		*buff++ = spi_xchng_byte(dev, 0x00); /* data read */
	*buff++ = spi_xchng_byte(dev, 0x00); /* LQI read */
	*buff++ = spi_xchng_byte(dev, 0x00); /* ED read */
	*buff = spi_xchng_byte(dev, 0x00); /* RX_STATUS read */
	deactivate_cs();
}

/****************************************
 * AT86RF212 Interrupt service routines *
 ****************************************/
void rf_pll_lock(rf_dev* dev)
{ /* pll enabled => switch to transmitt state */
	rf_reg_write(&at86rf212, TRX_STATE_REG, TRX_COMMAND_RX_ON);
}

void rf_pll_unlock(rf_dev* dev)
{ /* device switch into receive state */
}

void rf_rx_start(rf_dev* dev)
{ /* starting receive packet */
}

void rf_complite(rf_dev* dev)
{ /* last transfer complite */
}

void rf_set_params(rf_dev* dev)
{ /* Chip powered and clocked */
}

void rf_address_match(rf_dev* dev)
{ /* address matched */
}

void rf_access_violation(rf_dev* dev)
{ /* frame access violation*/
}

void rf_battery_low(rf_dev* dev)
{ /* battery voltage to low */
}

struct at86rf_isr rf_isr = {
	.pll_lock = &rf_pll_lock,
	.pll_unlock = &rf_pll_unlock,
	.rx_start = &rf_rx_start,
	.complite = &rf_complite,
	.set_params = &rf_set_params,
	.address_match = &rf_address_match,
	.access_violation = &rf_access_violation,
	.battery_low = &rf_battery_low,
};

ISR(RF_ISR_VECTOR)
{
	uint8_t status;

	/**/
	while((status = rf_reg_read(&at86rf212, IRQ_STATUS_REG)) == 0);
	
	if((status & 0x01) && (at86rf212.isr->pll_lock != NULL))
		at86rf212.isr->pll_lock(&at86rf212);
	if((status & 0x02) && (at86rf212.isr->pll_unlock != NULL))
		at86rf212.isr->pll_unlock(&at86rf212);
	if((status & 0x04) && (at86rf212.isr->rx_start != NULL))
		at86rf212.isr->rx_start(&at86rf212);
	if((status & 0x08) && (at86rf212.isr->complite != NULL))
		at86rf212.isr->complite(&at86rf212);
	if((status & 0x10) && (at86rf212.isr->set_params != NULL))
		at86rf212.isr->set_params(&at86rf212);
	if((status & 0x20) && (at86rf212.isr->address_match != NULL))
		at86rf212.isr->address_match(&at86rf212);
	if((status & 0x40) && (at86rf212.isr->access_violation != NULL))
		at86rf212.isr->access_violation(&at86rf212);
	if((status & 0x80) && (at86rf212.isr->battery_low != NULL))
		at86rf212.isr->battery_low(&at86rf212);
}

/************************************************************************/
/* File operations for AT86RF212 chips                                  */
/************************************************************************/
/*
 * Hardware reset chip
 */
void rf_reset(rf_dev* dev)
{
	RF_SPI_RST_PORT.OUTCLR = RF_SPI_RST_PIN; /* RESET and low (active) */
	{ volatile uint16_t tmp=0x2fff; while(tmp--); }
	RF_SPI_RST_PORT.OUTSET = RF_SPI_RST_PIN; /* RESET and high (inactive) */	
	{ volatile uint16_t tmp=0x2fff; while(tmp--); }

	rf_reg_write(dev, TRX_CTRL_1_REG, TRX_CTRL_SPI_TRX_STATUS | TRX_CTRL_TX_AUTO_CRC_ON | TRX_CTRL_PA_EXT_EN);
	rf_reg_write(dev, IRQ_MASK_REG, 0xFF); /* All interrupts enabled */

	{ volatile uint16_t tmp=0x2fff; while(tmp--); }
	rf_reg_write(dev, TRX_STATE_REG, TRX_COMMAND_PLL_ON);
}

/*
 * Configure basic RF parametrizes (frequency, power, modulations)
 */
int8_t rf_configure(rf_dev* dev, uint8_t page, uint8_t channel)
{
	switch(page) {
		/*
		 * Channel page 0 and 2 standard IEEE802.15.4 compliant
		 */
		case 0: /* ch. 0..10 BPSK */
			if(channel > 10)
				return -EBADCHANNELL;
			else if(channel == 0) {
				/* 868MHz BPSK 20kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0)); /* BPSK 20kb/s */
			} else {
				/* 906...924MHz BPSK 40kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x04); /* BPSK 40kb/s */
			};
			break;
		case 2: /* ch. 0..10 O-QPSK */
			if(channel > 10)
			return -EBADCHANNELL;
			else if(channel == 0) {
				/* 868MHz O-QPSK 100kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x08); /* O-QPSK 100kb/s */
			} else {
				/* 906...924MHz O-QPSK 250kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0C); /* O-QPSK 250kb/s */
			};
			break;
		/*
		 * Channel page 10 and 11 standard IEEE802.15.4 compliant frequency,
		 * but have extended PSDU speed (chip vendor extension)
		 */
		case 10: /* standard ch. 0..10 O-QPSK - extended PSDU data rate */
			if(channel > 10)
				return -EBADCHANNELL;
			else if(channel == 0) {
				/* 868MHz O-QPSK 200kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x09); /* O-QPSK 200kb/s */
			} else {
				/* 906...924MHz O-QPSK 500kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0D); /* O-QPSK 500kb/s */
			};
			break;
		case 11: /* standard ch. 0..10 O-QPSK - extended PSDU data rate */
			if(channel > 10)
				return -EBADCHANNELL;
			else if(channel == 0) {
				/* 868MHz O-QPSK 400kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0A); /* O-QPSK 400kb/s */
			} else {
				/* 906...924MHz O-QPSK 1000kb/s */
				rf_reg_write(dev,CC_CTRL_1_REG, 0); // USE PHY_CC_CCA Channel field
				rf_reg_write(dev,PHY_CC_CCA_REG, (rf_reg_read(dev,PHY_CC_CCA_REG) & 0xE0) | channel);
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0E); /* O-QPSK 1000kb/s */
			};
			break;
		/*
		 * Channel page 12,13,14 and later have 26 channel with freq 880..905Mhz
		 * OAO Radioavionica and Russian RGSN extension
		 */
		case 12: /* CC_BAND=5, CC_NUMBER=47..72, O-QPSK 250kb/s */
			rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0C); /* O-QPSK 250kb/s */
			break;
		case 13: /* CC_BAND=5, CC_NUMBER=47..72, O-QPSK 500kb/s */
			rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0D); /* O-QPSK 500kb/s */
			break;
		case 14: /* CC_BAND=5, CC_NUMBER=47..72, O-QPSK 1000kb/s */
			rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0E); /* O-QPSK 1000kb/s */
			break;
		/*
		 * Channel page 15,16,17 have 9 channels with freq 906..915Mhz
		 * and 10 channels with freq 925...935Mhz (0..8 and 9..19)
		 * OAO Radioavionica and Russian RGSN extension
		 */
		case 15: /* CC_BAND=5, O-QPSK 250kb/s */
			if(channel > 19)
				return -EBADCHANNELL;
			else if(channel > 8) {
				/* CC_NUMBER=73..82 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0C); /* O-QPSK 250kb/s */
			} else {
				/* CC_NUMBER=92..102 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0C); /* O-QPSK 250kb/s */
			};
		case 16: /* CC_BAND=5, O-QPSK 500kb/s */
			if(channel > 19)
				return -EBADCHANNELL;
			else if(channel > 8) {
				/* CC_NUMBER=73..82 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0D); /* O-QPSK 500kb/s */
			} else {
				/* CC_NUMBER=92..102 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0D); /* O-QPSK 500kb/s */
			};
		case 17: /* CC_BAND=5, O-QPSK 1000kb/s */
			if(channel > 19)
				return -EBADCHANNELL;
			else if(channel > 8) {
				/* CC_NUMBER=73..82 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0E); /* O-QPSK 1000kb/s */
			} else {
				/* CC_NUMBER=92..102 */
				rf_reg_write(dev,TRX_CTRL_2_REG, (rf_reg_read(dev,TRX_CTRL_2_REG) & 0xF0) | 0x0E); /* O-QPSK 1000kb/s */
		};
		default:
			return -EBADPAGE;
	};
	return ESUCCESS;
}

void rf_update_status(rf_dev* dev)
{ /* read unused register (0x00) simple update device status */
	rf_reg_read(dev, DUMMY_EMPTY_REG);
}

int8_t rf_frame_send(rf_dev* dev, uint8_t* frame, size_t len)
{ /* send frame to RF device (radio transmit) */
	return 0;
}

struct at86rf_fops	at86rf212_fops = {
	.reset = &rf_reset,
	.configure = &rf_configure,
	.update_status = &rf_update_status,
	.frame_send = &rf_frame_send,
};

/**********************************
 * Global initialization function *
 **********************************/
rf_dev* at86rf212_init(void)
{	
	PR.RF_SPI_PR_REG &= ~(PR_SPI_bm);
	
	RF_SPI_RST_PORT.DIRSET = RF_SPI_RST_PIN; /* RESET is output */
	RF_SPI_SLP_PORT.DIRSET = RF_SPI_SLP_PIN; /* SLP_TR is output */
	
	RF_SPI_SLP_PORT.OUTCLR = RF_SPI_SLP_PIN; /* SLP_TR low level */

	/* IRQ from transmitter */
	RF_SPI_IRQ_PORT.DIRCLR = RF_SPI_IRQ_PIN; /* IRQ pin is input */
	RF_SPI_IRQ_PORT.RF_SPI_IRQ_REG =  PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;
	RF_SPI_IRQ_PORT.INT0MASK = RF_SPI_IRQ_PIN;
	RF_SPI_IRQ_PORT.INTCTRL = PORT_INT0LVL_MED_gc;
	at86rf212.isr = &rf_isr;
	
	/* SPI */
	RF_SPI_CS_PORT.DIRSET = RF_SPI_CS_PIN; /* CS is output */
	RF_SPI_CS_PORT.OUTSET = RF_SPI_CS_PIN;
	RF_SPI_PORT.DIRSET = (PIN5_bm | PIN7_bm); /* MOSI & SCK - output */
	RF_SPI_PORT.DIRCLR = (PIN6_bm); /* MISO - input */

	RF_SPI_PORT.OUTSET = (PIN5_bm | PIN7_bm);
	RF_SPI_DEVICE.CTRL = SPI_MASTER_bm | SPI_ENABLE_bm | SPI_PRESCALER_DIV16_gc;

	at86rf212.spi_dev = &RF_SPI_DEVICE;
	at86rf212.fops = & at86rf212_fops;

	return &at86rf212;
}