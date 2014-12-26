/*
 * at86rf212.h
 *
 * Created: 24.12.2014 10:17:53
 *  Author: alex
 */ 


#ifndef _INCLUDE_AT86RF212_H_
#define _INCLUDE_AT86RF212_H_

#include <stddef.h>

#define BIT(X) (1<<X)

#define RF_SPI_PORT		PORTC
#define RF_SPI_PR_REG	PRPC
#define RF_SPI_DEVICE	SPIC

#define RF_SPI_CS_PORT	PORTC
#define RF_SPI_CS_PIN	PIN4_bm

#define RF_SPI_RST_PORT	PORTC
#define RF_SPI_RST_PIN	PIN0_bm

#define RF_SPI_SLP_PORT	PORTC
#define RF_SPI_SLP_PIN	PIN3_bm

#define RF_SPI_IRQ_PORT	PORTC
#define RF_SPI_IRQ_REG	PIN2CTRL
#define RF_SPI_IRQ_PIN	PIN2_bm

#define RF_ISR_VECTOR	PORTC_INT0_vect

struct at86rf_device;
struct at86rf_isr;
struct at86rf_fops;

#define DUMMY_EMPTY_REG	0x00
#define TRX_STATUS_REG	0x01
#define TRX_STATE_REG	0x02
#define TRX_CTRL_0_REG	0x03
#define TRX_CTRL_1_REG	0x04
#define PHY_TX_PWR_REG	0x05
#define PHY_RSSI_REG	0x06
#define PHY_ED_LEVEL_REG	0x07
#define PHY_CC_CCA_REG	0x08

#define IRQ_MASK_REG	0x0E
#define IRQ_STATUS_REG	0x0F

#define TRX_CTRL_PA_EXT_EN			BIT(7)
#define TRX_CTRL_IRQ2_EXT_EN		BIT(6)
#define TRX_CTRL_TX_AUTO_CRC_ON		BIT(5)
#define TRX_CTRL_RX_BL_CTRL			BIT(4)
#define TRX_CTRL_SPI_ZERO			(0<<2)
#define TRX_CTRL_SPI_TRX_STATUS		(1<<2)
#define TRX_CTRL_SPI_PHY_RSSI		(2<<2)
#define TRX_CTRL_SPI_IRQ_STATUS		(3<<2)
#define TRX_CTRL_IRQ_MASK_MODE		BIT(1)
#define TRX_CTRL_IRQ_POLARITY		BIT(0)


#define TRX_STATUS_P_ON					0x00
#define TRX_STATUS_BUSY_RX				0x01
#define TRX_STATUS_BUSY_TX				0x02
#define TRX_STATUS_RX_ON				0x06
#define TRX_STATUS_TRX_OFF				0x08
#define TRX_STATUS_PLL_ON				0x09
#define TRX_STATUS_SLEEP				0x0F
#define TRX_STATUS_BUSY_RX_AACK_ON		0x11
#define TRX_STATUS_BUSY_TX_ARET_ON		0x12
#define TRX_STATUS_RX_AACK_ON			0x16
#define TRX_STATUS_TX_ARET_ON			0x19
#define TRX_STATUS_RX_ON_NOCLK			0x1C
#define TRX_STATUS_RX_AACK_NOCLK		0x1D
#define TRX_STATUS_BUSY_RX_AACK_NOCLK	0x1E
#define TRX_STATUS_IN_PROGRESS			0x1F

#define TRX_COMMAND_NOP					0x00
#define TRX_COMMAND_TX_START			0x02
#define TRX_COMMAND_FORCE_TRX_OFF		0x03
#define TRX_COMMAND_FORCE_PLL_ON		0x04
#define TRX_COMMAND_RX_ON				0x06
#define TRX_COMMAND_TRX_OFF				0x08
#define TRX_COMMAND_PLL_ON				0x09
#define TRX_COMMAND_RX_AACK_ON			0x16
#define TRX_COMMAND_RX_ARET_ON			0x19

struct at86rf_device {
	SPI_t					*spi_dev;
	struct at86rf_isr		*isr;
	struct at86rf_fops		*fops;
	uint8_t					state;
	uint8_t recv_buff[132]; // PHY_STATUS + PHR(LEN) + PSDU(127 MAX) + LQI + ED + RX_STATUS
};

struct at86rf_isr {
	void	(*battery_low)(struct at86rf_device* dev);			/* IRQ7 */
	void	(*access_violation)(struct at86rf_device* dev);		/* IRQ6 */
	void	(*address_match)(struct at86rf_device* dev);		/* IRQ5 */
	void	(*set_params)(struct at86rf_device* dev);			/* IRQ4 */
	void	(*complite)(struct at86rf_device* dev);				/* IRQ3 */
	void	(*rx_start)(struct at86rf_device* dev);				/* IRQ2 */
	void	(*pll_unlock)(struct at86rf_device* dev);			/* IRQ1 */
	void	(*pll_lock)(struct at86rf_device* dev);				/* IRQ0 */
};

struct at86rf_fops {
	void	(*reset)(struct at86rf_device* dev);
	void	(*update_status)(struct at86rf_device* dev);
	uint8_t	(*frame_send)(struct at86rf_device* dev, uint8_t *frame, size_t len);
	uint8_t	(*frame_recv_callback)(struct at86rf_device* dev, uint8_t *frame, size_t len, uint8_t lqi);
};

typedef struct at86rf_device rf_dev;

/***********************
 * Function prptotypes *
 ***********************/
rf_dev* at86rf212_init(void);

#endif /* _INCLUDE_AT86RF212_H_ */