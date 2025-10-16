/*
 * HAL_SPI.h
 *
 *  Created on: Oct 7, 2025
 *      Author: jagadeep.g
 */

#ifndef STM32H7XX_HAL_INC_HAL_SPI_H_
#define STM32H7XX_HAL_INC_HAL_SPI_H_

#include <HAL_BUS.h>
#include <stdint.h>

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CFG1;
	volatile uint32_t CFG2;
	volatile uint32_t IER;
	volatile uint32_t SR;
	volatile uint32_t IFCR;
	uint32_t reserved;
	volatile uint32_t TXDR;
	uint32_t reserved2[3];
	volatile uint32_t RXDR;
	uint32_t reserved3[3];
	volatile uint32_t CRCPOLY;
	volatile uint32_t TXCRC;
	volatile uint32_t RXCRC;
	volatile uint32_t UDRDR;
	volatile uint32_t I2SCFGR;
}SPI_reg_t;

typedef struct{
	uint8_t DeviceMode;			//@SPI_DeviceMode
	uint8_t BusConfig;			//@SPI_BUsConfig
	uint8_t SclkSpeed;			//@SPI_CLKSpeed
	uint8_t DFF;				//@SPI_DFF
	uint8_t CPOL;				//@CPOL
	uint8_t CPHA;				//@CPHA
	uint8_t SSM;				//@SPI_SSM
}SPI_config_t;

typedef struct{
	SPI_reg_t 		*SPIx;			//Holds the base of the SPI peripheral registers
	SPI_config_t 	SPI_config;		//Holds the configuration values
	uint8_t			*pTxBuffer;		//transmit buffer address
	uint8_t			*pRxBuffer;		//receive buffer address
	uint32_t		TxLen;			//Transmit length
	uint32_t		RxLen;			//receive length
	uint8_t			TxState;		//stores the busy state of SPI peripheral during transmit
	uint8_t			RxState;		//stores the busy state during receive
}SPI_handle_t;

#define SPI1	((SPI_reg_t *)SPI1_BASE_ADDR)
#define SPI2	((SPI_reg_t *)SPI2_BASE_ADDR)
#define SPI3	((SPI_reg_t *)SPI3_BASE_ADDR)
#define SPI4	((SPI_reg_t *)SPI4_BASE_ADDR)
#define SPI5	((SPI_reg_t *)SPI5_BASE_ADDR)
#define SPI6	((SPI_reg_t *)SPI6_BASE_ADDR)

#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1<<12));	(RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1LRSTR |= (1<<14));	(RCC->APB1LRSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1LRSTR |= (1<<15));	(RCC->APB1LRSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1<<13));	(RCC->APB2RSTR &= ~(1<<13));}while(0)
#define SPI5_REG_RESET()	do{(RCC->APB2RSTR |= (1<<20));	(RCC->APB2RSTR &= ~(1<<20));}while(0)
#define SPI6_REG_RESET()	do{(RCC->APB4RSTR |= (1<<5));	(RCC->APB4RSTR &= ~(1<<5));}while(0)

/*
 * @SPI_DeviceMode
 */
#define SPI_MODE_MASTER		1
#define SPI_MODE_SLAVE		0

/*
 * @SPI_BUsConfig
 */
#define SPI_BUS_FD			0
#define SPI_BUS_HD			3
#define SPI_BUS_SIMPLEX_TX	1
#define SPI_BUS_SIMPLEX_RX	2

/*
 * @SPI_CLKSpeed
 */
#define SPI_CLK_DIV2		0
#define SPI_CLK_DIV4		1
#define SPI_CLK_DIV8		2
#define SPI_CLK_DIV16		3
#define SPI_CLK_DIV32		4
#define SPI_CLK_DIV64		5
#define SPI_CLK_DIV128		6
#define SPI_CLK_DIV256		7


/*
 * @SPI_DFF
 */
#define SPI_DFF_4BITS		0x03
#define SPI_DFF_5BITS		0x04
#define SPI_DFF_6BITS		0x05
#define SPI_DFF_7BITS		0x06
#define SPI_DFF_8BITS		0x07
#define SPI_DFF_9BITS		0x08
#define SPI_DFF_10BITS		0x09
#define SPI_DFF_11BITS		0x0A
#define SPI_DFF_12BITS		0x0B
#define SPI_DFF_13BITS		0x0C
#define SPI_DFF_14BITS		0x0D
#define SPI_DFF_15BITS		0x0E
#define SPI_DFF_16BITS		0x0F
#define SPI_DFF_17BITS		0x10
#define SPI_DFF_18BITS		0x11
#define SPI_DFF_19BITS		0x12
#define SPI_DFF_20BITS		0x13
#define SPI_DFF_21BITS		0x14
#define SPI_DFF_22BITS		0x15
#define SPI_DFF_23BITS		0x16
#define SPI_DFF_24BITS		0x17
#define SPI_DFF_25BITS		0x18
#define SPI_DFF_26BITS		0x19
#define SPI_DFF_27BITS		0x1A
#define SPI_DFF_28BITS		0x1B
#define SPI_DFF_29BITS		0x1C
#define SPI_DFF_30BITS		0x1D
#define SPI_DFF_31BITS		0x1E
#define SPI_DFF_32BITS		0x1F

/*
 * SPI interrupt status flags
 */
#define SPI_READY			0
#define SPI_TX_INT_BUSY		1
#define SPI_RX_INT_BUSY		2


/*
 * @CPOL
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 * @SPI_SSM
 */
#define SPI_SSM_SW			1
#define SPI_SSM_HW			0


/*
 * bit position of SPI peripherals in CFG and SR registers
 */
#define SPI_CFG2_MASTER_POS		22
#define SPI_CFG2_COMM_POS		17
#define SPI_CFG2_CPHA_POS		24
#define SPI_CFG2_CPOL_POS		25
#define SPI_CFG2_SSM_POS		26
#define SPI_CFG2_SSOE_POS		29
#define SPI_CFG2_AFCNTR_POS		31
#define SPI_CFG1_MBR_POS		28
#define SPI_CFG1_DFF_POS		0

#define SPI_SR_TXC_POS			12
#define SPI_SR_OVR_POS			6
#define SPI_SR_RXP_POS			0
#define SPI_SR_TXP_POS			1

#define SPI_CR1_SPE_POS			0
#define SPI_CR1_IOLOCK_POS		16
#define SPI_CR1_SSI_POS			12
#define SPI_CR1_CSTART_POS		9

#define SPI_CR2_TSIZE_POS		0

#define SPI_IER_OVRIE_POS		6
#define SPI_IER_TXPIE_POS		1
#define SPI_IER_RXPIE_POS		0

#define SPI_IFCR_OVRC_POS		6


/*
 * possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

/*
 * clock selection macros for SPI45
 */
#define SPI45_APB_CK			0
#define SPI45_PLL2_CK			1
#define SPI45_PLL3_CK 			2
#define SPI45_HSI_CK			3
#define SPI45_CSI_CK			4
#define SPI45_HSE_CK			5
/*
 * clock selection macros for SPI123
 */
#define SPI123_PLL1_CK			0
#define SPI123_PLL2_CK			1
#define SPI123_PLL3_CK 			2
#define SPI123_I2S_CK			3
#define SPI123_PER_CK			4

/*
 * clock selection macros for SPI6
 */
#define SPI6_RCC_CK				0
#define SPI6_PLL2_CK			1
#define SPI6_PLL3_CK			2
#define SPI6_HSI_CK				3
#define SPI6_CSI_CK				4
#define SPI6_HSE_CK				5

#define D2CCIP1R_SPI123SEL_POS	12
#define D2CCIP1R_SPI45SEL_POS	16
#define D3CCIPR_SPI6SEL_POS		28
/*
 * SPI init and deinit
 * SPIx		 :-	base address of SPI registers (SPI1-SPI6)
 * spi_config:-	parameters for config
 */
void HAL_SPI_init(SPI_handle_t *pSPI_handle);

void HAL_SPI_deinit(SPI_reg_t *SPIx);

/*
 * SPI Clock setup
 * SPIx	:-	base address of SPI registers (SPI1-SPI6)
 * type :-  ENABLE or DISABLE
 */
void HAL_SPI_CLK_control(SPI_reg_t *SPIx, uint8_t type);

/*
 * SPI123 Clock Select
 * type :- SPI123_PLL1_CK,SPI123_PLL2_CK,SPI123_PLL3_CK , SPI123_I2S_CK, SPI123_PER_CK
 */
void SPI123_CLK_SEL(uint8_t type);

/*
 * SPI45 Clock Select
 * type :- SPI45_APB_CK,SPI45_PLL2_CK,SPI45_PLL3_CK , SPI45_HSI_CK, SPI45_CSI_CK,SPI45_HSE_CK
 */
void SPI45_CLK_SEL(uint8_t type);


/*
 * Data send and receive
 */
void SPI_send(SPI_reg_t *SPIx, uint8_t *pTxbuffer, uint16_t len);

void SPI_recv(SPI_reg_t *SPIx, uint8_t *pRxbuffer, uint32_t len);


/*
 * IRQ config and handling of SPI
 */
void HAL_GPIO_irq_config(uint8_t irq_number, uint8_t type);

void HAL_GPIO_irq_priority(uint8_t irq_number, uint8_t priority);

uint8_t SPI_send_INT(SPI_handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_recv_INT(SPI_handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_IRQ_handling(SPI_handle_t *pHandle);

/*
 *  API to ENABLE or DISABLE SPIx peripheral
 */
void SPI_Peripheral_Control(SPI_reg_t *SPIx, uint8_t type);

void SPI_Appliaction_callback(SPI_handle_t *Handle, uint8_t event);

void SPI_Clear_OVR(SPI_reg_t *SPIx);

void SPI_Close_Tx(SPI_handle_t *pHandle);

void SPI_Close_Rx(SPI_handle_t *pHandle);

#endif /* STM32H7XX_HAL_INC_HAL_SPI_H_ */
